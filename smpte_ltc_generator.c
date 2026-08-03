/*
 * ============================================================================
 * smpte_ltc_generator.c
 *
 * SMPTE Linear Time Code (LTC) generator for TI-RTOS (SYS/BIOS).
 *
 * Generates 80-bit biphase-mark-encoded LTC frames and outputs them by
 * toggling a GPIO pin from a hardware Timer interrupt (Hwi).
 *
 * THEORY OF OPERATION
 * --------------------
 * LTC is biphase-mark encoded:
 *   - Every bit period ALWAYS has a transition at the start of the period.
 *   - A '1' bit has an ADDITIONAL transition in the middle of the period.
 *   - A '0' bit has NO transition in the middle.
 *
 * To generate this with a timer, we run the timer at 2x the bit rate
 * ("half-bit" ticks). Each half-bit tick either forces a transition
 * (start-of-bit, always) or conditionally toggles at the half-bit point
 * only if the current bit is '1'.
 *
 * Frame rate example: 25 fps (EBU), 80 bits/frame => bit rate = 2000 bit/s
 * => half-bit rate = 4000 Hz => timer period = 250 us.
 *
 * For 30 fps (SMPTE non-drop): 80*30 = 2400 bit/s => half-bit = 4800 Hz
 * For 29.97 drop-frame, use the same 30fps structure with drop-frame flag
 * bit set and frame-number skipping logic (not shown here for brevity).
 *
 * LTC 80-BIT FRAME LAYOUT (25fps EBU, bit 0 first transmitted):
 *   bits 0-3   : Frame units (BCD)
 *   bits 4-7   : (unused/user bits 1)  -- simplified: kept 0 here
 *   bits 8-9   : Frame tens (BCD, 2 bits, max 3)
 *   bit  10    : Drop frame flag (0 for 25fps)
 *   bit  11    : Color frame flag
 *   bits 12-15 : user bits 2 (0 here)
 *   bits 16-19 : Seconds units (BCD)
 *   bits 20-23 : user bits 3 (0 here)
 *   bits 24-26 : Seconds tens (BCD, 3 bits, max 5)
 *   bit  27    : Flag bit (unused here)
 *   bits 28-31 : user bits 4 (0 here)
 *   bits 32-35 : Minutes units (BCD)
 *   bits 36-39 : user bits 5 (0 here)
 *   bits 40-42 : Minutes tens (BCD, 3 bits)
 *   bit  43    : Flag bit
 *   bits 44-47 : user bits 6 (0 here)
 *   bits 48-51 : Hours units (BCD)
 *   bits 52-55 : user bits 7 (0 here)
 *   bits 56-57 : Hours tens (BCD, 2 bits, max 2)
 *   bits 58-59 : Flag bits (reserved)
 *   bits 60-63 : user bits 8 (0 here)
 *   bits 64-79 : Sync word = 0x3FFD  (fixed bit pattern, MUST NOT be BCD'd)
 *
 * This file implements the full bit-packing + biphase transmission engine.
 * User-bit fields are left as 0; fill them in if you need embedded metadata.
 * ============================================================================
 */

#include <stdint.h>
#include <string.h>

/* TI-RTOS / SYS-BIOS headers */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>

/* TI-RTOS driver headers (board-specific: adjust include to your BSP) */
#include <ti/drivers/GPIO.h>
#include "Board.h"          /* Board_LTC_OUT must be defined in Board.h  */

/* ------------------------------------------------------------------------ */
/* Configuration                                                            */
/* ------------------------------------------------------------------------ */
#define LTC_FPS              25          /* frame rate: 25, 24, or 30      */
#define LTC_BITS_PER_FRAME   80
#define LTC_BIT_RATE         (LTC_BITS_PER_FRAME * LTC_FPS)     /* bits/s  */
#define LTC_HALFBIT_RATE_HZ  (LTC_BIT_RATE * 2)                 /* Hz      */

#define LTC_GPIO_OUT         Board_SMPTE_OUT   /* GPIO pin driving the LTC line */

/* Sync word for 80-bit LTC frame (bits 64-79), transmitted LSB first */
#define LTC_SYNC_WORD        0x3FFDu

/* ------------------------------------------------------------------------ */
/* Timecode state                                                           */
/* ------------------------------------------------------------------------ */
typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t frames;
} SMPTE_Time;

static volatile SMPTE_Time gTimecode = { 0, 0, 0, 0 };

/* ------------------------------------------------------------------------ */
/* Bit-transmission state (shared with ISR)                                 */
/* ------------------------------------------------------------------------ */
static uint8_t  gFrameBits[LTC_BITS_PER_FRAME]; /* 0/1 per bit, bit0 first  */
static volatile uint16_t gBitIndex   = 0;   /* which bit (0..79) is active  */
static volatile uint8_t  gHalfBitPos = 0;   /* 0 = start-of-bit, 1 = mid-bit*/
static volatile uint8_t  gLineLevel  = 0;   /* current output level 0/1    */

static Timer_Handle gTimerHandle;

/* ------------------------------------------------------------------------ */
/* Forward declarations                                                     */
/* ------------------------------------------------------------------------ */
static void LTC_buildFrame(SMPTE_Time *t, uint8_t frameBits[LTC_BITS_PER_FRAME]);
static void LTC_advanceTimecode(volatile SMPTE_Time *t);
static void LTC_timerHwi(UArg arg);

/* ============================================================================
 * LTC_buildFrame
 *
 * Packs the current timecode into 80 BCD/flag/sync bits, LSB-first per
 * SMPTE convention, ready for biphase transmission.
 * ============================================================================
 */
static void LTC_buildFrame(SMPTE_Time *t, uint8_t frameBits[LTC_BITS_PER_FRAME])
{
    memset(frameBits, 0, LTC_BITS_PER_FRAME);

    uint8_t frameUnits = t->frames % 10;
    uint8_t frameTens  = t->frames / 10;
    uint8_t secUnits   = t->seconds % 10;
    uint8_t secTens    = t->seconds / 10;
    uint8_t minUnits   = t->minutes % 10;
    uint8_t minTens    = t->minutes / 10;
    uint8_t hourUnits  = t->hours % 10;
    uint8_t hourTens   = t->hours / 10;

    /* helper macro: write 'nbits' LSB-first from value 'v' starting at 'pos' */
    #define WRITE_BITS(pos, v, nbits) \
        do { \
            int _i; \
            for (_i = 0; _i < (nbits); _i++) { \
                frameBits[(pos) + _i] = ((v) >> _i) & 0x1; \
            } \
        } while (0)

    WRITE_BITS(0,  frameUnits, 4);   /* bits 0-3   */
    WRITE_BITS(8,  frameTens,  2);   /* bits 8-9   */
    frameBits[10] = 0;               /* drop-frame flag (0 = non-drop)     */
    frameBits[11] = 0;               /* color frame flag                   */

    WRITE_BITS(16, secUnits, 4);     /* bits 16-19 */
    WRITE_BITS(24, secTens,  3);     /* bits 24-26 */
    frameBits[27] = 0;               /* flag bit                           */

    WRITE_BITS(32, minUnits, 4);     /* bits 32-35 */
    WRITE_BITS(40, minTens,  3);     /* bits 40-42 */
    frameBits[43] = 0;               /* flag bit                           */

    WRITE_BITS(48, hourUnits, 4);    /* bits 48-51 */
    WRITE_BITS(56, hourTens,  2);    /* bits 56-57 */
    frameBits[58] = 0;               /* reserved                           */
    frameBits[59] = 0;               /* reserved                           */

    WRITE_BITS(64, LTC_SYNC_WORD, 16); /* bits 64-79 sync word              */

    #undef WRITE_BITS
}

/* ============================================================================
 * LTC_advanceTimecode
 *
 * Increments frames/seconds/minutes/hours with rollover, called once per
 * completed 80-bit frame.
 * ============================================================================
 */
static void LTC_advanceTimecode(volatile SMPTE_Time *t)
{
    t->frames++;
    if (t->frames >= LTC_FPS) {
        t->frames = 0;
        t->seconds++;
        if (t->seconds >= 60) {
            t->seconds = 0;
            t->minutes++;
            if (t->minutes >= 60) {
                t->minutes = 0;
                t->hours++;
                if (t->hours >= 24) {
                    t->hours = 0;
                }
            }
        }
    }
}

/* ============================================================================
 * LTC_timerHwi  -- Hwi callback, fires at LTC_HALFBIT_RATE_HZ
 *
 * Implements biphase-mark-code transmission:
 *   - gHalfBitPos == 0 (start of bit): ALWAYS toggle the output line.
 *   - gHalfBitPos == 1 (mid of bit):   toggle ONLY if current bit == 1.
 *
 * After the mid-bit half-tick of the last bit in a frame, build the next
 * frame's bit table and advance the timecode.
 * ============================================================================
 */
static void LTC_timerHwi(UArg arg)
{
    uint8_t currentBit = gFrameBits[gBitIndex];

    if (gHalfBitPos == 0) {
        /* Start of bit: transition is mandatory */
        gLineLevel ^= 1;
        GPIO_write(LTC_GPIO_OUT, gLineLevel);
        gHalfBitPos = 1;
    } else {
        /* Middle of bit: transition only for a '1' */
        if (currentBit) {
            gLineLevel ^= 1;
            GPIO_write(LTC_GPIO_OUT, gLineLevel);
        }
        gHalfBitPos = 0;

        /* This bit is now fully transmitted; advance to next bit */
        gBitIndex++;
        if (gBitIndex >= LTC_BITS_PER_FRAME) {
            gBitIndex = 0;

            /* Frame complete: compute the next frame's timecode + bits.
             * Kept short (no heap, no blocking calls) since this runs
             * in interrupt context. */
            LTC_advanceTimecode(&gTimecode);

            SMPTE_Time snapshot;
            snapshot.hours   = gTimecode.hours;
            snapshot.minutes = gTimecode.minutes;
            snapshot.seconds = gTimecode.seconds;
            snapshot.frames  = gTimecode.frames;

            LTC_buildFrame(&snapshot, gFrameBits);
        }
    }
}

/* ============================================================================
 * LTC_init
 *
 * Sets up the GPIO output and creates+starts the periodic Timer that drives
 * the biphase-mark bitstream at LTC_HALFBIT_RATE_HZ.
 * ============================================================================
 */
Bool LTC_init(void)
{
    Timer_Params timerParams;
    Error_Block  eb;

    Error_init(&eb);

    /* Board_initGPIO() must already have been called in main() before this */
    GPIO_write(LTC_GPIO_OUT, 0);

    /* Build the very first frame (00:00:00:00) before the timer starts */
    LTC_buildFrame((SMPTE_Time *)&gTimecode, gFrameBits);
    gBitIndex   = 0;
    gHalfBitPos = 0;
    gLineLevel  = 0;

    Timer_Params_init(&timerParams);
    timerParams.periodType = Timer_PeriodType_MICROSECS;
    timerParams.period     = 1000000UL / LTC_HALFBIT_RATE_HZ; /* us per half-bit */
    timerParams.startMode  = Timer_StartMode_AUTO;
    timerParams.runMode    = Timer_RunMode_CONTINUOUS;

    /* Timer_ANY lets SYS/BIOS pick a free hardware timer instance; pin a
     * specific instance number instead of Timer_ANY if you need a
     * particular peripheral (e.g. for a shared clock domain). */
    gTimerHandle = Timer_create(Timer_ANY, LTC_timerHwi, &timerParams, &eb);

    if (gTimerHandle == NULL) {
        System_printf("LTC_init: Timer_create failed\n");
        return FALSE;
    }

    System_printf("LTC_init: LTC generator running at %d fps, "
                  "half-bit rate %d Hz (period %d us)\n",
                  LTC_FPS, LTC_HALFBIT_RATE_HZ, timerParams.period);

    return TRUE;
}

/* ============================================================================
 * LTC_setTime -- optional helper to jam-set the timecode (e.g. from a UI
 * task or GPS/word-clock reference). Safe to call from task context; the
 * write is done with interrupts briefly disabled to keep it atomic against
 * the Hwi.
 * ============================================================================
 */
void LTC_setTime(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t frames)
{
    UInt key = Hwi_disable();

    gTimecode.hours   = hours;
    gTimecode.minutes = minutes;
    gTimecode.seconds = seconds;
    gTimecode.frames  = frames;

    Hwi_restore(key);
}

/* End-Of-File */
