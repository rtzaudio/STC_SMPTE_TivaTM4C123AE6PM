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
 *
 * DROP-FRAME (29.97 fps) SUPPORT
 * -------------------------------
 * 29.97 fps drop-frame timecode is transmitted using the same bit structure
 * as 30 fps (LTC_FPS must be set to 30 when LTC_DROP_FRAME is enabled) but:
 *   1. The drop-frame flag bit (bit 10) is set to 1.
 *   2. Frame numbers 0 and 1 are skipped at the start of every minute,
 *      EXCEPT minutes that are exact multiples of 10 (00, 10, 20, 30, 40, 50).
 * This drops 2 frames x 9 minutes-per-10 = 18 counted frames every 10
 * minutes, which compensates for the ~0.1% difference between a 30 fps
 * count and real 29.97 fps video -- no video frames are ever actually
 * dropped, only frame *numbers* in the timecode count.
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

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
/* Tivaware Driver files */
#include <driverlib/eeprom.h>
#include <driverlib/fpu.h>
#include <driverlib/debug.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/timer.h>
#include <inc/hw_ints.h>
#include <inc/hw_timer.h>

/* Generic Includes */
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "Board.h"          /* Board_LTC_OUT must be defined in Board.h  */

/* ------------------------------------------------------------------------ */
/* Configuration                                                            */
/* ------------------------------------------------------------------------ */
#define LTC_FPS              30          /* frame rate: 25, 24, or 30      */
#define LTC_DROP_FRAME       1           /* 1 = 29.97 drop-frame, 0 = normal.
                                           * NOTE: drop-frame requires
                                           * LTC_FPS == 30 (the timecode is
                                           * still counted in 30ths; only
                                           * the frame *numbering* drops).  */
#define LTC_BITS_PER_FRAME   80
#define LTC_BIT_RATE         (LTC_BITS_PER_FRAME * LTC_FPS)     /* bits/s  */
#define LTC_HALFBIT_RATE_HZ  (LTC_BIT_RATE * 2)                 /* Hz      */

/* GPIO pin driving the LTC line */
#define LTC_GPIO_OUT         Board_SMPTE_OUT

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
static volatile Bool gRunning = FALSE;   /* TRUE while the timer/ISR is active */

/* ------------------------------------------------------------------------ */
/* Forward declarations                                                     */
/* ------------------------------------------------------------------------ */
static void LTC_buildFrame(SMPTE_Time *t, uint8_t frameBits[LTC_BITS_PER_FRAME]);
static void LTC_advanceTimecode(volatile SMPTE_Time *t);
static void LTC_timerHwi(UArg arg);

Bool LTC_init(void);
void LTC_start(void);
void LTC_stop(void);
Bool LTC_isRunning(void);

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
    uint8_t secUnits    = t->seconds % 10;
    uint8_t secTens     = t->seconds / 10;
    uint8_t minUnits    = t->minutes % 10;
    uint8_t minTens     = t->minutes / 10;
    uint8_t hourUnits   = t->hours % 10;
    uint8_t hourTens    = t->hours / 10;

    int _i;

    /* helper macro: write 'nbits' LSB-first from value 'v' starting at 'pos' */
    #define WRITE_BITS(pos, v, nbits) \
        do { \
            for (_i = 0; _i < (nbits); _i++) { \
                frameBits[(pos) + _i] = ((v) >> _i) & 0x1; \
            } \
        } while (0)

    WRITE_BITS(0,  frameUnits, 4);   /* bits 0-3   */
    WRITE_BITS(8,  frameTens,  2);   /* bits 8-9   */
    frameBits[10] = LTC_DROP_FRAME ? 1 : 0;  /* drop-frame flag             */
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
 * completed 80-bit frame. When LTC_DROP_FRAME is enabled, applies the
 * SMPTE drop-frame rule at each minute boundary: frame numbers 0 and 1 are
 * skipped unless the new minute is a multiple of 10.
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

#if LTC_DROP_FRAME
            /* New minute has just begun (seconds==0, frames==0). Skip
             * frame numbers 0 and 1 unless this minute is a multiple
             * of 10 -- e.g. at 00:01:00;00 the count jumps straight to
             * 00:01:00;02, but 00:10:00;00 is NOT skipped. */
            if ((t->minutes % 10) != 0) {
                t->frames = 2;
            }
#endif
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

            /* Toggle the LED on each packet received */
            GPIO_toggle(Board_STAT_LED);
        }
    }
}

/* ============================================================================
 * LTC_init
 *
 * Sets up the GPIO output and creates (but does not start) the periodic
 * Timer that will drive the biphase-mark bitstream at LTC_HALFBIT_RATE_HZ.
 * Call LTC_start() afterward to actually begin transmitting.
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

    /* USER start mode: the timer is created in the stopped state so the
     * generator does not begin transmitting until LTC_start() is called.
     */
    Timer_Params_init(&timerParams);

    timerParams.periodType = Timer_PeriodType_MICROSECS;
    timerParams.period     = 1000000UL / LTC_HALFBIT_RATE_HZ; /* us per half-bit */
    timerParams.startMode  = Timer_StartMode_USER;
    timerParams.runMode    = Timer_RunMode_CONTINUOUS;

    /* Timer_ANY lets SYS/BIOS pick a free hardware timer instance; pin a
     * specific instance number instead of Timer_ANY if you need a
     * particular peripheral (e.g. for a shared clock domain).
     */
    gTimerHandle = Timer_create(Timer_ANY, LTC_timerHwi, &timerParams, &eb);

    if (gTimerHandle == NULL) {
        System_printf("LTC_init: Timer_create failed\n");
        return false;
    }

    gRunning = FALSE;

    //System_printf("LTC_init: LTC generator ready (%d fps%s), "
    //              "half-bit rate %d Hz (period %d us).\n",
    //              LTC_FPS, LTC_DROP_FRAME ? " drop-frame" : "",
    //              LTC_HALFBIT_RATE_HZ, timerParams.period);

    return TRUE;
}

/* ============================================================================
 * LTC_start
 *
 * Begins (or resumes) LTC transmission. Resets the bit/half-bit state so
 * playback always restarts cleanly at the beginning of a bit, rather than
 * possibly resuming mid-bit from wherever a previous LTC_stop() left off.
 * Safe to call from task context; a no-op if already running.
 * ============================================================================
 */
void LTC_start(void)
{
    if (!gRunning)
    {
        /* Initialize variables for start state */
        UInt key = Hwi_disable();
        gBitIndex   = 0;
        gHalfBitPos = 0;
        gLineLevel  = 0;
        /* Set output to low state initially */
        GPIO_write(LTC_GPIO_OUT, 0);
        Hwi_restore(key);

        /* Turn the LED on to indicate active */
        GPIO_write(Board_STAT_LED, Board_LED_ON);

        /* Enable the signal out relay to connect the SMPTE
         * output signal to channel 24 on the tape machine.
         */
        GPIO_write(Board_RELAY, Board_RELAY_ON);
        Task_sleep(50);

        /* Start the timer to begin shifting data out */
        Timer_start(gTimerHandle);
        gRunning = TRUE;
    }
}

/* ============================================================================
 * LTC_stop
 *
 * Halts the timer/ISR and drives the output line to a known idle (low)
 * state. The current timecode value in gTimecode is preserved, so a
 * subsequent LTC_start() resumes counting from where it left off (use
 * LTC_setTime() first if you instead want to jam to a new value).
 * Safe to call from task context; a no-op if already stopped.
 * ============================================================================
 */
void LTC_stop(void)
{
    if (gRunning)
    {
        Timer_stop(gTimerHandle);
        gRunning = FALSE;
        /* SMPTE output pin low */
        GPIO_write(LTC_GPIO_OUT, PIN_LOW);
        /* Relay and status LED off */
        GPIO_write(Board_RELAY, Board_RELAY_OFF);
        GPIO_write(Board_STAT_LED, Board_LED_ON);
    }
}

/* ============================================================================
 * LTC_isRunning -- returns TRUE if the generator is currently transmitting.
 * ============================================================================
 */
Bool LTC_isRunning(void)
{
    return gRunning;
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
