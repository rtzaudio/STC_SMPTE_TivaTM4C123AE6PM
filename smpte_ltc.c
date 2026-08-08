/*
 * smpte_ltc.c
 *
 * Core LTC decode engine. Hardware-agnostic: SMPTE_LTC_onEdge() only needs
 * a monotonically increasing tick count from whatever free-running timer
 * you wire up (see smpte_hw.c for the TM4C-specific glue).
 *
 * -- Reverse-direction decoding --
 *
 * LTC's sync word (0x3FFD, transmitted as bits 64-79 of each 80-bit frame)
 * was deliberately chosen to be asymmetric: it does not equal its own
 * bit-reversal, and -- by construction of the surrounding biphase-mark and
 * BCD-data constraints -- neither pattern is expected to occur anywhere
 * else in a valid bitstream. That's what makes direction-sensing possible
 * at all: a receiver that just shifts bits into a continuously-sliding
 * window without knowing the tape direction will see the *forward* sync
 * pattern when played forward, and the *bit-reversed* sync pattern when
 * played backward.
 *
 * Why the reversal, concretely: on forward playback, the window's tail
 * (its 16 newest bits) holds SMPTE bit index 64 at the oldest/MSB
 * position and index 79 at the newest/LSB position, which is exactly how
 * SMPTE_SYNC_WORD is defined. On reverse playback the *same 16 physical
 * transitions* are traversed in the opposite time order, so the window's
 * tail instead holds index 79 at the oldest/MSB position and index 64 at
 * the newest/LSB position -- the bit-reversal of the forward pattern,
 * SMPTE_SYNC_WORD_REVERSE.
 *
 * A further complication: on reverse playback the sync word is
 * (physically) encountered *before* the 64 data bits of the same frame,
 * since sync is transmitted last in forward time and rewinding traverses
 * time backward. So when a reverse sync is detected, this frame's data
 * bits have not arrived yet -- they arrive next, across the following 64
 * edges, in descending SMPTE-index order (63, 62, ..., 0). Those bits are
 * captured into dec->revData (indexed by their true SMPTE bit index, so
 * revData is always in normal field order once complete) and the frame is
 * only parsed once the *next* reverse sync confirms exactly 64 bits were
 * collected in between.
 *
 * This logic was derived analytically rather than taken from a verified
 * reference decoder -- validate it against a real reverse-play LTC
 * capture before relying on it.
 */

/* Generic Includes */
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
#include <ti/sysbios/knl/Queue.h>
//#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Tivaware Driver files */
#include <driverlib/eeprom.h>
#include <driverlib/fpu.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/adc.h>
#include <driverlib/can.h>
#include <driverlib/debug.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/ssi.h>
#include <driverlib/i2c.h>
#include <driverlib/qei.h>
#include <driverlib/interrupt.h>
#include <driverlib/pwm.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/timer.h>
#include <driverlib/uart.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_types.h>
#include <inc/hw_ssi.h>
#include <inc/hw_i2c.h>
#include <inc/hw_timer.h>

/* Generic Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "smpte_ltc.h"

/* Ignore intervals shorter than this -- electrical glitch / contact bounce
 * on the comparator output, not a real LTC transition. Tune to your timer
 * tick rate; at an 80 MHz tick rate this is ~50ns, comfortably below any
 * real half-bit period (LTC half-bit periods are on the order of
 * 200-2000us across the 24-30 fps range).
 */

#define SMPTE_MIN_INTERVAL_TICKS   4u

/*
 * Static Function Prototypes
 */

static void SMPTE_LTC_parseFrame(SMPTE_Decoder *dec);
static void SMPTE_LTC_parseFrameReverse(SMPTE_Decoder *dec);
static void resetLock(SMPTE_Decoder *dec);

/* Initialize the decoder */

void SMPTE_LTC_init(SMPTE_Decoder *dec)
{
    memset(dec, 0, sizeof(*dec));

    /* full 32-bit range; narrow this after init.
     * if your tick source is smaller (see smpte_ltc.h)
     */
    dec->tickMask = 0xFFFFFFFFu;
}

/* Forward-window bit access: frame[] is a continuously-shifting 80-bit
 * window, oldest bit at frame[0] bit7, newest at frame[9] bit0
 */

static inline uint8_t frameGetBit(const uint8_t *frame, uint8_t smpteBitIndex)
{
    uint8_t posFromLsb = (uint8_t)(79u - smpteBitIndex);
    uint8_t byteIdx    = (uint8_t)(9u - (posFromLsb >> 3));
    uint8_t bitInByte  = (uint8_t)(posFromLsb & 0x7u);

    return (uint8_t)((frame[byteIdx] >> bitInByte) & 0x1u);
}

/* -- reverse-buffer bit access: revData[] is a plain 64-bit buffer already
 * indexed by true SMPTE bit index (0 = MSB of revData[0]), since bits are
 * written to their real index as they arrive --
 */

static inline void bufSetBit(uint8_t *buf, uint8_t bitIndex, uint8_t val)
{
    uint8_t byteIdx   = (uint8_t)(bitIndex >> 3);
    uint8_t bitInByte = (uint8_t)(7u - (bitIndex & 0x7u));
    if (val) {
        buf[byteIdx] = (uint8_t)(buf[byteIdx] | (1u << bitInByte));
    } else {
        buf[byteIdx] = (uint8_t)(buf[byteIdx] & ~(1u << bitInByte));
    }
}

static inline uint8_t bufGetBit(const uint8_t *buf, uint8_t bitIndex)
{
    uint8_t byteIdx   = (uint8_t)(bitIndex >> 3);
    uint8_t bitInByte = (uint8_t)(7u - (bitIndex & 0x7u));
    return (uint8_t)((buf[byteIdx] >> bitInByte) & 0x1u);
}

/* SMPTE fields are transmitted LSB-first, so bit `startBit` is the LSB of
 * the field. Same convention for both accessors above.
 */

typedef uint8_t (*BitGetter)(const uint8_t *, uint8_t);

static uint32_t getField(BitGetter get, const uint8_t *buf, uint8_t startBit, uint8_t numBits)
{
    uint32_t val = 0;
    uint8_t k;
    for (k = 0; k < numBits; k++) {
        val |= ((uint32_t)get(buf, (uint8_t)(startBit + k))) << k;
    }
    return val;
}

/* SMPTE 12M bit offsets (identical for both directions -- direction only
 * changes how the 64 data bits got collected, not what they mean). The
 * non-drop / drop-frame 30 fps and 25 fps EBU variants share this layout;
 * only the interpretation of bit 10 (drop frame flag, meaningless at 25
 * fps) and the BGF flag bits differs between variants -- that
 * interpretation is application knowledge, not something decodable
 * purely from the bitstream.
 */

enum {
    BIT_FRAME_UNITS = 0,   /* 4 bits */
    BIT_USER1       = 4,   /* 4 bits */
    BIT_FRAME_TENS  = 8,   /* 2 bits */
    BIT_DROP_FRAME  = 10,  /* 1 bit  */
    BIT_COLOR_FRAME = 11,  /* 1 bit  */
    BIT_USER2       = 12,  /* 4 bits */
    BIT_SEC_UNITS   = 16,  /* 4 bits */
    BIT_USER3       = 20,  /* 4 bits */
    BIT_SEC_TENS    = 24,  /* 3 bits */
    /* bit 27 = BGF/polarity-correction flag, not decoded here */
    BIT_USER4       = 28,  /* 4 bits */
    BIT_MIN_UNITS   = 32,  /* 4 bits */
    BIT_USER5       = 36,  /* 4 bits */
    BIT_MIN_TENS    = 40,  /* 3 bits */
    /* bit 43 = BGF flag, not decoded here */
    BIT_USER6       = 44,  /* 4 bits */
    BIT_HOUR_UNITS  = 48,  /* 4 bits */
    BIT_USER7       = 52,  /* 4 bits */
    BIT_HOUR_TENS   = 56,  /* 2 bits */
    /* bits 58-59 = BGF flags, not decoded here */
    BIT_USER8       = 60   /* 4 bits */
    /* bits 64-79 = sync word, checked directly in SMPTE_LTC_onEdge() */
};

static void parseCommon(SMPTE_Timecode *tc, BitGetter get, const uint8_t *buf)
{
    tc->frames  = (uint8_t)(getField(get, buf, BIT_FRAME_TENS, 2) * 10u +
                             getField(get, buf, BIT_FRAME_UNITS, 4));
    tc->seconds = (uint8_t)(getField(get, buf, BIT_SEC_TENS, 3) * 10u +
                             getField(get, buf, BIT_SEC_UNITS, 4));
    tc->minutes = (uint8_t)(getField(get, buf, BIT_MIN_TENS, 3) * 10u +
                             getField(get, buf, BIT_MIN_UNITS, 4));
    tc->hours   = (uint8_t)(getField(get, buf, BIT_HOUR_TENS, 2) * 10u +
                             getField(get, buf, BIT_HOUR_UNITS, 4));

    tc->dropFrame  = getField(get, buf, BIT_DROP_FRAME, 1)  != 0;
    tc->colorFrame = getField(get, buf, BIT_COLOR_FRAME, 1) != 0;

    tc->userBits[0] = (uint8_t)getField(get, buf, BIT_USER1, 4);
    tc->userBits[1] = (uint8_t)getField(get, buf, BIT_USER2, 4);
    tc->userBits[2] = (uint8_t)getField(get, buf, BIT_USER3, 4);
    tc->userBits[3] = (uint8_t)getField(get, buf, BIT_USER4, 4);
    tc->userBits[4] = (uint8_t)getField(get, buf, BIT_USER5, 4);
    tc->userBits[5] = (uint8_t)getField(get, buf, BIT_USER6, 4);
    tc->userBits[6] = (uint8_t)getField(get, buf, BIT_USER7, 4);
    tc->userBits[7] = (uint8_t)getField(get, buf, BIT_USER8, 4);
}

static void SMPTE_LTC_parseFrame(SMPTE_Decoder *dec)
{
    parseCommon(&dec->tc, frameGetBit, dec->frame);
    dec->tc.direction = SMPTE_DIR_FORWARD;
}

static void SMPTE_LTC_parseFrameReverse(SMPTE_Decoder *dec)
{
    parseCommon(&dec->tc, bufGetBit, dec->revData);
    dec->tc.direction = SMPTE_DIR_REVERSE;
}

/* Shared reset for both a hard dropout detected inline in onEdge() and an
 * externally-detected total-silence timeout (onTimeout()). Clears clock
 * recovery and every in-progress bit-accumulation buffer so a fresh
 * signal locks cleanly rather than risking a spurious match against
 * stale leftover bits. Deliberately does NOT touch dec->tc / frameReady /
 * frameCount -- the last decoded timecode stays available to the
 * consumer alongside the error flags.
 */

static void resetLock(SMPTE_Decoder *dec)
{
    dec->haveLastEdge    = false;
    dec->avgHalfBitTicks = 0;
    dec->halfBitPending  = false;
    dec->signalPresent   = false;
    dec->revBitCount     = 0;
    dec->bitsSinceSync   = 0;
    dec->direction       = SMPTE_DIR_UNKNOWN;
    memset(dec->frame, 0, sizeof(dec->frame));
    memset(dec->revData, 0, sizeof(dec->revData));
}

void SMPTE_LTC_onTimeout(SMPTE_Decoder *dec)
{
    resetLock(dec);
    dec->timedOut = true;
    dec->timeoutCount++;
}

bool SMPTE_LTC_onEdge(SMPTE_Decoder *dec, uint32_t nowTicks)
{
    if (!dec->haveLastEdge)
    {
        dec->lastEdgeTick = nowTicks;
        dec->haveLastEdge = true;
        return false;
    }

    /* unsigned sub, masked to the tick counter's actual width, so a
     * hardware counter narrower than 32 bits (e.g. a 24-bit CCP capture
     * register) still wraps correctly instead of producing one huge
     * bogus interval every rollover
     */

    uint32_t delta = (nowTicks - dec->lastEdgeTick) & dec->tickMask;

    dec->lastEdgeTick = nowTicks;

    if (delta < SMPTE_MIN_INTERVAL_TICKS)
    {
        return false;   /* glitch, don't disturb clock recovery */
    }

    if (dec->avgHalfBitTicks == 0)
    {
        /* bootstrap: seed the running average with the first plausible
         * interval; it will settle within a handful of edges. */

        dec->avgHalfBitTicks = delta;

        return false;
    }

    uint32_t shortMax = dec->avgHalfBitTicks + (dec->avgHalfBitTicks >> 1); /* ~1.5x */
    uint32_t longMax  = dec->avgHalfBitTicks * 3u;                          /* generous outer bound */

    bool isShort;

    if (delta <= shortMax)
    {
        isShort = true;
    }
    else if (delta <= longMax)
    {
        isShort = false;
    }
    else
    {
        /* way outside expectation: dropout, cable unplugged, huge speed
         * change (fast-wind). Drop clock-recovery lock and start over. */

        resetLock(dec);

        dec->badSyncCount++;

        return false;
    }

    dec->signalPresent = true;
    dec->timedOut      = false;   /* real edges are arriving again */

    bool    bitReady = false;
    uint8_t bitVal   = 0;

    if (!isShort)
    {
        /* one long interval = a full bit cell with no mid-cell transition -> "0" */

        dec->halfBitPending = false;  /* a stray pending half is discarded */
        bitVal   = 0;
        bitReady = true;

        /* long intervals are a solid timing reference; nudge the average
         * gently using half the long interval
         */
        dec->avgHalfBitTicks = (uint32_t)((int32_t)dec->avgHalfBitTicks +
                                (((int32_t)(delta >> 1) - (int32_t)dec->avgHalfBitTicks) / 8));
    }
    else
    {
        /* short interval = half a bit cell */

        if (!dec->halfBitPending)
        {
            dec->halfBitPending = true;    /* first half seen, wait for the second */
        }
        else
        {
            dec->halfBitPending = false;
            bitVal   = 1;
            bitReady = true;
        }

        dec->avgHalfBitTicks = (uint32_t)((int32_t)dec->avgHalfBitTicks +
                                (((int32_t)delta - (int32_t)dec->avgHalfBitTicks) / 8));
    }

    if (!bitReady) {
        return false;
    }

    dec->bitsSinceSync++;

    /* feed the forward continuously-shifting window (used for forward
     * sync detection and forward field extraction)
     */

    int i;

    for (i = 0; i < 9; i++)
    {
        dec->frame[i] = (uint8_t)((dec->frame[i] << 1) | (dec->frame[i + 1] >> 7));
    }

    dec->frame[9] = (uint8_t)((dec->frame[9] << 1) | bitVal);

    /* also feed the reverse-direction data collector, in case a
     * reverse frame is currently mid-collection. Harmless no-op once 64
     * bits are already collected and awaiting confirmation.
     */

    if (dec->revBitCount < 64u)
    {
        uint8_t idx = (uint8_t)(63u - dec->revBitCount);
        bufSetBit(dec->revData, idx, bitVal);
        dec->revBitCount++;
    }

    uint16_t tail = (uint16_t)(((uint16_t)dec->frame[8] << 8) | dec->frame[9]);

    bool gotFrame = false;

    if (tail == SMPTE_SYNC_WORD)
    {
        /* a genuine frame is always exactly 80 bits; flag (but still
         * resync to) anything else as a framing anomaly
         */

        if (dec->direction == SMPTE_DIR_FORWARD && dec->bitsSinceSync != 80u)
        {
            dec->badSyncCount++;
        }

        SMPTE_LTC_parseFrame(dec);
        dec->direction     = SMPTE_DIR_FORWARD;
        dec->bitsSinceSync = 0;
        dec->revBitCount   = 0;   /* discard any stale reverse-collection in progress */
        dec->tc.frameCount++;
        dec->frameReady = true;
        gotFrame = true;
    }
    else if (tail == SMPTE_SYNC_WORD_REVERSE)
    {
        if (dec->direction == SMPTE_DIR_REVERSE && dec->revBitCount == 64u)
        {
            SMPTE_LTC_parseFrameReverse(dec);
            dec->tc.frameCount++;
            dec->frameReady = true;
            gotFrame = true;
        }
        else if (dec->direction == SMPTE_DIR_REVERSE)
        {
            dec->badSyncCount++;   /* wrong bit count collected between reverse syncs */
        }

        dec->direction     = SMPTE_DIR_REVERSE;
        dec->revBitCount   = 0;    /* this sync starts the *next* frame's collection */
        dec->bitsSinceSync = 0;
    }

    return gotFrame;
}
