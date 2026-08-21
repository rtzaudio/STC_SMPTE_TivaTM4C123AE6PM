/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2016, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================ */

#ifndef STC_SMPTE_DECODER_H
#define STC_SMPTE_DECODER_H

#include <stdint.h>
#include <stdbool.h>

#include "libltc/ltc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SMPTE Constants and Data Structures for encoding and decoding packets.
 */

#define SMPTE_SYNC_WORD           0x3FFDu   /* forward: bits 64-79, MSB-first (bit64..bit79) */
#define SMPTE_SYNC_WORD_REVERSE   0xBFFCu   /* 16 bits, bit-reversed - what a continuously   */
                                            /* shifting receiver sees when the signal plays  */
                                            /* backward. See smpte_ltc.c for the derivation. */
typedef enum {
    SMPTE_DIR_UNKNOWN = 0,
    SMPTE_DIR_FORWARD,
    SMPTE_DIR_REVERSE
} SMPTE_Direction;

typedef struct {
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
    uint8_t  frames;
    bool     dropFrame;
    bool     colorFrame;
    uint8_t  userBits[8];      /* 8 x 4-bit user-bit nibbles, raw */
    uint32_t frameCount;       /* running count of decoded frames, for rate/health checks */
    SMPTE_Direction direction; /* direction this specific frame was decoded in */
} SMPTE_Decoder_Timecode;

/* Opaque-ish decoder state -- allocate one instance per LTC input channel. */

typedef struct {
    /* timing / clock recovery */
    uint32_t lastEdgeTick;
    uint32_t avgHalfBitTicks;   /* adaptive running estimate of a half-bit period, in timer ticks */
    bool     haveLastEdge;
    bool     halfBitPending;

    /* Bit width of the hardware tick counter, as an AND mask (e.g.
     * 0xFFFFFFFF for a full 32-bit free-running timer, 0x00FFFFFF for a
     * 16-bit GPTM capture register extended with an 8-bit prescaler).
     * SMPTE_LTC_init() defaults this to 0xFFFFFFFF; override it right
     * after init if your tick source is narrower, so that edge-to-edge
     * deltas wrap correctly instead of producing one huge bogus interval
     * every time the hardware counter rolls over. */
    uint32_t tickMask;

    /* forward-direction bit accumulation: continuously shifting 80-bit
     * window, MSB-first, oldest bit at frame[0] bit7. Sync is detected
     * by checking the tail after every accepted bit, so this is
     * self-synchronizing frame to frame. */
    uint8_t  frame[10];

    /* reverse-direction bit accumulation: a reverse sync marks the start
     * of a frame's data (which then arrives in descending SMPTE-index
     * order); bits are written directly to their true index as they
     * arrive, so revData is always in normal MSB-first field order once
     * revBitCount reaches 64. */
    uint8_t  revData[8];
    uint8_t  revBitCount;           /* bits collected into revData since the last reverse sync, 0-64 */

    SMPTE_Direction direction;      /* last confirmed playback direction */
    uint32_t bitsSinceSync;         /* bits accepted since the last confirmed forward sync, for a
                                     * frame-length sanity check (a genuine frame is always 80 bits) */
    /* decoded output */
    SMPTE_Decoder_Timecode tc;
    volatile bool  frameReady;      /* set in ISR context when a new frame has been parsed */

    /* health / diagnostics */
    volatile bool  signalPresent;   /* edges are arriving and classifying cleanly */
    volatile bool  timedOut;        /* no edges at all for longer than expected -- see onTimeout() */
    uint32_t       badSyncCount;    /* framing anomalies: wrong bit-spacing, dropout, bit slips */
    uint32_t       timeoutCount;    /* number of times onTimeout() has fired */
} SMPTE_Decoder;

/* Call once at startup, and again any time you want to force a resync. */
void SMPTE_LTC_init(SMPTE_Decoder *dec);

/*
 * Call from the edge-capture Hwi with the current tick count of whatever
 * timestamp source you're using. Does all of the time-critical work:
 * interval classification, biphase-mark bit decode, direction-aware
 * sync-word detection and frame parse. Written to be short and
 * branch-light so it is safe to run at interrupt level.
 *
 * Returns true if a full timecode frame was just decoded (dec->tc is valid).
 */
bool SMPTE_LTC_onEdge(SMPTE_Decoder *dec, uint32_t nowTicks);

/*
 * Call periodically (e.g. every few ms) from a Task or Clock context --
 * NOT from the edge Hwi -- whenever your own watchdog mechanism has
 * determined that no edges have arrived for longer than expected (LTC
 * dropout, cable unplugged, tape stopped). This is deliberately not
 * driven by a tick comparison inside this file, since the right "now"
 * reference depends on your timestamp source (in particular, a GPTM
 * configured for edge-time capture does not expose a live free-running
 * read the way a plain free-run timer does -- see smpte_hw.c). Resets
 * clock recovery and bit-accumulation state so a fresh signal can lock
 * cleanly instead of reusing a stale average; leaves the last decoded
 * dec->tc value in place so a consumer can still see "last known good"
 * timecode alongside the timedOut flag.
 */
void SMPTE_LTC_onTimeout(SMPTE_Decoder *dec);

/* Convert a tick delta to microseconds given the timer's tick frequency. */
static inline uint32_t SMPTE_LTC_ticksToUs(uint32_t ticks, uint32_t timerHz)
{
    return (uint32_t)(((uint64_t)ticks * 1000000u) / timerHz);
}

/*** Function Prototypes ***************************************************/

void SMPTE_initDecoder(void);
int SMPTE_Decoder_Start();
int SMPTE_Decoder_Stop(void);
void SMPTE_Decoder_Reset(void);

SMPTE_Decoder *SMPTE_HW_getDecoder(void);
uint32_t SMPTE_HW_getTimerHz(void);
bool SMPTE_HW_isRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* SMPTE_LTC_H */

/* End-Of-File */
