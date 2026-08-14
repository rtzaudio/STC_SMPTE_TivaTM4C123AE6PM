/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2021-2024, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================
 *
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================ */

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>
#include <xdc/runtime/Memory.h>
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
#include <driverlib/interrupt.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/timer.h>
/* Tivaware Driver Peripherals */
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_types.h>
#include <inc/hw_ssi.h>
#include <inc/hw_i2c.h>
#include <inc/hw_timer.h>
/* Generic Includes */
#include <file.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
/* XDCtools Header files */
#include "Board.h"
#include "STC_SMPTE.h"

/* Ignore intervals shorter than this -- electrical glitch / contact bounce
 * on the comparator output, not a real LTC transition. Tune to your timer
 * tick rate; at an 80 MHz tick rate this is ~50ns, comfortably below any
 * real half-bit period (LTC half-bit periods are on the order of
 * 200-2000us across the 24-30 fps range).
 */

#define SMPTE_MIN_INTERVAL_TICKS    4u

/* Wide Timer 0 sub-timer A is natively 32-bit in capture mode -- no
 * prescale extension needed, unlike the standard-timer version.
 */

#define SMPTE_CAPTURE_TICK_MASK     0xFFFFFFFFu

/* Watchdog tuning: declare signal loss after this many ms with zero
 * edges. Comfortably above any real LTC half-bit period (~200-2000us)
 * with margin for slow-motion scrubbing; tighten it if your application
 * needs a faster "signal lost" reaction.
 */

#define SMPTE_WATCHDOG_PERIOD_MS    5u
#define SMPTE_WATCHDOG_TIMEOUT_MS   20u

/*** Global Data Items ***/

SMPTETimecode g_timecode;

bool g_bPostInterrupts;
bool g_decoderEnabled;
bool g_bPostInterrupts;

/*** Static Data Items ***/

static volatile uint32_t gEdgeSeq         = 0;   /* bumped once per capture ISR */
static uint32_t          gLastSeenEdgeSeq = 0;
static uint32_t          gStaleMs         = 0;

static bool              gRunning = false;
static uint32_t          gTimerHz;
static SMPTE_Decoder     gLtcDecoder;
static Clock_Struct      gWatchdogClockStruct;

/* Hwi_Struct for timer interrupt handlers */
static Hwi_Struct        wtimer0AHwiStruct;
static Hwi_Struct        wtimer0BHwiStruct;
static volatile uint32_t g_uiPeriod = 0;
static volatile uint32_t g_uiHighCount = 0;
static volatile uint32_t g_uiLowCount = 0;

/*** Static Function Prototypes ***/

static Void WTimer0AHwi(UArg arg);
static Void WTimer0BHwi(UArg arg);

static Void DecodeTaskFxn(UArg arg0, UArg arg1);
static void watchdogClockFxn(UArg arg);

static void SMPTE_LTC_parseFrame(SMPTE_Decoder *dec);
static void SMPTE_LTC_parseFrameReverse(SMPTE_Decoder *dec);
static void resetLock(SMPTE_Decoder *dec);

/*** External Data Items ***/

extern SYSCFG g_cfg;
extern uint32_t g_systemClock;

//*****************************************************************************
//********************** SMPTE DECODER SUPPORT ********************************
//*****************************************************************************

void SMPTE_initDecoder(void)
{
    Error_Block eb;
    Hwi_Params hwiParams;

    gTimerHz = SysCtlClockGet();

    SMPTE_LTC_init(&gLtcDecoder);
    gLtcDecoder.tickMask = SMPTE_CAPTURE_TICK_MASK;

    /* Create INT_WTIMER0 hardware interrupt handler */
    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&(wtimer0AHwiStruct), INT_WTIMER0A, WTimer0AHwi, &hwiParams, &eb);
    if (Error_check(&eb))
        System_abort("Couldn't construct WTIMER0A error hwi");

    /* Create INT_WTIMER0B hardware interrupt handler */
    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&(wtimer0BHwiStruct), INT_WTIMER0B, WTimer0BHwi, &hwiParams, &eb);
    if (Error_check(&eb))
        System_abort("Couldn't construct WTIMER0B error hwi");

    /* --- route the LTC input pin (PC4) to Wide Timer 0's CCP0 capture input --- */

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0));

    /* Disable global interrupts */
    IntMasterDisable();

    /* Configure PC4 to WT0CCP0 for timer input */
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* Configure the GPIO to be CCP pins for the Timer peripheral */
    GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    GPIOPinConfigure(GPIO_PC5_WT0CCP1);

    /* Disable the timer before we change anything */
    TimerDisable(WTIMER0_BASE, TIMER_BOTH);

    /* Capture on both rising and falling edges -- biphase-mark decode
     * needs the interval between every transition, not just one polarity
     * Configure WTIMER0, sub-timer A, as 32-bit edge-time capture,
     * up-counting
     */
    TimerConfigure(WTIMER0_BASE, (TIMER_CFG_SPLIT_PAIR |
                                  TIMER_CFG_A_PERIODIC | TIMER_CFG_A_CAP_TIME_UP |
                                  TIMER_CFG_B_PERIODIC | TIMER_CFG_B_CAP_TIME_UP));

    /* To use the wide timer in edge time mode, it must be preloaded with initial
     * values. If the prescaler is used, then it must be preloaded as well.
     * Since we want to use all 48-bits for both timers it will be loaded with
     * the maximum of 0xFFFFFFFF for the 32-bit wide split timers, and 0xFF to add
     * the additional 8-bits to the split timers with the prescaler.
     */
    TimerLoadSet(WTIMER0_BASE, TIMER_BOTH, 0xFFFFFFFF);
    TimerPrescaleSet(WTIMER0_BASE, TIMER_BOTH, 0x00);

    TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(WTIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

    /* Clear the interrupt status flag.  This is done to make sure the
     * interrupt flag is cleared before we enable it.
     */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT|TIMER_CAPB_EVENT);

    /* Enable the Timer A and B interrupts for Capture Events */
    TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(WTIMER0_BASE, TIMER_CAPB_EVENT);

    /* timer left disabled here -- SMPTE_HW_start() enables it */
    /* Enable the interrupts for Timer A and Timer B on the processor (NVIC) */
    IntEnable(INT_WTIMER0A);
    IntEnable(INT_WTIMER0B);

    /* Renable master interrtupts */
    IntMasterEnable();

    /* --- construct (but do not yet start) the signal-loss watchdog --- */
    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.period    = SMPTE_WATCHDOG_PERIOD_MS;   /* assumes default 1ms Clock tick */
    clockParams.startFlag = false;
    Clock_construct(&gWatchdogClockStruct, watchdogClockFxn, SMPTE_WATCHDOG_PERIOD_MS, &clockParams);

    /* Create the SMPTE packet decoder task */
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    Error_init(&eb);
    taskParams.stackSize = 2048;
    taskParams.priority  = 10;
    Task_create((Task_FuncPtr)DecodeTaskFxn, &taskParams, &eb);
}

//*****************************************************************************
// Initialize and reset all decoder global variables
//*****************************************************************************

Void SMPTE_Decoder_Reset(void)
{

}

//*****************************************************************************
// Initialize and start the SMPTE decoder edge timer interrupts
//*****************************************************************************

int SMPTE_Decoder_Start(void)
{
    if (!gRunning)
    {
        SMPTE_LTC_init(&gLtcDecoder);

        gLtcDecoder.tickMask = SMPTE_CAPTURE_TICK_MASK;

        gEdgeSeq         = 0;
        gLastSeenEdgeSeq = 0;
        gStaleMs         = 0;

        TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);

        TimerEnable(WTIMER0_BASE, TIMER_A);
        TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT);

        /* Enable timer A & B interrupts ???? */
        IntEnable(INT_WTIMER0A);

        Clock_start(Clock_handle(&gWatchdogClockStruct));

        g_bPostInterrupts = true;

        gRunning = true;
    }

    return 0;
}

//*****************************************************************************
// Stop the SMPTE Decoder
//*****************************************************************************

int SMPTE_Decoder_Stop(void)
{
    if (gRunning)
    {
        Clock_stop(Clock_handle(&gWatchdogClockStruct));

        /* Disable both Timer A and Timer B */
        TimerDisable(WTIMER0_BASE, TIMER_BOTH);

        IntDisable(INT_WTIMER0A);
        IntDisable(INT_WTIMER0B);

        /* Disable the Timer A and B interrupts for Capture Events */
        TimerIntDisable(WTIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

        /* Clear any interrupts pending */
        TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

        /* Status LED */
        GPIO_write(Board_STAT_LED, Board_LED_ON);

        /* SMPTE input mute on */
        GPIO_write(Board_SMPTE_MUTE, PIN_LOW);
        GPIO_write(Board_FRAME_SYNC, PIN_LOW);
        GPIO_write(Board_DIRECTION, PIN_LOW);
        GPIO_write(Board_SMPTE_INT_N, PIN_HIGH);
        GPIO_write(Board_BUSY_N, PIN_HIGH);

        g_bPostInterrupts = false;

        gRunning = false;
    }

    return 1;
}

SMPTE_Decoder *SMPTE_HW_getDecoder(void)
{
    return &gLtcDecoder;
}

bool SMPTE_HW_isRunning(void)
{
    return gRunning;
}

uint32_t SMPTE_HW_getTimerHz(void)
{
    return gTimerHz;
}

/*
 * WTIMER0A capture interrupt. Runs in Hwi context. The edge time itself
 * was already latched by hardware the instant the pin transitioned; all
 * this ISR does is clear the flag and read the latched value out, so its
 * own scheduling latency doesn't touch the timestamp's accuracy.
 */

#if 0
static void timerCaptureHwi(UArg arg)
{
    /* Clear the interrupt flag first */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);
    /* Read time at which interrupt occurred */
    uint32_t capturedTick = TimerValueGet(WTIMER0_BASE, TIMER_A);
    /* increment edge counter */
    gEdgeSeq++;
    /* process the capture tick state */
    SMPTE_LTC_onEdge(&gLtcDecoder, capturedTick);
}
#endif

/* Rising Edge Interrupt (Start of Pulse) */
Void WTimer0AHwi(UArg arg)
{
    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);
    /* Echo high pin change to SYNC pin */
    GPIO_write(Board_FRAME_SYNC, PIN_HIGH);
    /* Store the start time */
    g_uiHighCount = TimerValueGet(WTIMER0_BASE, TIMER_A);
    /* Call the edge change interrupt handler */
    SMPTE_LTC_onEdge(&gLtcDecoder, g_uiHighCount);
}

/* Falling Edge Interrupt (End of Pulse) */
Void WTimer0BHwi(UArg arg)
{
    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);
    /* Echo low pin change to SYNC pin */
    GPIO_write(Board_FRAME_SYNC, PIN_LOW);
    /* Store the end time */
    g_uiLowCount = TimerValueGet(WTIMER0_BASE, TIMER_B);

    /* Call the edge change interrupt handler */
    SMPTE_LTC_onEdge(&gLtcDecoder, g_uiLowCount);
}

/*
 * Runs periodically in Swi context (TI-RTOS Clock functions run at Swi
 * level). Purely counts elapsed watchdog periods since gEdgeSeq last
 * changed -- deliberately independent of the capture timer itself, per
 * the note in the file header.
 */

static void watchdogClockFxn(UArg arg)
{
    if (!gRunning) {
        return;
    }

    if (gEdgeSeq == gLastSeenEdgeSeq)
    {
        if (gStaleMs < SMPTE_WATCHDOG_TIMEOUT_MS)
        {
            gStaleMs += SMPTE_WATCHDOG_PERIOD_MS;

            if (gStaleMs >= SMPTE_WATCHDOG_TIMEOUT_MS)
            {
                SMPTE_LTC_onTimeout(&gLtcDecoder);   /* fires once per stall event */
            }
        }
    }
    else
    {
        gLastSeenEdgeSeq = gEdgeSeq;
        gStaleMs         = 0;
    }
}

//*****************************************************************************
// This task decodes 80-bit SMPTE packets fed to it from the edge interrupt
// handlers. Once a valid packet sync word is found in the stream, the 64-bit
// word is passed to this task to extract and decode all the time and other
// information to provide the main SPI host task with the time code
// information needed via an SPI interrupt.
//*****************************************************************************

Void DecodeTaskFxn(UArg arg0, UArg arg1)
{
    bool wasSignalPresent = true;
    bool wasTimedOut = false;
    uint32_t key;
    uint32_t lastBadSyncCount = 0;
    SMPTE_Timecode local;
    SMPTE_Decoder *dec = SMPTE_HW_getDecoder();

    /* Initialize and start edge decode interrupts */
    SMPTE_Decoder_Start();

    for (;;)
    {
        if (dec->frameReady)
        {
            key = Hwi_disable();
            local = dec->tc;
            dec->frameReady = false;
            Hwi_restore(key);

            /* Toggle the LED on each packet received */
            GPIO_toggle(Board_STAT_LED);

            System_printf("%02u:%02u:%02u:%02u%s [%s]\n",
                          local.hours, local.minutes,
                          local.seconds, local.frames,
                          local.dropFrame ? " DF" : "",
                          local.direction == SMPTE_DIR_REVERSE ? "REV" : "FWD");
        }

        if (dec->signalPresent != wasSignalPresent)
        {
            wasSignalPresent = dec->signalPresent;

            if (!wasSignalPresent) {
                System_printf("LTC: edges present but failing to classify\n");
            }
        }

        if (dec->timedOut != wasTimedOut)
        {
            wasTimedOut = dec->timedOut;

            if (wasTimedOut)
            {
                System_printf("LTC: signal lost -- no edges (timeout #%u)\n", dec->timeoutCount);
            }
            else
            {
                System_printf("LTC: signal recovered\n");
            }
        }

        if (dec->badSyncCount != lastBadSyncCount)
        {
            lastBadSyncCount = dec->badSyncCount;
            System_printf("LTC: framing anomaly (total: %u)\n", dec->badSyncCount);
        }

        /* Frames arrive every ~33-42ms depending on fps; 5ms poll gives
         * comfortable margin without burning cycles.
         */
        Task_sleep(5);
    }
}


#if 0
    /*
     * Loop waiting for SMPTE word packets to arrive
     */

    while (true)
    {
        /* Wait for an 80-bit timecode word */
        //if (!Mailbox_pend(mailboxWord, &word, 100))
        {
            //GPIO_write(Board_STAT_LED, Board_LED_ON);
            //continue;
        }

        /* Toggle the LED on each packet received */
        GPIO_toggle(Board_STAT_LED);

        /* Now extract any time and other data from the packet */
        key = GateMutex_enter(gateMutex0);
        //g_timecode.frame = (uint8_t)(word.ltc.frame_units + (word.ltc.frame_tens * 10));
        //g_timecode.secs  = (uint8_t)(word.ltc.secs_units  + (word.ltc.secs_tens  * 10));
        //g_timecode.mins  = (uint8_t)(word.ltc.mins_units  + (word.ltc.mins_tens  * 10));
        //g_timecode.hours = (uint8_t)(word.ltc.hours_units + (word.ltc.hours_tens * 10));
        GateMutex_leave(gateMutex0, key);

        Task_sleep(1000);

        /* Assert the interrupt line to notify host packet is ready  */
        if (g_bPostInterrupts)
        {
            //GPIO_write(Board_SMPTE_INT_N, PIN_LOW);
        }
    }
#endif

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
    // Increment the edge counter
    gEdgeSeq++;

    /* Calculate the pulse period from rising edge to falling edge */
    if (g_uiLowCount > g_uiHighCount)
        g_uiPeriod = g_uiLowCount - g_uiHighCount;
    else
        g_uiPeriod = g_uiHighCount - g_uiLowCount;

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

/* End-Of-File */
