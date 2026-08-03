/*
 * smpte_hw.c
 *
 * TM4C hardware setup for the LTC decoder using GPTM hardware edge-time
 * capture instead of a GPIO-interrupt + software timestamp. The timer
 * itself latches the tick count into a capture register at the instant
 * of the pin edge; the CPU only has to read that register out whenever
 * it eventually gets around to servicing the interrupt. That removes
 * Hwi-entry latency/jitter from the timestamp entirely -- the only
 * remaining uncertainty is the ~2-clock input synchronizer delay, which
 * is fixed and common-mode to every edge.
 *
 * Uses Timer0, sub-timer A, in 16-bit "Input Edge-Time" mode with the
 * 8-bit prescaler enabled, giving a 24-bit free-running capture counter
 * clocked at the full system clock. 24 bits at, say, 80 MHz wraps every
 * ~210ms -- comfortably longer than any real LTC edge-to-edge interval
 * (roughly 40-2000us across normal play and fast-shuttle speeds), so a
 * single-wraparound-safe masked subtraction in the decode engine
 * (SMPTE_Decoder.tickMask) is all that's needed to handle rollover.
 *
 * Pin: this example uses PB6 (T0CCP0), a common Timer0 CCP0 pin on
 * TM4C123-family parts. If PB6 isn't free on your board, or you're on a
 * TM4C129-family part, pick a different CCP-capable pin/timer block from
 * your part's datasheet ("GPIO Pins And Alternate Functions" table) and
 * update GPIOPinConfigure()/GPIOPinTypeTimer()/TIMERx_BASE/INT_TIMERxA
 * below to match. Whichever pin you choose is dedicated to the timer's
 * alternate function -- it's no longer usable as a general-purpose GPIO
 * interrupt input, so this replaces (not supplements) a GPIO-interrupt
 * based setup for the same signal.
 *
 * Wiring assumption unchanged from before: raw LTC audio must be squared
 * to a clean 0/3.3V digital signal by an external comparator or biased
 * Schmitt-trigger inverter ahead of this pin.
 *
 * -- Signal-loss watchdog --
 *
 * A GPTM in edge-time capture mode does not offer a way to read a live
 * "now" count -- TimerValueGet() on this timer only ever returns the
 * value from the last capture, so if edges stop arriving entirely there
 * is nothing to read that would notice. Detecting that requires a
 * separate, independent time reference: a periodic TI-RTOS Clock counts
 * how many watchdog periods have passed since the capture ISR last ran
 * (tracked via a simple edge-sequence counter), and calls
 * SMPTE_LTC_onTimeout() once that exceeds SMPTE_WATCHDOG_TIMEOUT_MS.
 * Tune that constant to comfortably exceed your slowest expected valid
 * half-bit period (normal LTC is ~200-2000us; the default here allows a
 * generous margin for slow-motion scrubbing).
 */
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
#include <ti/sysbios/knl/Swi.h>
//#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

#include <driverlib/pin_map.h>
#include <driverlib/timer.h>
#include <driverlib/sysctl.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ssi.h>
#include <inc/hw_i2c.h>

#include "smpte_ltc.h"

static SMPTE_Decoder gLtcDecoder;
static uint32_t      gTimerHz;
static Hwi_Struct    gCaptureHwiStruct;
static Clock_Struct  gWatchdogClockStruct;

/* 16-bit counter + 8-bit prescaler = 24-bit capture range */
#define SMPTE_CAPTURE_TICK_MASK   0x00FFFFFFu

/* Watchdog tuning: declare signal loss after this many ms with zero
 * edges. Comfortably above any real LTC half-bit period (~200-2000us)
 * with margin for slow-motion scrubbing; tighten it if your application
 * needs a faster "signal lost" reaction. */
#define SMPTE_WATCHDOG_PERIOD_MS    5u
#define SMPTE_WATCHDOG_TIMEOUT_MS   20u

static volatile uint32_t gEdgeSeq         = 0;   /* bumped once per capture ISR */
static uint32_t          gLastSeenEdgeSeq = 0;
static uint32_t          gStaleMs         = 0;

/*
 * Timer0A capture interrupt. Runs in Hwi context. The edge time itself
 * was already latched by hardware the instant the pin transitioned; all
 * this ISR does is clear the flag and read the latched value out, so its
 * own scheduling latency doesn't touch the timestamp's accuracy.
 */
static void timerCaptureHwi(UArg arg)
{
    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);
    uint32_t capturedTick = TimerValueGet(TIMER0_BASE, TIMER_A);
    gEdgeSeq++;
    SMPTE_LTC_onEdge(&gLtcDecoder, capturedTick);
}

/*
 * Runs periodically in Swi context (TI-RTOS Clock functions run at Swi
 * level). Purely counts elapsed watchdog periods since gEdgeSeq last
 * changed -- deliberately independent of the capture timer itself, per
 * the note in the file header.
 */
static void watchdogClockFxn(UArg arg)
{
    if (gEdgeSeq == gLastSeenEdgeSeq) {
        if (gStaleMs < SMPTE_WATCHDOG_TIMEOUT_MS) {
            gStaleMs += SMPTE_WATCHDOG_PERIOD_MS;
            if (gStaleMs >= SMPTE_WATCHDOG_TIMEOUT_MS) {
                SMPTE_LTC_onTimeout(&gLtcDecoder);   /* fires once per stall event */
            }
        }
    } else {
        gLastSeenEdgeSeq = gEdgeSeq;
        gStaleMs         = 0;
    }
}

void SMPTE_HW_init(void)
{
    /* --- decoder state --- */
    SMPTE_LTC_init(&gLtcDecoder);
    gLtcDecoder.tickMask = SMPTE_CAPTURE_TICK_MASK;

    gTimerHz = SysCtlClockGet();

    /* --- route the LTC input pin to Timer0's CCP0 capture input --- */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralWaitReady(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB6_T0CCP0);
    //GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

    /* --- configure Timer0A as 16-bit edge-time capture, up-counting,
     * with the 8-bit prescaler enabled for the full 24-bit range --- */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralWaitReady(SYSCTL_PERIPH_TIMER0);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_CAP_TIME_UP);
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xFF);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFF);

    /* capture on both rising and falling edges -- biphase-mark decode
     * needs the interval between every transition, not just one polarity */
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    /* --- register the capture interrupt with TI-RTOS --- */
    Error_Block eb;
    Error_init(&eb);
    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);
    hwiParams.priority = 0x40;   /* tune to your system's priority scheme */
    Hwi_construct(&gCaptureHwiStruct, INT_TIMER0A, timerCaptureHwi, &hwiParams, &eb);
    if (Error_check(&eb)) {
        System_abort("SMPTE_HW_init: failed to construct Timer0A capture Hwi\n");
    }

    /* --- signal-loss watchdog: periodic Clock, independent of the
     * capture timer (see file header) --- */
    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.period    = SMPTE_WATCHDOG_PERIOD_MS;   /* assumes default 1ms Clock tick */
    clockParams.startFlag = TRUE;
    Clock_construct(&gWatchdogClockStruct, watchdogClockFxn,
                     SMPTE_WATCHDOG_PERIOD_MS, &clockParams);
}

SMPTE_Decoder *SMPTE_HW_getDecoder(void)
{
    return &gLtcDecoder;
}

uint32_t SMPTE_HW_getTimerHz(void)
{
    return gTimerHz;
}
