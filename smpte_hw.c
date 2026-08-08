/*
 * smpte_hw.c
 *
 * TM4C123AE6PM hardware setup for the LTC decoder using GPTM hardware
 * edge-time capture on Wide Timer 0, sub-timer A (WT0CCP0), input pin
 * PC4. The timer latches the tick count into a capture register at the
 * instant of the pin edge; the CPU only has to read that register out
 * whenever it eventually gets around to servicing the interrupt. That
 * removes Hwi-entry latency/jitter from the timestamp entirely -- the
 * only remaining uncertainty is the ~2-clock input synchronizer delay,
 * which is fixed and common-mode to every edge.
 *
 * Wide timers on TM4C123 are natively 32-bit per sub-timer (vs 16-bit for
 * the standard Timer0-5 blocks), so edge-time capture mode here gives a
 * full 32-bit free-running range without needing the prescaler trick the
 * standard-timer version required -- at 80 MHz that wraps roughly every
 * 53 seconds, several orders of magnitude longer than any real LTC
 * edge-to-edge interval (~40-2000us across normal play and fast-shuttle
 * speeds). tickMask is set to the full 0xFFFFFFFF accordingly.
 *
 * PC4/WT0CCP0 is the standard Wide Timer 0 CCP0 pin mapping on TM4C123
 * parts. Double-check it against the TM4C123AE6PM datasheet's "GPIO
 * Pins And Alternate Functions" table for your specific package/pin
 * count before wiring it up -- pin availability varies across TM4C123
 * suffix/package variants and this hasn't been verified against that
 * specific part's datasheet.
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
 *
 * -- Start / stop --
 *
 * SMPTE_HW_init() does one-time setup only (pin mux, timer
 * configuration, Hwi/Clock construction) and leaves everything stopped.
 * Call SMPTE_HW_start() to begin capturing and decoding, and
 * SMPTE_HW_stop() to halt it (timer counter stopped, capture interrupt
 * masked at the NVIC, watchdog Clock stopped). Both are idempotent and
 * safe to call repeatedly. Stopping does not clear the last decoded
 * dec->tc value; starting resets decoder state for a clean resync.
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
#include <file.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "Board.h"
#include "STC_SMPTE.h"

/* Wide Timer 0 sub-timer A is natively 32-bit in capture mode -- no
 * prescale extension needed, unlike the standard-timer version.
 */

#define SMPTE_CAPTURE_TICK_MASK   0xFFFFFFFFu

/* Watchdog tuning: declare signal loss after this many ms with zero
 * edges. Comfortably above any real LTC half-bit period (~200-2000us)
 * with margin for slow-motion scrubbing; tighten it if your application
 * needs a faster "signal lost" reaction.
 */

#define SMPTE_WATCHDOG_PERIOD_MS    5u
#define SMPTE_WATCHDOG_TIMEOUT_MS   20u

/*
 * Static Data Items
 */

static volatile uint32_t gEdgeSeq         = 0;   /* bumped once per capture ISR */
static uint32_t          gLastSeenEdgeSeq = 0;
static uint32_t          gStaleMs         = 0;

static bool              gRunning = false;
static uint32_t          gTimerHz;
static SMPTE_Decoder     gLtcDecoder;
static Hwi_Struct        gCaptureHwiStruct;
static Clock_Struct      gWatchdogClockStruct;

/*
 * WTIMER0A capture interrupt. Runs in Hwi context. The edge time itself
 * was already latched by hardware the instant the pin transitioned; all
 * this ISR does is clear the flag and read the latched value out, so its
 * own scheduling latency doesn't touch the timestamp's accuracy.
 */

static void timerCaptureHwi(UArg arg)
{
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);
    uint32_t capturedTick = TimerValueGet(WTIMER0_BASE, TIMER_A);
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

/*
 * One-time setup: pin mux, timer configuration, Hwi/Clock construction.
 * Leaves the timer stopped and the capture interrupt masked -- call
 * SMPTE_HW_start() to actually begin decoding.
 */

void SMPTE_HW_init(void)
{
    SMPTE_LTC_init(&gLtcDecoder);

    gLtcDecoder.tickMask = SMPTE_CAPTURE_TICK_MASK;

    gTimerHz = SysCtlClockGet();

    /* --- route the LTC input pin (PC4) to Wide Timer 0's CCP0 capture input --- */

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);

    /* --- configure WTIMER0, sub-timer A, as 32-bit edge-time capture,
     * up-counting --- */

    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0));

    TimerConfigure(WTIMER0_BASE, TIMER_CFG_A_CAP_TIME_UP);
    TimerLoadSet(WTIMER0_BASE, TIMER_A, 0xFFFFFFFFu);

    /* capture on both rising and falling edges -- biphase-mark decode
     * needs the interval between every transition, not just one polarity
     */
    TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT);
    /* timer left disabled here -- SMPTE_HW_start() enables it */

    /* --- construct (but do not yet enable) the capture interrupt --- */

    Error_Block eb;
    Error_init(&eb);
    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);
    hwiParams.priority   = 0x40;   /* tune to your system's priority scheme */
    hwiParams.enableInt  = false;  /* stay masked until SMPTE_HW_start() */

    Hwi_construct(&gCaptureHwiStruct, INT_WTIMER0A, timerCaptureHwi, &hwiParams, &eb);

    if (Error_check(&eb)) {
        System_abort("SMPTE_HW_init: failed to construct WTIMER0A capture Hwi\n");
    }

    /* --- construct (but do not yet start) the signal-loss watchdog --- */

    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.period    = SMPTE_WATCHDOG_PERIOD_MS;   /* assumes default 1ms Clock tick */
    clockParams.startFlag = false;

    Clock_construct(&gWatchdogClockStruct, watchdogClockFxn, SMPTE_WATCHDOG_PERIOD_MS, &clockParams);
}

/* Begin (or resume) capturing and decoding. Resets decoder state for a
 * clean resync. Safe to call again while already running (no-op).
 */

void SMPTE_HW_start(void)
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
        Hwi_enableInterrupt(INT_WTIMER0A);

        Clock_start(Clock_handle(&gWatchdogClockStruct));

        gRunning = true;
    }
}

/* Halt capturing and decoding: timer counter stopped, capture interrupt
 * masked, watchdog stopped. Leaves the last decoded dec->tc value
 * untouched. Safe to call again while already stopped (no-op).
 */

void SMPTE_HW_stop(void)
{
    if (gRunning)
    {
        Clock_stop(Clock_handle(&gWatchdogClockStruct));

        Hwi_disableInterrupt(INT_WTIMER0A);
        TimerDisable(WTIMER0_BASE, TIMER_A);

        /* Status LED */
        GPIO_write(Board_STAT_LED, Board_LED_ON);

        /* SMPTE input mute on */
        GPIO_write(Board_SMPTE_MUTE, PIN_LOW);
        GPIO_write(Board_FRAME_SYNC, PIN_LOW);
        GPIO_write(Board_DIRECTION, PIN_LOW);
        GPIO_write(Board_SMPTE_INT_N, PIN_HIGH);
        GPIO_write(Board_BUSY_N, PIN_HIGH);

        gRunning = false;
    }
}

bool SMPTE_HW_isRunning(void)
{
    return gRunning;
}

SMPTE_Decoder *SMPTE_HW_getDecoder(void)
{
    return &gLtcDecoder;
}

uint32_t SMPTE_HW_getTimerHz(void)
{
    return gTimerHz;
}
