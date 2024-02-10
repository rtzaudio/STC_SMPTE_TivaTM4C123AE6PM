/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2021, RTZ Professional Audio, LLC
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

/* XDCtools Header files */
#include "Board.h"
#include "STC_SMPTE.h"
#include "STC_SMPTE_SPI.h"
#include "libltc\ltc.h"

/* Constants and Macros */
extern SYSCFG g_cfg;
extern  uint32_t g_systemClock;

/* SMPTE Decoder variables */
SMPTETimecode g_rxTime;
LTCFrame g_rxFrame;
bool g_decoderEnabled = false;

volatile uint32_t g_uiPeriod = 0;
volatile uint32_t g_uiHighCount = 0;
volatile uint32_t g_uiLowCount = 0;
volatile uint32_t g_uiAveragePeriod = 0;

volatile int g_rxBitCount = 0;

volatile bool first_transition = false;
volatile int new_bit = 0;

//the sync word is available in bit 64 - 79,
//see https://en.wikipedia.org/wiki/Linear_timecode

const uint64_t sync_word_fwd = 0b0011111111111101;
const uint64_t sync_word_rev = 0b1011111111111100;
const uint64_t sync_mask = 0b1111111111111111;

static void HandleEdgeChange(void);

//*****************************************************************************
//********************** SMPTE DECODER SUPPORT ********************************
//*****************************************************************************

Void SMPTE_Decoder_Reset(void)
{
    /* Zero out the starting time struct */
    memset(&g_rxTime, 0, sizeof(g_rxTime));

    /* Set default time zone */
    strcpy(g_rxTime.timezone, g_cfg.timezone);

    g_rxTime.years  = 24;       /* LTC date uses 2-digit year 00-99  */
    g_rxTime.months = 1;        /* valid months are 1..12            */
    g_rxTime.days   = 1;        /* day of month 1..31                */
    g_rxTime.hours  = 0;        /* hour 0..23                        */
    g_rxTime.mins   = 0;        /* minute 0..60                      */
    g_rxTime.secs   = 0;        /* second 0..60                      */
    g_rxTime.frame  = 0;        /* sub-second frame 0..(FPS - 1)     */

    /* Reset the frame buffer */
    ltc_frame_reset(&g_rxFrame);

    /* Set the starting time members in the smpte frame */
    ltc_time_to_frame(&g_rxFrame, &g_rxTime, LTC_TV_525_60, 0);
}

//*****************************************************************************
// Initialize and start the SMPTE decoder edge timer interrupts
//*****************************************************************************

int SMPTE_Decoder_Start(void)
{
    g_decoderEnabled = true;

    /* Status LED on */
    GPIO_write(Board_STAT_LED, Board_LED_ON);

    GPIO_write(Board_FRAME_SYNC, PIN_LOW);

    /* Make sure the decoder interrupt isn't enabled */
    SMPTE_Decoder_Stop();

    /* Zero out the starting time struct */
    memset(&g_rxTime, 0, sizeof(g_rxTime));

    /* Reset global variables */
    new_bit = 0;
    first_transition = false;
    g_rxBitCount = g_uiPeriod = g_uiLowCount = g_uiHighCount = 0;

    /* Reset the frame buffer */
    ltc_frame_reset(&g_rxFrame);

    /* Configure the GPIO for the Timer peripheral */
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* Configure the GPIO to be CCP pins for the Timer peripheral */
    GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    GPIOPinConfigure(GPIO_PC5_WT0CCP1);

    /* Initialize Timers A and B to both run as periodic up-count edge capture
     * This will split the 64-bit timer into two 32-bit timers.
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

    /* Configure Timer A to trigger on a Positive Edge and configure
     * Timer B to trigger on a Negative Edge.
     */
    TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(WTIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

    /* Clear the interrupt status flag.  This is done to make sure the
     * interrupt flag is cleared before we enable it.
     */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    /* Enable the Timer A and B interrupts for Capture Events */
    TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    /* Enable the interrupts for Timer A and Timer B on the processor (NVIC) */
    IntEnable(INT_WTIMER0A);
    IntEnable(INT_WTIMER0B);

    /* Enable both Timer A and Timer B to begin the application */
    TimerEnable(WTIMER0_BASE, TIMER_BOTH);

    /* SMPTE input mute off */
    GPIO_write(Board_SMPTE_MUTE, PIN_HIGH);

    return 0;
}

//*****************************************************************************
// Stop the SMPTE Decoder
//*****************************************************************************

int SMPTE_Decoder_Stop(void)
{
    /* SMPTE input mute on */
    GPIO_write(Board_SMPTE_MUTE, PIN_LOW);

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

    g_decoderEnabled = false;

    return 0;
}

//*****************************************************************************
// SMPTE Input Edge Timing Interrupts (32-BIT WIDE TIMER IMPLEMENTATION)
//*****************************************************************************

/* Rising Edge Interrupt (Start of Pulse) */
Void WTimer0AIntHandler(UArg arg)
{
    /* Echo high pin change to SYNC pin */
    GPIO_write(Board_FRAME_SYNC, PIN_HIGH);

    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);

    /* Store the start time */
    g_uiHighCount = TimerValueGet(WTIMER0_BASE, TIMER_A);

    HandleEdgeChange();
}

/* Falling Edge Interrupt (End of Pulse) */
Void WTimer0BIntHandler(UArg arg)
{
    /* Echo low pin change to SYNC pin */
    GPIO_write(Board_FRAME_SYNC, PIN_LOW);

    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);

    /* Store the end time */
    g_uiLowCount = TimerValueGet(WTIMER0_BASE, TIMER_B);

    HandleEdgeChange();
}

//*****************************************************************************
//
//*****************************************************************************

void HandleEdgeChange(void)
{
    uint32_t t;
    bool new_bit_available = false;

    if (g_uiLowCount > g_uiHighCount)
        g_uiPeriod = g_uiLowCount - g_uiHighCount;
    else
        g_uiPeriod = g_uiHighCount - g_uiLowCount;

    /*  pulse width: 416.7us(30fps)
     *  0-bit x 30fps --> 416.7us/bit
     *
     * Then compare with average lenth of a one bit (haveing some margin
     * of at least of 1/4 of average, finally decide if it is a
     * long (0) or a short (1)
     */

    /* Calculate the average pulse period */
    g_uiAveragePeriod = (g_uiPeriod >> 2);

    /* Normally we'd divide the period count by 80 here to get the pulse
     * time in microseconds. However, we scale all the other timing
     * constants by 80 instead to avoid this divison.
     */
    t  = g_uiPeriod;

    /* Now look at the period and decide if it's a one or zero */
    if (t >= ONE_TIME_MIN && t < ONE_TIME_MAX)
    {
        if (first_transition)
        {
            /* First bit transition */
            first_transition = false;
        }
        else
        {
            // Second bit transition
            first_transition = true;
            new_bit_available = true;
            new_bit = 1;
        }
    }
    else if (t >= ZERO_TIME_MIN && t < ZERO_TIME_MAX)
    {
        new_bit_available = true;
        first_transition = true;
        new_bit = 0;
    }
    else
    {
        //first bit change? or something is wrong!
    }

    if (new_bit_available)
    {
        /* Shift new bit into the 80-bit frame receive buffer */

        ltcframe_t* p = (ltcframe_t*)&g_rxFrame;

        uint64_t w64 = p->data;
        uint16_t w16 = p->sync;

        /* shift the sync bits */
        w16 = w16 << 1;

        /* carry bit 63 into sync bit zero if needed */
        if (w64 & 0x8000000000000000)
            w16 |= 1;
        else
            w16 &= ~1;

        /* shift the frame word bits */
        w64 = w64 << 1;

        /* Add in the new data high bit if needed */
        if (new_bit)
            w64 |= 1;
        else
            w64 &= ~1;

        /* store shifted frame & sync bits back into frame buffer */
        p->data = w64;
        p->sync = w16;

        g_rxBitCount++;

        /* Must see the SMPTE sync word at the end of frame to consider it a
         * valid packet. If so, we parse out the frame members into our buffer.
         */

        if (g_rxBitCount >= LTC_FRAME_BIT_COUNT)
        {
            /* Toggle the LED on each packet received */
            GPIO_toggle(Board_STAT_LED);

            if (g_rxFrame.sync_word == sync_word_fwd)
            {
                /* Parse the buffer and get time members in the SMPTE frame */
                //ltc_frame_to_time(&g_rxTime, &g_rxFrame, 0);

                g_rxTime.hours = hour(p);       // g_rxFrame.hours_units + g_rxFrame.hours_tens * 10;
                g_rxTime.mins  = minute(p);    //g_rxFrame.mins_units  + g_rxFrame.mins_tens * 10;
                g_rxTime.secs  = second(p);    //g_rxFrame.secs_units  + g_rxFrame.secs_tens * 10;
                g_rxTime.frame = frame(p);  //g_rxFrame.frame_units + g_rxFrame.frame_tens * 10;

                /* Reset the pulse bit counter */
                g_rxBitCount = 0;
            }
            else if (g_rxFrame.sync_word == sync_word_rev)
            {
                /* Reset the pulse bit counter */
                g_rxBitCount = 0;
            }
        }
    }
}

//*****************************************************************************
//
//*****************************************************************************

int hour(ltcframe_t* ltc)
{
    return 10 * ((int) (ltc->data >> 56) & 0x03)  + ((int) (ltc->data >> 48) & 0x0f);
}

int minute(ltcframe_t* ltc)
{
    return 10 * ((int) (ltc->data >> 40) & 0x07)  + ((int) (ltc->data >> 32) & 0x0f);
}

int second(ltcframe_t* ltc)
{
    return 10 * ((int) (ltc->data >> 24) & 0x07)  + ((int) (ltc->data >> 16) & 0x0f);
}

int frame(ltcframe_t* ltc)
{
    return 10 * ((int) (ltc->data >>  8) & 0x03)  + ((int) (ltc->data >>  0) & 0x0f);
}

bool bit10(ltcframe_t* ltc)
{
    return (int) (ltc->data >> 10) & 0x01;
}

bool bit11(ltcframe_t* ltc)
{
    return (int) (ltc->data >> 11) & 0x01;
}

bool bit27(ltcframe_t* ltc)
{
    return (int) (ltc->data >> 27) & 0x01;
}

bool bit43(ltcframe_t* ltc)
{
    return (int) (ltc->data >> 43) & 0x01;
}

bool bit58(ltcframe_t* ltc)
{
    return (int) (ltc->data >> 58) & 0x01;
}

bool bit59(ltcframe_t* ltc)
{
    return (int) (ltc->data >> 59) & 0x01;
}

uint32_t userdata(ltcframe_t * ltc)
{
    return  ((int) (ltc->data >>  4) & 0x0000000fUL) | ((int) (ltc->data >>  8) & 0x000000f0UL) |
            ((int) (ltc->data >> 12) & 0x00000f00UL) | ((int) (ltc->data >> 16) & 0x0000f000UL) |
            ((int) (ltc->data >> 20) & 0x000f0000UL) | ((int) (ltc->data >> 24) & 0x00f00000UL) |
            ((int) (ltc->data >> 28) & 0x0f000000UL) | ((int) (ltc->data >> 32) & 0xf0000000UL);
}



#if 0

void LTC_SMPTE::parse_code()
{
    frame = (code[1] & 0x03) * 10 + (code[0] & 0x0f);
    sec   = (code[3] & 0x07) * 10 + (code[2] & 0x0f);
    min   = (code[5] & 0x07) * 10 + (code[4] & 0x0f);
    hour  = (code[7] & 0x03) * 10 + (code[6] & 0x0f);
    drop  = code[1] & (1<<2) ? 1 : 0;
    received = 1;
}

handleInterrupt() {
  noInterrupts(); // stop being interrupted (got to hurry not to miss the next one)
  long time = micros(); // record curent time (in micro-seconds)
  duration = time - lastTime; // Get the duration from the last interrupt
  ...
  compare with average lenth of a one (have some margin at least of 1/4 of average - see further)
  (Calculate 1/4 by bit shifting to be quick).
  decide if it is a long (0) or a short (1)
  ...
  lastTime = time;
  if is was a one {
     long average = duration; // average time for a one (short)
  }
}

#endif

/* End-Of-File */
