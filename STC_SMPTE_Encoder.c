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

/* Returns the state of a bit number in the frame buffer */
#define FRAME_GETBIT(framebuf, bitnum)    ( (((framebuf[bitnum / 8]) >> (bitnum % 8)) & 0x01) )

/* SMPTE Encoder variables */
bool g_encoderEnabled = false;
uint8_t  g_txBitState = 0;
uint8_t  g_txHalfBit = 0;
uint32_t g_txBitCount = 0;
uint32_t g_txFrameCount = 0;
SMPTETimecode g_txTime;
LTCFrame g_txFrame;

/* Global Config variables */
extern SYSCFG g_cfg;
extern uint32_t g_systemClock;

//*****************************************************************************
//********************** SMPTE ENCODER SUPPORT ********************************
//*****************************************************************************

void SMPTE_Encoder_Reset(void)
{
    /* Zero out the starting time struct */
    memset(&g_txTime, 0, sizeof(g_txTime));

    /* Set default time zone */
    strcpy(g_txTime.timezone, g_cfg.timezone);

  //g_txTime.years  = 0;        /* LTC date uses 2-digit year 00-99  */
  //g_txTime.months = 1;        /* valid months are 1..12            */
  //g_txTime.days   = 1;        /* day of month 1..31                */

    g_txTime.hours  = 0;        /* hour 0..23                        */
    g_txTime.mins   = 0;        /* minute 0..60                      */
    g_txTime.secs   = 0;        /* second 0..60                      */
    g_txTime.frame  = 0;        /* sub-second frame 0..(FPS - 1)     */

    /* Reset the frame buffer */
    ltc_frame_reset(&g_txFrame);
}

//*****************************************************************************
// Start the SMPTE Generator
//*****************************************************************************

int SMPTE_Encoder_Start(void)
{
    uint32_t clockrate;

    if (g_encoderEnabled)
        return -1;

    /* Turn the LED on to indicate active */
    GPIO_write(Board_STAT_LED, Board_LED_ON);

    /* Set the starting time members in the SMPTE tx frame buffer */
    ltc_time_to_frame(&g_txFrame, &g_txTime, LTC_TV_525_60, 0);

    /* Setup timer interrupt 2x bit clocks:
     * 24 fps = 3840Hz
     * 25 fps = 4000Hz
     * 30 fps = 4800Hz
     */

    switch(g_cfg.frame_rate)
    {
    case 24:
        /* 24 fps */
        clockrate = 3840;
        break;

    case 25:
        /* 25 fps */
        clockrate = 4000;
        break;

    case 30:
        /* 30 fps */
        clockrate = 4800;
        break;

    default:
        /* default to 30 fps */
        g_cfg.frame_rate = 30;
        clockrate = 4800;
        break;
    }

    g_txFrameCount = 0;
    g_txBitCount = 0;
    g_txHalfBit = 0;
    g_encoderEnabled = true;

    /* Pre-load the state of the first bit in the frame */
    g_txBitState = FRAME_GETBIT(((uint8_t*)&g_txFrame), g_txBitCount);

    /* Enable the signal out relay to connect the SMPTE
     * output signal to channel 24 on the tape machine.
     */
    GPIO_write(Board_RELAY, Board_RELAY_ON);
    Task_sleep(50);

    /* SMPTE output pin low initially */
    GPIO_write(Board_SMPTE_OUT, PIN_LOW);

    /* Set TIMER_A clock rate */
    TimerLoadSet(WTIMER1_BASE, TIMER_A, g_systemClock/clockrate);

    /* Configure the WTIMER1A interrupt for timer timeout */
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the TIMER1B interrupt on the processor (NVIC) */
    IntEnable(INT_WTIMER1A);

    /* Enable TIMER1A */
    TimerEnable(WTIMER1_BASE, TIMER_A);

    return 0;
}

//*****************************************************************************
// Stop the SMPTE Generator
//*****************************************************************************

int SMPTE_Encoder_Stop(void)
{
    if (!g_encoderEnabled)
        return 0;

    /* Disable TIMER1A */
    TimerDisable(WTIMER1_BASE, TIMER_A);

    /* Disable the TIMER1A interrupt */
    IntDisable(INT_WTIMER1A);

    /* Turn off TIMER1B interrupt */
    TimerIntDisable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* Clear any pending interrupt flag */
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* SMPTE output pin low */
    GPIO_write(Board_SMPTE_OUT, PIN_LOW);

    /* Relay off */
    GPIO_write(Board_RELAY, Board_RELAY_OFF);
    GPIO_write(Board_STAT_LED, Board_LED_ON);

    g_encoderEnabled = false;

    return 1;
}
//*****************************************************************************
// SMPTE Generator WTIMER Interrupt Handler
//*****************************************************************************

Void WTimer1AIntHandler(UArg arg)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* Set drop frame bit if enabled */
    g_txFrame.dfbit = (g_cfg.drop_frame) ? 1 : 0;

    /* Flip half bit state indicator */
    g_txHalfBit = !g_txHalfBit;

    /* First half bit true, the flip at start of new bit */
    if (g_txHalfBit)
    {
        GPIO_toggle(Board_SMPTE_OUT);
    }
    else
    {
        /* Half bit false indicates second half of bit */
        if (g_txBitState)
        {
            /* A high 1-bit changes half way */
            GPIO_toggle(Board_SMPTE_OUT);
        }

        /* Check if a full 80-bit frame has been transmitted */
        if (g_txBitCount >= LTC_FRAME_BIT_COUNT)
        {
            /* If so, then increment the frame time */
            ltc_frame_increment(&g_txFrame, g_cfg.frame_rate, LTC_TV_625_50, 0);

            /* Increment frame counter */
            ++g_txFrameCount;

            /* Reset frame bit counter */
            g_txBitCount = 0;

            /* Toggle the LED on each packet received */
            GPIO_toggle(Board_STAT_LED);
        }

        /* Pre-load the state of the next bit to go out */
        g_txBitState = FRAME_GETBIT(((uint8_t*)&g_txFrame), g_txBitCount);

        /* Increment the frame bit counter */
        ++g_txBitCount;
    }
}

Void WTimer1BIntHandler(UArg arg)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
}

/* End-Of-File */
