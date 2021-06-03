/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2016, RTZ Professional Audio, LLC
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
#include "Utils.h"
#include "libltc\ltc.h"
#include "libltc\encoder.h"

/* Constants */

/* Global Data Items */

SYSPARMS g_sys;

/* Static Data Items */

uint32_t g_systemClock;

/* SMPTE Data Items */

bool running = false;
SMPTETimecode g_smpte_time;
LTCFrame g_smpte_frame;
LTCFrame g_smpte_ones;
LTCFrame g_smpte_zeros;

volatile uint8_t  g_bitState = 0;
volatile uint32_t g_bitCount = 0;
volatile uint8_t  g_halfBit = 0;

const char timezone[6] = "+0100";

/* Static Function Prototypes */

Int main();
void SMPTE_Start();
void SMPTE_Stop(void);
Void SlaveTask(UArg a0, UArg a1);
Void Timer1AIntHandler(UArg arg);
Void Timer1BIntHandler(UArg arg);

//*****************************************************************************
// Main Program Entry Point
//*****************************************************************************

Int main()
{
    Error_Block eb;
    Task_Params taskParams;

    g_systemClock = SysCtlClockGet();

    Board_initGeneral();
    Board_initGPIO();
    Board_initSPI();

    /* Now start the main application button polling task */

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1500;
    taskParams.priority  = 10;

    if (Task_create(SlaveTask, &taskParams, &eb) == NULL)
        System_abort("SlaveTask!\n");

    BIOS_start();    /* does not return */

    return(0);
}

//*****************************************************************************
//
//*****************************************************************************

Void SlaveTask(UArg a0, UArg a1)
{
    SPI_Params spiParams;
    SPI_Handle hSlave;

    /* Initialize the default servo and program data values */
    memset(&g_sys, 0, sizeof(SYSPARMS));

    /* Read system parameters from EEPROM */
    //SysParamsRead(&g_sys);
    InitSysDefaults(&g_sys);

    /* Open SLAVE SPI port from STC motherboard
     * 1 Mhz, Moto fmt, polarity 1, phase 0
     */

    SPI_Params_init(&spiParams);
    spiParams.transferMode  = SPI_MODE_BLOCKING;
    spiParams.mode          = SPI_SLAVE;
    spiParams.frameFormat   = SPI_POL1_PHA0;
    spiParams.bitRate       = 250000;
    spiParams.dataSize      = 16;
    spiParams.transferCallbackFxn = NULL;

    hSlave = SPI_open(STC_SMPTE_SPI0, &spiParams);

    if (hSlave == NULL)
        System_abort("Error initializing SPI0\n");

    /****************************************************************
     * Enter the main application button processing loop forever.
     ****************************************************************/

    /* Map the timer interrupt handlers. We don't make sys/bios calls
     * from these interrupt handlers and there is no need to create a
     * context handler with stack swapping for these. These handlers
     * just update some globals variables and need to execute as
     * quickly and efficiently as possible.
     */
    Hwi_plug(INT_WTIMER1A, Timer1AIntHandler);
    Hwi_plug(INT_WTIMER1B, Timer1BIntHandler);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);

    // Configure TIMER1B as a 16-bit periodic timer.
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_PERIODIC);

    //SMPTE_Start();

    for(;;)
    {
        bool success;
        uint16_t ulCommand;
        uint16_t ulReply = 0;
        SPI_Transaction transaction;

        transaction.count = 1;
        transaction.rxBuf = (Ptr)&ulCommand;
        transaction.txBuf = (Ptr)&ulReply;

        /* Send the SPI transaction */
        success = SPI_transfer(hSlave, &transaction);

        if (!success)
        {
            System_printf("SPI slave rx failed\n");
            System_flush();
        }
        else
        {
            //System_printf("SPI slave rx %04x\n", ulCommand);
            //System_flush();

            switch(ulCommand)
            {
            case 0xFE21:
                SMPTE_Start();
                break;

            case 0xFE20:
                SMPTE_Stop();
                break;
            }
        }
    }
}


void SMPTE_Start(void)
{
    g_smpte_time.hours  = 23;
    g_smpte_time.mins   = 59;
    g_smpte_time.secs   = 59;
    g_smpte_time.frame  = 0;
    //g_smpte_time.years  = 8;
    //g_smpte_time.months = 12;
    //g_smpte_time.days   = 31;

    memset(&g_smpte_ones, 0xFF, sizeof(LTCFrame));
    memset(&g_smpte_zeros, 0x00, sizeof(LTCFrame));

    strcpy(g_smpte_time.timezone, timezone);

    ltc_frame_reset(&g_smpte_frame);

    ltc_time_to_frame(&g_smpte_frame, &g_smpte_time, LTC_TV_525_60, 0);

    //memcpy(&g_smpte_frame, &g_smpte_ones, sizeof(LTCFrame));

    /* Enable the signal out relay to connect the SMPTE
     * output signal to channel 24 on the tape machine.
     */
    GPIO_write(Board_RELAY, Board_RELAY_ON);
    Task_sleep(100);
    // Turn the LED on to indicate active
    GPIO_write(Board_STAT_LED, Board_LED_ON);
    // SMPTE output pin low initially
    GPIO_write(Board_SMPTE_OUT, PIN_LOW);

    // x2 bit clocks:
    // 24fps  = 3840Hz
    // 25fps = 4000Hz
    // 30fps = 4800Hz

    g_bitCount = g_halfBit = 0;

    uint8_t* frame = (uint8_t*)&g_smpte_frame;

    g_bitState = ((frame[g_bitCount/8]) >> (g_bitCount % 8)) & 0x01;

    // Set the TIMER1B load value to 1ms.
    TimerLoadSet(WTIMER1_BASE, TIMER_A, g_systemClock/4800);
    // Configure the TIMER1B interrupt for timer timeout.
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Enable the TIMER1B interrupt on the processor (NVIC).
    IntEnable(INT_WTIMER1A);
    // Enable TIMER1A.
    TimerEnable(WTIMER1_BASE, TIMER_A);
}

void SMPTE_Stop(void)
{
    // Disable TIMER1A.
    TimerDisable(WTIMER1_BASE, TIMER_A);
    // Disable the TIMER1A interrupt.
    IntDisable(INT_WTIMER1A);
    // Turn off TIMER1B interrupt.
    TimerIntDisable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Clear any pending interrupt flag.
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    Task_sleep(100);
    // SMPTE output pin low
    GPIO_write(Board_SMPTE_OUT, PIN_LOW);
    // Relay and LED off
    GPIO_write(Board_RELAY, Board_RELAY_OFF);
    GPIO_write(Board_STAT_LED, Board_LED_OFF);
}

//*****************************************************************************
// WTIMER INTERRUPT HANDLERS
//*****************************************************************************

#if 0
Void Timer1AIntHandler(UArg arg)
{
    uint8_t* pframe = (uint8_t*)&g_smpte_frame;
    uint8_t bit, mask;
    uint32_t i, s;

    // Clear the timer interrupt flag.
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* Bi-phase state machine */

    i = g_bitCount / 8;     /* byte index into frame */
    s = g_bitCount % 8;     /* shift to current bit  */

    mask = pframe[i];

    bit = ((mask >> s) & 0x01) ? 1 : 0;

    switch(g_state)
    {
    case 0:
        /* flip bit high initially */
        GPIO_toggle(Board_SMPTE_OUT);
        ++g_state;
        break;

    case 1:
        /* comment this out to get 1.2khz 50/50 duty cycle wave form */
        /* Toggle on the cell boundary and again at the half cell time
         * if the bit is a 1, otherwise not.
         */
        if (bit)
            GPIO_toggle(Board_SMPTE_OUT);
        /* Otherwise, don't flip for duty cycle period */
        ++g_state;
        break;

    case 2:
        /* toggle again to complete falling edge */
        GPIO_toggle(Board_SMPTE_OUT);
        ++g_state;
        break;

    case 3:
        /* delay again for duty cycle reset state to start next bit */
        g_state = 0;
        if (++g_bitCount >= LTC_FRAME_BIT_COUNT)
        {
            g_bitCount = 0;
            ltc_frame_increment(&g_smpte_frame, 30, LTC_TV_625_50, 0);
        }
        break;
    }
}

#else

Void Timer1AIntHandler(UArg arg)
{
    // Clear the timer interrupt flag.
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    uint8_t* frame = (uint8_t*)&g_smpte_frame;

    g_bitState = ((frame[g_bitCount/8]) >> (g_bitCount % 8)) & 0x01;

    g_halfBit = !g_halfBit;                 // Flip the halfbit state

    if (g_halfBit)
    {                                       // First half bit (always flip at start of new bit)
        GPIO_toggle(Board_SMPTE_OUT);
    }
    else
    {                                       // Half bit is false (second half of bit)
        if (g_bitState)
        {                                   // 1 bit so change half way
            GPIO_toggle(Board_SMPTE_OUT);
        }

        if (g_bitCount >= 80)
        {
#if 0
            // Last bit sent?
            if (continuous)
            {
                tcTime = micros() - tcStartTime;

                tcStartTime = micros();

                if (tcTime > 40000)
                {                           // If greater than 40ms
                    tcSpeed--;              // Reduce timer1 compare threshold
                }
                else
                {
                    tcSpeed++;              // Increment timer1 compare threshold
                }
                OCR1A = tcSpeed;            // And store it
            }
            else
            {
                tcStopping = true;          // Signal we are stopping
            }
#endif
            ltc_frame_increment(&g_smpte_frame, 30, LTC_TV_625_50, 0);

            g_bitCount = 0;
        }

        uint8_t* frame = (uint8_t*)&g_smpte_frame;

        g_bitState = ((frame[g_bitCount/8]) >> (g_bitCount % 8)) & 0x01;

        ++g_bitCount;
    }
}
#endif

Void Timer1BIntHandler(UArg arg)
{
    // Clear the timer interrupt flag.
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
}

/* End-Of-File */
