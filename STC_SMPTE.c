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
#include "Utils.h"
#include "libltc\ltc.h"

/* Constants and Macros */

/* Returns the state of a bit number in the frame buffer */
#define FRAME_BITSTATE(framebuf, bitnum)    ( (((framebuf[bitnum / 8]) >> (bitnum % 8)) & 0x01) )

/* Global Data Items */
SYSCFG g_cfg;

/* Static Data Items */
uint32_t g_systemClock;

/* SMPTE Data Items */
SMPTETimecode g_smpte_time;
LTCFrame g_smpte_frame;

static bool g_running = false;
static bool g_drop_frame = false;

volatile int g_frame_rate = 30;
volatile uint8_t  g_bitState = 0;
volatile uint8_t  g_halfBit = 0;
volatile uint32_t g_bitCount = 0;
volatile uint32_t g_frameCount = 0;

const char timezone[6] = "+0100";

/* Static Function Prototypes */

Int main();
void SMPTE_Generator_Reset(void);
int SMPTE_Generator_Start();
int SMPTE_Generator_Stop(void);
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

    return 0;
}

//*****************************************************************************
//
//*****************************************************************************

Void SlaveTask(UArg a0, UArg a1)
{
    bool success;
    uint16_t ulRequest;
    uint16_t ulReply;
    uint16_t ulDummy;
    SPI_Transaction transaction1;
    SPI_Transaction transaction2;
    SPI_Params spiParams;
    SPI_Handle hSlave;

    /* Set busy pin high to indicate busy status */
    GPIO_write(Board_BUSY, PIN_HIGH);

    /* Initialize the default servo and program data values */
    memset(&g_cfg, 0, sizeof(SYSCFG));

    /* Read system parameters from EEPROM */
    //SysParamsRead(&g_cfg);
    InitSysDefaults(&g_cfg);

    /* Open SLAVE SPI port from STC motherboard
     * 1 Mhz, Moto fmt, polarity 1, phase 0
     */
    SPI_Params_init(&spiParams);

    spiParams.transferMode  = SPI_MODE_BLOCKING;
    spiParams.mode          = SPI_SLAVE;
    spiParams.frameFormat   = SPI_POL1_PHA0;
    spiParams.bitRate       = 250000;
    spiParams.dataSize      = 16;

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

    /* Configure TIMER1B as a 16-bit periodic timer */
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_PERIODIC);

    /* Reset the SMPTE frame buffer to zeros */
    SMPTE_Generator_Reset();

    /*
     * Enter the main SPI slave processing loop
     */

    for(;;)
    {
        ulReply = ulRequest = ulDummy = 0;

        transaction1.count = 1;
        transaction1.txBuf = (Ptr)&ulDummy;
        transaction1.rxBuf = (Ptr)&ulRequest;

        /* Send the SPI transaction */

        GPIO_write(Board_BUSY, PIN_LOW);
        success = SPI_transfer(hSlave, &transaction1);
        GPIO_write(Board_BUSY, PIN_HIGH);

        if (!success)
        {
            System_printf("SPI slave rx failed\n");
            System_flush();
            /* Loop if error reading SPI! */
            continue;
        }

        uint8_t opcode = (ulRequest & SMPTE_REG_MASK) >> 8;

        if (opcode == SMPTE_REG_REVID)
        {
            /* ====================================================
             * SMPTE CARD REV/ID REGISTER (RO)
             * ====================================================
             */

            ulReply = SMPTE_REVID;

            /* Send the reply word back */
            transaction2.count = 1;
            transaction2.txBuf = (Ptr)&ulReply;
            transaction2.rxBuf = (Ptr)&ulDummy;

            /* Send the SPI transaction */
            GPIO_write(Board_BUSY, PIN_LOW);
            success = SPI_transfer(hSlave, &transaction2);
            GPIO_write(Board_BUSY, PIN_HIGH);

            if (!success)
            {
                System_printf("SMPTE slave reply failed!\n");
                System_flush();
            }
        }
        else if (opcode == SMPTE_REG_GENCTL)
        {
            /* ====================================================
             * SMPTE GENERATOR CONTROL REGISTER (RW)
             * ====================================================
             */

            if (ulRequest & SMPTE_F_READ)
            {
                /* READ SMPTE GENERATOR CONTROL REGISTER */

                switch(g_frame_rate)
                {
                case 24:
                    ulReply = SMPTE_GENCTL_FPS24;
                    break;

                case 25:
                    ulReply = SMPTE_GENCTL_FPS25;
                    break;

                case 30:
                    ulReply = (g_drop_frame) ? SMPTE_GENCTL_FPS30D : SMPTE_GENCTL_FPS30;
                    break;

                default:
                    ulReply = 0;
                    break;
                }

                if (g_running)
                    ulReply |= SMPTE_GENCTL_ENABLE;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&ulReply;
                transaction2.rxBuf = (Ptr)&ulDummy;

                /* Send the SPI transaction */
                GPIO_write(Board_BUSY, PIN_LOW);
                success = SPI_transfer(hSlave, &transaction2);
                GPIO_write(Board_BUSY, PIN_HIGH);

                if (!success)
                {
                    System_printf("SMPTE Slave reply failed!\n");
                    System_flush();
                }
            }
            else
            {
                /* WRITE SMPTE GENERATOR CONTROL REGISTER */

                /* Stop the generator if it's already running */
                SMPTE_Generator_Stop();

                /* Determine the frame rate requested */
                switch(SMPTE_GENCTL_FPS(ulRequest))
                {
                case SMPTE_GENCTL_FPS24:
                    g_frame_rate = 24;
                    g_drop_frame = false;
                    break;

                case SMPTE_GENCTL_FPS25:
                    g_frame_rate = 25;
                    g_drop_frame = false;
                    break;

                case SMPTE_GENCTL_FPS30:
                    g_frame_rate = 30;
                    g_drop_frame = false;
                    break;

                case SMPTE_GENCTL_FPS30D:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;

                default:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;
                }

                /* Start the SMPTE generator if enable flag set */
                if (ulRequest & SMPTE_GENCTL_ENABLE)
                {
                    /* Don't reset frame counts on resume */
                    if (!(ulRequest & SMPTE_GENCTL_RESUME))
                    {
                        SMPTE_Generator_Reset();
                    }

                    SMPTE_Generator_Start();
                }
                else
                {
                    SMPTE_Generator_Stop();
                }
            }
        }
        else if (opcode == SMPTE_REG_STAT)
        {
            /* ====================================================
             * SMPTE STATUS REGISTER
             * ====================================================
             */

        }
        else if (opcode == SMPTE_REG_DATA)
        {
            /* ====================================================
             * SMPTE STATUS REGISTER
             * ====================================================
             */

        }
    }
}

//*****************************************************************************
// Reset SMPTE generator start time
//*****************************************************************************

void SMPTE_Generator_Reset(void)
{
    /* Zero out the starting time struct */
    memset(&g_smpte_time, 0, sizeof(g_smpte_time));

    /* Set default time zone */
    strcpy(g_smpte_time.timezone, timezone);

    //g_smpte_time.years  = 0;        /* LTC date uses 2-digit year 00-99  */
    //g_smpte_time.months = 1;        /* valid months are 1..12            */
    //g_smpte_time.days   = 1;        /* day of month 1..31                */

    g_smpte_time.hours  = 0;        /* hour 0..23                        */
    g_smpte_time.mins   = 0;        /* minute 0..60                      */
    g_smpte_time.secs   = 0;        /* second 0..60                      */
    g_smpte_time.frame  = 0;        /* sub-second frame 0..(FPS - 1)     */

    /* Reset the frame buffer */
    ltc_frame_reset(&g_smpte_frame);

    /* Set the starting time members in the smpte frame */
    ltc_time_to_frame(&g_smpte_frame, &g_smpte_time, LTC_TV_525_60, 0);
}

//*****************************************************************************
// Start the SMPTE Generator
//*****************************************************************************

int SMPTE_Generator_Start(void)
{
    uint32_t clockrate;

    if (g_running)
        return -1;

    /* Setup timer interrupt 2x bit clocks:
     * 24 fps = 3840Hz
     * 25 fps = 4000Hz
     * 30 fps = 4800Hz
     */

    switch(g_frame_rate)
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
        g_frame_rate = 30;
        clockrate = 4800;
        break;
    }

    g_halfBit = 0;
    g_bitCount = 0;
    g_frameCount = 0;
    g_running = true;

    /* Pre-load the state of the first bit in the frame */
    g_bitState = FRAME_BITSTATE(((uint8_t*)&g_smpte_frame), g_bitCount);

    /* Enable the signal out relay to connect the SMPTE
     * output signal to channel 24 on the tape machine.
     */
    GPIO_write(Board_RELAY, Board_RELAY_ON);
    Task_sleep(50);

    /* Turn the LED on to indicate active */
    GPIO_write(Board_STAT_LED, Board_LED_ON);

    /* SMPTE output pin low initially */
    GPIO_write(Board_SMPTE_OUT, PIN_LOW);

    /* Set the TIMER1B load value to 1ms */
    TimerLoadSet(WTIMER1_BASE, TIMER_A, g_systemClock/clockrate);

    /* Configure the TIMER1B interrupt for timer timeout */
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

int SMPTE_Generator_Stop(void)
{
    if (!g_running)
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

    /* Relay and LED off */
    GPIO_write(Board_RELAY, Board_RELAY_OFF);
    GPIO_write(Board_STAT_LED, Board_LED_OFF);

    g_running = false;

    return 1;
}

//*****************************************************************************
// SMPTE Generator WTIMER Interrupt Handler
//*****************************************************************************

Void Timer1AIntHandler(UArg arg)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* Set drop frame bit if enabled */
    g_smpte_frame.dfbit = (g_drop_frame) ? 1 : 0;

    /* Flip half bit state indicator */
    g_halfBit = !g_halfBit;

    /* First half bit true, the flip at start of new bit */
    if (g_halfBit)
    {
        GPIO_toggle(Board_SMPTE_OUT);
    }
    else
    {
        /* Half bit false indicates second half of bit */
        if (g_bitState)
        {
            /* A high 1-bit changes half way */
            GPIO_toggle(Board_SMPTE_OUT);
        }

        /* Check if a full 80-bit frame has been transmitted */
        if (g_bitCount >= LTC_FRAME_BIT_COUNT)
        {
            /* If so, then increment the frame time */
            ltc_frame_increment(&g_smpte_frame, g_frame_rate, LTC_TV_625_50, 0);

            /* Increment frame counter */
            ++g_frameCount;

            /* Reset frame bit counter */
            g_bitCount = 0;
        }

        /* Pre-load the state of the next bit to go out */
        g_bitState = FRAME_BITSTATE(((uint8_t*)&g_smpte_frame), g_bitCount);

        /* Increment the frame bit counter */
        ++g_bitCount;

        /* Toggle the status LED on high bits */
        if (g_bitState)
            GPIO_toggle(Board_STAT_LED);
    }
}

Void Timer1BIntHandler(UArg arg)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
}

/* End-Of-File */
