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
#include "Utils.h"
#include "libltc\ltc.h"

#define DECODE_TIMER_SIZE       16

/* Constants and Macros */

/* Returns the state of a bit number in the frame buffer */
#define FRAME_BITSTATE(framebuf, bitnum)    ( (((framebuf[bitnum / 8]) >> (bitnum % 8)) & 0x01) )

/* Global Data Items */
SYSCFG g_cfg;
uint32_t g_systemClock;
const char timezone[6] = "+0100";

/* Global Config variables */
int  g_frame_rate = 30;
bool g_drop_frame = false;

/* SMPTE Encoder variables */
bool g_encoderEnabled = false;
uint8_t  g_txBitState = 0;
uint8_t  g_txHalfBit = 0;
uint32_t g_txBitCount = 0;
uint32_t g_txFrameCount = 0;
LTCFrame g_txFrame;
SMPTETimecode g_txTime;

/* SMPTE Decoder variables */

bool g_decoderEnabled = false;
LTCFrame g_rxFrame;
uint8_t* const g_code = (uint8_t*)&g_rxFrame;
SMPTETimecode g_rxTime;

volatile uint32_t g_ui32HighPeriod;
volatile uint32_t g_ui32HighStartCount;
volatile uint32_t g_ui32HighEndCount;

volatile int g_oneflg = 0;
volatile int g_bitCount = 0;
volatile int g_drop = 0;
volatile int g_fps = 0;

/*
 * Function Prototypes
 */

Int main();

int SMPTE_Encoder_Start();
int SMPTE_Encoder_Stop(void);
void SMPTE_Encoder_Reset(void);

int SMPTE_Decoder_Start();
int SMPTE_Decoder_Stop(void);
void SMPTE_Decoder_Reset(void);

Void SPI_SlaveTask(UArg a0, UArg a1);

Void WTimer1AIntHandler(UArg arg);
Void WTimer1BIntHandler(UArg arg);
Void WTimer0AIntHandler(UArg arg);
Void WTimer0BIntHandler(UArg arg);
Void Timer5AIntHandler(UArg arg);
Void Timer5BIntHandler(UArg arg);


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

//*****************************************************************************
// Main Program Entry Point
//*****************************************************************************

Int main()
{
    Error_Block eb;
    Task_Params taskParams;

    /* Get the system clock frequency */
    g_systemClock = SysCtlClockGet();

    /* Enable the floating point hardware unit */
    FPUEnable();

    /* Initialize peripherals for our board hardware */
    Board_initGeneral();
    Board_initGPIO();
    Board_initSPI();

    /* WTIMER1 - SMPTE output generator
     * WTIMER0 - SMPTE input (64-bit timer option pins PC4 & PC5)
     * TIMER5  - SMPTE input (16-bit timer option pins PG2 & PG3)
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    /* Map the timer interrupt handlers. We don't make sys/bios calls
     * from these interrupt handlers and there is no need to create a
     * context handler with stack swapping for these. These handlers
     * just update some globals variables and need to execute as
     * quickly and efficiently as possible.
     */
    Hwi_plug(INT_TIMER5A, Timer5AIntHandler);
    Hwi_plug(INT_TIMER5B, Timer5BIntHandler);
    Hwi_plug(INT_WTIMER1A, WTimer1AIntHandler);
    Hwi_plug(INT_WTIMER1B, WTimer1BIntHandler);
    Hwi_plug(INT_WTIMER0A, WTimer0AIntHandler);
    Hwi_plug(INT_WTIMER0B, WTimer0BIntHandler);

    /* Now start the main application button polling task */

    Error_init(&eb);

    Task_Params_init(&taskParams);

    taskParams.stackSize = 1500;
    taskParams.priority  = 10;

    if (Task_create(SPI_SlaveTask, &taskParams, &eb) == NULL)
        System_abort("SPI_SlaveTask!\n");

    BIOS_start();    /* does not return */

    return 0;
}

//*****************************************************************************
// This is the main SPI slave task that handles communications with the
// SPI master and allows control of the encoder and decoder services.
//*****************************************************************************

Void SPI_SlaveTask(UArg a0, UArg a1)
{
    bool success;
    uint16_t uRequest;
    uint16_t uReply;
    uint16_t uDummy;
    uint16_t uData;
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

    /* Reset the SMPTE encoder and decoder */
    SMPTE_Encoder_Reset();
    SMPTE_Decoder_Reset();

    SMPTE_Decoder_Start();

    /*
     * Enter the main SPI slave processing loop
     */

    for(;;)
    {
        uReply = uRequest = uDummy = 0;

        transaction1.count = 1;
        transaction1.txBuf = (Ptr)&uDummy;
        transaction1.rxBuf = (Ptr)&uRequest;

        /* Send the SPI transaction */

        GPIO_write(Board_BUSY, PIN_LOW);
        success = SPI_transfer(hSlave, &transaction1);
        GPIO_write(Board_BUSY, PIN_HIGH);

        if (!success)
        {
            /* Loop if error reading SPI! */
            System_printf("SPI slave rx failed\n");
            System_flush();
            continue;
        }

        /* SMPTE CONTROLLER SPI SLAVE REGISTERS
         *
         * All registers are 16-bits with the upper word containing the command
         * and flag bits. The lower 8-bits contains any associated data byte.
         *
         *   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
         *   | R | A | A | A | C | C | C | C | B | B | B | B | B | B | B | B |
         *   | W | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
         *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         *     |   |       |   |           |   |                           |
         *     |   +---+---+   +-----+-----+   +-------------+-------------+
         *     |       |             |                       |
         *    R/W     RSVD          REG                  DATA/FLAGS
         */

        /* lower 8-bits contats data or flags */
        uData = uRequest & SMPTE_DATA_MASK;

        switch((uRequest & SMPTE_REG_MASK) >> 8)
        {
        case SMPTE_REG_REVID:

            /* ====================================================
             * SMPTE CARD REV/ID REGISTER (RO)
             * ====================================================
             */

            uReply = SMPTE_REVID;

            /* Send the reply word back */
            transaction2.count = 1;
            transaction2.txBuf = (Ptr)&uReply;
            transaction2.rxBuf = (Ptr)&uDummy;

            /* Send the SPI transaction */
            GPIO_write(Board_BUSY, PIN_LOW);
            success = SPI_transfer(hSlave, &transaction2);
            GPIO_write(Board_BUSY, PIN_HIGH);
            break;

        case SMPTE_REG_ENCCTL:

            /* ====================================================
             * SMPTE GENERATOR CONTROL REGISTER (RW)
             * ====================================================
             */

            if (uRequest & SMPTE_F_READ)
            {
                /* READ SMPTE GENERATOR CONTROL REGISTER */

                switch(g_frame_rate)
                {
                case 24:
                    uReply = SMPTE_CTL_FPS24;
                    break;

                case 25:
                    uReply = SMPTE_CTL_FPS25;
                    break;

                case 30:
                    uReply = (g_drop_frame) ? SMPTE_CTL_FPS30D : SMPTE_CTL_FPS30;
                    break;

                default:
                    uReply = 0;
                    break;
                }

                if (g_encoderEnabled)
                    uReply |= SMPTE_ENCCTL_ENABLE;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&uReply;
                transaction2.rxBuf = (Ptr)&uDummy;

                /* Send the SPI transaction */
                GPIO_write(Board_BUSY, PIN_LOW);
                success = SPI_transfer(hSlave, &transaction2);
                GPIO_write(Board_BUSY, PIN_HIGH);
            }
            else
            {
                /* WRITE SMPTE GENERATOR CONTROL REGISTER */

                /* Stop the generator if it's already running */
                SMPTE_Encoder_Stop();

                /* Determine the frame rate requested */
                switch(SMPTE_ENCCTL_FPS(uRequest))
                {
                case SMPTE_CTL_FPS24:
                    g_frame_rate = 24;
                    g_drop_frame = false;
                    break;

                case SMPTE_CTL_FPS25:
                    g_frame_rate = 25;
                    g_drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30:
                    g_frame_rate = 30;
                    g_drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30D:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;

                default:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;
                }

                /* Start the SMPTE generator if enable flag set */
                if (uRequest & SMPTE_ENCCTL_ENABLE)
                {
                    /* Reset frame counts on requested */
                    if (uRequest & SMPTE_ENCCTL_RESET)
                        SMPTE_Encoder_Reset();

                    SMPTE_Encoder_Start();
                }
            }
            break;

        case SMPTE_REG_DECCTL:

            /* ====================================================
             * SMPTE GENERATOR CONTROL REGISTER (RW)
             * ====================================================
             */

            if (uRequest & SMPTE_F_READ)
            {
                /* READ SMPTE DECODER CONTROL REGISTER */

                switch(g_frame_rate)
                {
                case 24:
                    uReply = SMPTE_CTL_FPS24;
                    break;

                case 25:
                    uReply = SMPTE_CTL_FPS25;
                    break;

                case 30:
                    uReply = (g_drop_frame) ? SMPTE_CTL_FPS30D : SMPTE_CTL_FPS30;
                    break;

                default:
                    uReply = 0;
                    break;
                }

                if (g_decoderEnabled)
                    uReply |= SMPTE_DECCTL_ENABLE;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&uReply;
                transaction2.rxBuf = (Ptr)&uDummy;

                /* Send the SPI transaction */
                GPIO_write(Board_BUSY, PIN_LOW);
                success = SPI_transfer(hSlave, &transaction2);
                GPIO_write(Board_BUSY, PIN_HIGH);
            }
            else
            {
                /* WRITE SMPTE DECODER CONTROL REGISTER */

                /* Stop the generator if it's already running */
                SMPTE_Decoder_Stop();

                /* Determine the frame rate requested */
                switch(SMPTE_DECCTL_FPS(uRequest))
                {
                case SMPTE_CTL_FPS24:
                    g_frame_rate = 24;
                    g_drop_frame = false;
                    break;

                case SMPTE_CTL_FPS25:
                    g_frame_rate = 25;
                    g_drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30:
                    g_frame_rate = 30;
                    g_drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30D:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;

                default:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;
                }

                /* Start the SMPTE generator if enable flag set */
                if (uRequest & SMPTE_DECCTL_ENABLE)
                {
                    /* Reset frame counts on requested */
                    if (uRequest & SMPTE_DECCTL_RESET)
                        SMPTE_Decoder_Reset();

                    SMPTE_Decoder_Start();
                }
            }
            break;

        case SMPTE_REG_STAT:

            /* ====================================================
             * SMPTE STATUS REGISTER
             * ====================================================
             */

            break;

        case SMPTE_REG_DATA:

            /* ====================================================
             * SMPTE DATA REGISTER
             * ====================================================
             */

            break;

        case SMPTE_REG_HOUR:

            /* ====================================================
             * SMPTE (HOUR) REGISTER (0-23)
             * ====================================================
             */

            if (uRequest & SMPTE_F_READ)
            {
                uReply = (uint16_t)g_txTime.hours;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&uReply;
                transaction2.rxBuf = (Ptr)&uDummy;

                /* Send the SPI transaction */
                GPIO_write(Board_BUSY, PIN_LOW);
                success = SPI_transfer(hSlave, &transaction2);
                GPIO_write(Board_BUSY, PIN_HIGH);
            }
            else
            {
                if (uData < 24)
                {
                    g_txTime.hours = (uint8_t)uData;
                }
            }
            break;

        case SMPTE_REG_MINS:

            /* ====================================================
             * SMPTE (MINS) REGISTER (0-59)
             * ====================================================
             */

            if (uRequest & SMPTE_F_READ)
            {
                uReply = (uint16_t)g_txTime.mins;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&uReply;
                transaction2.rxBuf = (Ptr)&uDummy;

                /* Send the SPI transaction */
                GPIO_write(Board_BUSY, PIN_LOW);
                success = SPI_transfer(hSlave, &transaction2);
                GPIO_write(Board_BUSY, PIN_HIGH);
            }
            else
            {
                if (uData <= 60)
                {
                    g_txTime.mins = (uint8_t)uData;
                }
            }
            break;

        case SMPTE_REG_SECS:

            /* ====================================================
             * SMPTE (SECS) REGISTER (0-59)
             * ====================================================
             */

            if (uRequest & SMPTE_F_READ)
            {
                uReply = (uint16_t)g_txTime.secs;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&uReply;
                transaction2.rxBuf = (Ptr)&uDummy;

                /* Send the SPI transaction */
                GPIO_write(Board_BUSY, PIN_LOW);
                success = SPI_transfer(hSlave, &transaction2);
                GPIO_write(Board_BUSY, PIN_HIGH);
            }
            else
            {
                if (uData <= 60)
                {
                    g_txTime.secs = (uint8_t)uData;
                }
            }
            break;

        case SMPTE_REG_FRAME:

            /* ====================================================
             * SMPTE (FRAME) NUMBER REGISTER (0-29)
             * ====================================================
             */

            if (uRequest & SMPTE_F_READ)
            {
                uReply = (uint16_t)g_txTime.frame;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&uReply;
                transaction2.rxBuf = (Ptr)&uDummy;

                /* Send the SPI transaction */
                GPIO_write(Board_BUSY, PIN_LOW);
                success = SPI_transfer(hSlave, &transaction2);
                GPIO_write(Board_BUSY, PIN_HIGH);
            }
            else
            {
                if (uData < 30)
                {
                    g_txTime.frame = (uint8_t)uData;
                }
            }
            break;

        default:
            /* Unknown command */
            break;
        }
    }
}

//*****************************************************************************
//********************** SMPTE ENCODER SUPPORT ********************************
//*****************************************************************************

void SMPTE_Encoder_Reset(void)
{
    /* Zero out the starting time struct */
    memset(&g_txTime, 0, sizeof(g_txTime));

    /* Set default time zone */
    strcpy(g_txTime.timezone, timezone);

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

    /* Set the starting time members in the SMPTE tx frame buffer */
    ltc_time_to_frame(&g_txFrame, &g_txTime, LTC_TV_525_60, 0);

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

    g_txFrameCount = 0;
    g_txBitCount = 0;
    g_txHalfBit = 0;
    g_encoderEnabled = true;

    /* Pre-load the state of the first bit in the frame */
    g_txBitState = FRAME_BITSTATE(((uint8_t*)&g_txFrame), g_txBitCount);

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

    /* Relay and LED off */
    GPIO_write(Board_RELAY, Board_RELAY_OFF);
    GPIO_write(Board_STAT_LED, Board_LED_OFF);

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
    g_txFrame.dfbit = (g_drop_frame) ? 1 : 0;

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
            ltc_frame_increment(&g_txFrame, g_frame_rate, LTC_TV_625_50, 0);

            /* Increment frame counter */
            ++g_txFrameCount;

            /* Reset frame bit counter */
            g_txBitCount = 0;
        }

        /* Pre-load the state of the next bit to go out */
        g_txBitState = FRAME_BITSTATE(((uint8_t*)&g_txFrame), g_txBitCount);

        /* Increment the frame bit counter */
        ++g_txBitCount;

        /* Toggle the status LED on high bits */
        //if (g_txBitState)
        //    GPIO_toggle(Board_STAT_LED);
    }
}

Void WTimer1BIntHandler(UArg arg)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
}

//*****************************************************************************
//********************** SMPTE DECODER SUPPORT ********************************
//*****************************************************************************

Void SMPTE_Decoder_Reset(void)
{
    /* Zero out the starting time struct */
    memset(&g_rxTime, 0, sizeof(g_rxTime));

    /* Set default time zone */
    strcpy(g_rxTime.timezone, timezone);

    g_rxTime.years  = 0;        /* LTC date uses 2-digit year 00-99  */
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
    /* Make sure the decoder interrupt isn't enabled */
    SMPTE_Decoder_Stop();

    /* Zero out the starting time struct */
    memset(&g_rxTime, 0, sizeof(g_rxTime));

    /* Set default time zone */
    strcpy(g_rxTime.timezone, timezone);

    /* Reset global variables */
    g_oneflg = g_bitCount = g_drop = g_fps = 0;

    /* Reset the frame buffer */
    ltc_frame_reset(&g_rxFrame);

#if (DECODE_TIMER_SIZE == 32)

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

    // To use the timer in Edge Time mode, it must be preloaded with initial
    // values.  If the prescaler is used, then it must be preloaded as well.
    // Since we want to use all 24-bits for both timers it will be loaded with
    // the maximum of 0xFFFF for the 16-bit wide split timers, and 0xFF to add
    // the additional 8-bits to the split timers with the prescaler.

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

#else

    /* Configure the GPIO PG2 and PG3 for the Timer peripheral */
    GPIOPinTypeTimer(GPIO_PORTG_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    /* Configure the GPIO to be CCP pins for the Timer peripheral */
    GPIOPinConfigure(GPIO_PG2_T5CCP0);
    GPIOPinConfigure(GPIO_PG3_T5CCP1);

    /* Initialize Timers A and B to both run as periodic up-count edge capture
     * This will split the 64-bit timer into two 32-bit timers.
     */
    TimerConfigure(TIMER5_BASE, (TIMER_CFG_SPLIT_PAIR |
                                 TIMER_CFG_A_PERIODIC | TIMER_CFG_A_CAP_TIME_UP |
                                 TIMER_CFG_B_PERIODIC | TIMER_CFG_B_CAP_TIME_UP));

    /* To use the timer in Edge Time mode, it must be preloaded with initial
     * values.  If the prescaler is used, then it must be preloaded as well.
     * Since we want to use all 24-bits for both timers it will be loaded with
     * the maximum of 0xFFFF for the 16-bit wide split timers, and 0xFF to add
     * the additional 8-bits to the split timers with the prescaler.
     */
    TimerLoadSet(TIMER5_BASE, TIMER_BOTH, 0xFFFF);
    TimerPrescaleSet(TIMER5_BASE, TIMER_BOTH, 0xFF);

    /* Configure Timer A to trigger on a Positive Edge and configure
     * Timer B to trigger on a Negative Edge.
     */
    TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMER5_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

    /* Clear the interrupt status flag.  This is done to make sure the
     * interrupt flag is cleared before we enable it.
     */
    TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    /* Enable the Timer A and B interrupts for Capture Events */
    TimerIntEnable(TIMER5_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    /* Enable the interrupts for Timer A and Timer B on the processor (NVIC) */
    IntEnable(INT_TIMER5A);
    IntEnable(INT_TIMER5B);

    /* Enable both Timer A and Timer B to begin the application */
    TimerEnable(TIMER5_BASE, TIMER_BOTH);

#endif
    return 0;
}

//*****************************************************************************
// Stop the SMPTE Decoder
//*****************************************************************************

int SMPTE_Decoder_Stop(void)
{
#if (DECODE_TIMER_SIZE == 32)
    /* Disable both Timer A and Timer B */
    TimerDisable(WTIMER0_BASE, TIMER_BOTH);

    IntDisable(INT_WTIMER0A);
    IntDisable(INT_WTIMER0B);

    /* Disable the Timer A and B interrupts for Capture Events */
    TimerIntDisable(WTIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    /* Clear any interrupts pending */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
#else
    /* Disable both Timer A and Timer B */
    TimerDisable(TIMER5_BASE, TIMER_BOTH);

    IntDisable(INT_TIMER5A);
    IntDisable(INT_TIMER5B);

    /* Disable the Timer A and B interrupts for Capture Events */
    TimerIntDisable(TIMER5_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    /* Clear any interrupts pending */
    TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
#endif
    return 0;
}

//*****************************************************************************
// SMPTE Input Edge Timing Interrupts (32-BIT WIDE TIMER IMPLEMENTATION)
//*****************************************************************************

/* Rising Edge Interrupt */
Void WTimer0AIntHandler(UArg arg)
{
    TimerLoadSet(WTIMER0_BASE, TIMER_A, 0xFFFFFFFF);

    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);

    /* Store the end time.  In Edge Time Mode, the prescaler is used for the
     * the most significant bits.  Therefore, it must be shifted by 16 before
     * being added onto the final value.
     */
    g_ui32HighStartCount = (TimerValueGet(WTIMER0_BASE, TIMER_A));  // + (TimerPrescaleGet(WTIMER0_BASE, TIMER_A) << 32);
}

/* Falling Edge Interrupt */
Void WTimer0BIntHandler(UArg arg)
{
    uint8_t* code;
    uint32_t i, b, t;

    TimerLoadSet(WTIMER0_BASE, TIMER_B, 0xFFFFFFFF);

    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);

    /* Store the end time.  In Edge Time Mode, the prescaler is used for the
     * the most significant bits.  Therefore, it must be shifted by 16 before
     * being added onto the final value.
     */
    g_ui32HighEndCount = (TimerValueGet(WTIMER0_BASE, TIMER_B));    // + (TimerPrescaleGet(WTIMER0_BASE, TIMER_B) << 32);

    /* Simple check to avoid overflow cases.  The End Count is the
     * second measurement taken and therefore should never be smaller
     * than the Start Count unless the timer has overflowed.  If that
     * occurs, then add 2^16-1 to the End Count before subtraction.
     */
    if (g_ui32HighEndCount > g_ui32HighStartCount)
    {
        g_ui32HighPeriod = g_ui32HighEndCount - g_ui32HighStartCount;
    }
    else
    {
        g_ui32HighPeriod = (uint32_t)(((uint64_t)g_ui32HighEndCount + 16777215) - (uint64_t)g_ui32HighStartCount);

        g_ui32HighPeriod = g_ui32HighStartCount - g_ui32HighEndCount;
    }

    /* Now look at the period and decide if it's a one or zero */
    t  = g_ui32HighPeriod;

    /* We've interrupted on the falling edge and can now
     * calculate the pulse width time in microseconds by
     * dividing t period count by 80.
     *
     *  pulse width: 416.7us(30fps)
     *  0bit x 30frame/s --> 416.7us/bit
     */

    if (t >= ONE_TIME_MIN && t < ONE_TIME_MAX)
    {
        if (g_oneflg == 0)
        {
            g_oneflg = 1;
            return;
        }
        else
        {
            g_oneflg = 0;
            b = 1;
        }
    }
    else if (t >= ZERO_TIME_MIN && t < ZERO_TIME_MAX)
    {
        g_oneflg = 0;
        b = 0;

        if (t >= FPS24_REF - 10 && t <= FPS24_REF + 10)
        {
            g_fps = 0;
        }
        else if (t >= FPS25_REF - 10 && t <= FPS25_REF + 10)
        {
            g_fps = 1;
        }
        else if (t >= FPS30_REF - 10 && t <= FPS30_REF + 10)
        {
            g_fps = g_drop ? 2 : 3; // 29.97 / 30
        }
    }
    else
    {
        g_oneflg = 0;
        g_bitCount = 0;
        return;
    }

    /* Shift the bit into the 80-bit receive buffer */

    code = (uint8_t*)&g_rxFrame;

    //code[0] = (code[0] >> 1) | ((code[0+1] & 1) ? 0x80 : 0);
    //code[1] = (code[1] >> 1) | ((code[1+1] & 1) ? 0x80 : 0);
    //code[2] = (code[2] >> 1) | ((code[2+1] & 1) ? 0x80 : 0);
    //code[3] = (code[3] >> 1) | ((code[3+1] & 1) ? 0x80 : 0);
    //code[4] = (code[4] >> 1) | ((code[4+1] & 1) ? 0x80 : 0);
    //code[5] = (code[5] >> 1) | ((code[5+1] & 1) ? 0x80 : 0);
    //code[6] = (code[6] >> 1) | ((code[6+1] & 1) ? 0x80 : 0);
    //code[7] = (code[7] >> 1) | ((code[7+1] & 1) ? 0x80 : 0);
    //code[8] = (code[8] >> 1) | ((code[8+1] & 1) ? 0x80 : 0);
    //code[9] = (code[9] >> 1) | (b ? 0x80 : 0);

    for (i=0; i < 9; i ++)
        code[i] = (code[i] >> 1) | ((code[i+1] & 1) ? 0x80 : 0);

    code[9] = (code[9] >> 1) | (b ? 0x80 : 0);

    g_bitCount++;

    /* Must see the SMPTE sync word at the end of frame to consider it a
     * valid packet. If so, we parse out the frame members into our buffer.
     */
    if ((code[8] == 0xFC) && (code[9] == 0xBF) && (g_bitCount >= 80))
    {
        /* Parse the buffer and get time members in the SMPTE frame */
        //ltc_frame_to_time(&g_rxTime, &g_rxFrame, 0);

        g_rxTime.hours = g_rxFrame.hours_units + g_rxFrame.hours_tens * 10;
        g_rxTime.mins  = g_rxFrame.mins_units  + g_rxFrame.mins_tens * 10;
        g_rxTime.secs  = g_rxFrame.secs_units  + g_rxFrame.secs_tens * 10;
        g_rxTime.frame = g_rxFrame.frame_units + g_rxFrame.frame_tens * 10;

        /* Toggle the LED on each packet received */
        GPIO_toggle(Board_STAT_LED);

        /* Reset the pulse bit counter */
        g_bitCount = 0;
    }
}

//*****************************************************************************
// SMPTE Input Edge Timing Interrupts (16-BIT PRE-SCALED IMPLEMENTATION)
//*****************************************************************************

/* Rising Edge Interrupt */
Void Timer5AIntHandler(UArg arg)
{
    /* Clear the timer interrupt */
    TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT);

    /* Store the end time.  In Edge Time Mode, the prescaler is used for the
     * the most significant bits.  Therefore, it must be shifted by 16 before
     * being added onto the final value.
     */
    g_ui32HighStartCount = (TimerValueGet(TIMER5_BASE, TIMER_A)) +
                           (TimerPrescaleGet(TIMER5_BASE, TIMER_A) << 16);
}

/* Falling Edge Interrupt */
Void Timer5BIntHandler(UArg arg)
{
    uint8_t* code;
    uint32_t i, b, t;

    /* Clear the timer interrupt */
    TimerIntClear(TIMER5_BASE, TIMER_CAPB_EVENT);

    /* Store the end time.  In Edge Time Mode, the prescaler is used for the
     * the most significant bits.  Therefore, it must be shifted by 16 before
     * being added onto the final value.
     */

    g_ui32HighEndCount = (TimerValueGet(TIMER5_BASE, TIMER_B)) +
                         (TimerPrescaleGet(TIMER5_BASE, TIMER_B) << 16);

    /* Simple check to avoid overflow cases.  The End Count is the
     * second measurement taken and therefore should never be smaller
     * than the Start Count unless the timer has overflowed.  If that
     * occurs, then add 2^16-1 to the End Count before subtraction.
     */

    if (g_ui32HighEndCount > g_ui32HighStartCount)
    {
        g_ui32HighPeriod = g_ui32HighEndCount - g_ui32HighStartCount;
    }
    else
    {
        g_ui32HighPeriod = (g_ui32HighEndCount + 16777215) - g_ui32HighStartCount;
    }

    /* Now look at the high pulse period and determine if it's a one or zero */
    t  = g_ui32HighPeriod;

    /* We've interrupted on the falling edge and can now
     * calculate the pulse width time in microseconds by
     * dividing t period count by 80.
     *
     *  pulse width: 416.7us(30fps)
     *  0bit x 30frame/s --> 416.7us/bit
     */

    //t /= 80;

    if (t >= ONE_TIME_MIN && t < ONE_TIME_MAX)
    {
        if (g_oneflg == 0)
        {
            g_oneflg = 1;
            return;
        }
        else
        {
            g_oneflg = 0;
            b = 1;
        }
    }
    else if (t >= ZERO_TIME_MIN && t < ZERO_TIME_MAX)
    {
        g_oneflg = 0;
        b = 0;

        if (t >= FPS24_REF - 10 && t <= FPS24_REF + 10)
        {
            g_fps = 0;
        }
        else if (t >= FPS25_REF - 10 && t <= FPS25_REF + 10)
        {
            g_fps = 1;
        }
        else if (t >= FPS30_REF - 10 && t <= FPS30_REF + 10)
        {
            g_fps = g_drop ? 2 : 3; // 29.97 / 30
        }
    }
    else
    {
        /* No bit found in valid frame length time */
        g_oneflg = 0;
        g_bitCount = 0;
        return;
    }

    /* Shift the bit into the 80-bit receive buffer */

    code = (uint8_t*)&g_rxFrame;

    for (i=0; i < 9; i ++)
        code[i] = (code[i] >> 1) | ((code[i+1] & 1) ? 0x80 : 0);

    code[9] = (code[9] >> 1) | (b ? 0x80 : 0);

    g_bitCount++;

    /* Must see the SMPTE sync word at the end of frame to consider it a
     * valid packet. If so, we parse out the frame members into our buffer.
     */
    if ((code[8] == 0xFC) && (code[9] == 0xBF) && (g_bitCount >= 80))
    {
        /* Parse the buffer and get time members in the SMPTE frame */
        //ltc_frame_to_time(&g_rxTime, &g_rxFrame, 0);

        g_rxTime.hours = g_rxFrame.hours_units + g_rxFrame.hours_tens * 10;
        g_rxTime.mins  = g_rxFrame.mins_units  + g_rxFrame.mins_tens * 10;
        g_rxTime.secs  = g_rxFrame.secs_units  + g_rxFrame.secs_tens * 10;
        g_rxTime.frame = g_rxFrame.frame_units + g_rxFrame.frame_tens * 10;

        /* Toggle the LED on each packet received */
        GPIO_toggle(Board_STAT_LED);

        /* Reset the pulse bit counter */
        g_bitCount = 0;
    }
}

/* End-Of-File */
