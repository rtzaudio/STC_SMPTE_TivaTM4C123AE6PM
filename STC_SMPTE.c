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

/* Global Data Items */
SYSCFG g_cfg;
uint32_t g_systemClock;

extern SMPTETimecode g_txTime;
extern bool g_encoderEnabled;
extern bool g_decoderEnabled;

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

    /* Turn the LED on */
    GPIO_write(Board_STAT_LED, Board_LED_ON);

    /* WTIMER1 - SMPTE output generator
     * WTIMER0 - SMPTE input (64-bit timer option pins PC4 & PC5)
     */
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
    Hwi_plug(INT_WTIMER1A, WTimer1AIntHandler);
    Hwi_plug(INT_WTIMER1B, WTimer1BIntHandler);
    //Hwi_plug(INT_WTIMER0A, WTimer0AIntHandler);
    //Hwi_plug(INT_WTIMER0B, WTimer0BIntHandler);

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

    /* Read system parameters from EEPROM */
    //SysParamsRead(&g_cfg);
    InitSysDefaults(&g_cfg);

    /* Set busy pin high to indicate busy status */
    GPIO_write(Board_BUSY, PIN_HIGH);

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

    /* Startup the packet decoder task and interrupts */
    STC_SMPTE_initDecoder();

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

                switch(g_cfg.frame_rate)
                {
                case 24:
                    uReply = SMPTE_CTL_FPS24;
                    break;

                case 25:
                    uReply = SMPTE_CTL_FPS25;
                    break;

                case 30:
                    uReply = (g_cfg.drop_frame) ? SMPTE_CTL_FPS30D : SMPTE_CTL_FPS30;
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
                    g_cfg.frame_rate = 24;
                    g_cfg.drop_frame = false;
                    break;

                case SMPTE_CTL_FPS25:
                    g_cfg.frame_rate = 25;
                    g_cfg.drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30:
                    g_cfg.frame_rate = 30;
                    g_cfg.drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30D:
                    g_cfg.frame_rate = 30;
                    g_cfg.drop_frame = true;
                    break;

                default:
                    g_cfg.frame_rate = 30;
                    g_cfg.drop_frame = true;
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

                switch(g_cfg.frame_rate)
                {
                case 24:
                    uReply = SMPTE_CTL_FPS24;
                    break;

                case 25:
                    uReply = SMPTE_CTL_FPS25;
                    break;

                case 30:
                    uReply = (g_cfg.drop_frame) ? SMPTE_CTL_FPS30D : SMPTE_CTL_FPS30;
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
                    g_cfg.frame_rate = 24;
                    g_cfg.drop_frame = false;
                    break;

                case SMPTE_CTL_FPS25:
                    g_cfg.frame_rate = 25;
                    g_cfg.drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30:
                    g_cfg.frame_rate = 30;
                    g_cfg.drop_frame = false;
                    break;

                case SMPTE_CTL_FPS30D:
                    g_cfg.frame_rate = 30;
                    g_cfg.drop_frame = true;
                    break;

                default:
                    g_cfg.frame_rate = 30;
                    g_cfg.drop_frame = true;
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
// Set default runtime values
//*****************************************************************************

void InitSysDefaults(SYSCFG* p)
{
    /* zero out config structure */
    memset(p, 0, sizeof(SYSCFG));

    /* set default parameters */
    p->version    = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    p->build      = FIRMWARE_BUILD;
    p->debug      = 0;
    p->sysflags   = 0;
    p->frame_rate = 30;
    p->drop_frame = false;

    strcpy(p->timezone, "+0100");
}

//*****************************************************************************
// Write system parameters from our global settings buffer to EEPROM.
//
// Returns:  0 = Sucess
//          -1 = Error writing EEPROM data
//*****************************************************************************

int32_t SysParamsWrite(SYSCFG* sp)
{
    int32_t rc = 0;

    sp->version = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    sp->build   = FIRMWARE_BUILD;
    sp->magic   = MAGIC;

    rc = EEPROMProgram((uint32_t *)sp, 0, sizeof(SYSCFG));

    System_printf("Writing System Parameters (size=%d)\n", sizeof(SYSCFG));
    System_flush();

    return rc;
 }

//*****************************************************************************
// Read system parameters into our global settings buffer from EEPROM.
//
// Returns:  0 = Success
//          -1 = Error reading flash
//
//*****************************************************************************

int32_t SysParamsRead(SYSCFG* sp)
{
    InitSysDefaults(sp);

    EEPROMRead((uint32_t *)sp, 0, sizeof(SYSCFG));

    if (sp->magic != MAGIC)
    {
        System_printf("ERROR Reading System Parameters - Resetting Defaults...\n");
        System_flush();

        InitSysDefaults(sp);

        SysParamsWrite(sp);

        return -1;
    }

    if (sp->version != MAKEREV(FIRMWARE_VER, FIRMWARE_REV))
    {
        System_printf("WARNING New Firmware Version - Resetting Defaults...\n");
        System_flush();

        InitSysDefaults(sp);

        SysParamsWrite(sp);

        return -1;
    }

    if (sp->build < FIRMWARE_MIN_BUILD)
    {
        System_printf("WARNING New Firmware BUILD - Resetting Defaults...\n");
        System_flush();

        InitSysDefaults(sp);

        SysParamsWrite(sp);

        return -1;
    }

    System_printf("System Parameters Loaded (size=%d)\n", sizeof(SYSCFG));
    System_flush();

    return 0;
}

/* End-Of-File */
