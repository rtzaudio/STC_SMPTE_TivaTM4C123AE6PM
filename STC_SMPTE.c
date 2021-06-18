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

/* Constants and Macros */

/* pulse width: 416.7us(30fps)
 * 80bit x 30frame/s --> 416.7us/bit
 */

//#define ONE_TIME_MIN    180
//#define ONE_TIME_MAX    280
//#define ZERO_TIME_MIN   390
//#define ZERO_TIME_MAX   540

#define FPS24_REF       521
#define FPS25_REF       500
#define FPS30_REF       417

/* Returns the state of a bit number in the frame buffer */
#define FRAME_BITSTATE(framebuf, bitnum)    ( (((framebuf[bitnum / 8]) >> (bitnum % 8)) & 0x01) )

/* Global Data Items */
SYSCFG g_cfg;
uint32_t g_systemClock;
const char timezone[6] = "+0100";

/* Global Config variables */
static int  g_frame_rate = 30;
static bool g_drop_frame = false;

/* SMPTE Encoder variables */
static bool g_txRunning = false;
static uint8_t  g_txBitState = 0;
static uint8_t  g_txHalfBit = 0;
static uint32_t g_txBitCount = 0;
static uint32_t g_txFrameCount = 0;
static LTCFrame g_txFrame;
static SMPTETimecode g_txTime;

/* SMPTE Decoder variables */
static LTCFrame g_rxFrame;
static SMPTETimecode g_rxTime;

static uint8_t* code = (uint8_t*)&g_rxFrame;
static bool g_firsttime = true;



/* Static Function Prototypes */
Int main();
int SMPTE_Encoder_Start();
int SMPTE_Encoder_Stop(void);
int SMPTE_Decoder_Start();
int SMPTE_Decoder_Stop(void);
void SMPTE_Encoder_Reset(void);
Void SPI_SlaveTask(UArg a0, UArg a1);
Void Timer1AIntHandler(UArg arg);
Void Timer1BIntHandler(UArg arg);
Void Timer0AIntHandler(UArg arg);
Void Timer0BIntHandler(UArg arg);
void gpioStreamInHwi(unsigned int index);

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

    /* Map the timer interrupt handlers. We don't make sys/bios calls
     * from these interrupt handlers and there is no need to create a
     * context handler with stack swapping for these. These handlers
     * just update some globals variables and need to execute as
     * quickly and efficiently as possible.
     */
    Hwi_plug(INT_WTIMER1A, Timer1AIntHandler);
    Hwi_plug(INT_WTIMER1B, Timer1BIntHandler);

    //Hwi_plug(INT_WTIMER0A, Timer0AIntHandler);
    //Hwi_plug(INT_WTIMER0B, Timer0BIntHandler);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    /* Setup callback Hwi handler the SMPTE pulse stream input */
    GPIO_setCallback(Board_SMPTE_IN, gpioStreamInHwi);

    /* Enable the SMPTE input stream interrupts */
    GPIO_enableInt(Board_SMPTE_IN);

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

    /* Configure TIMER1B as a 16-bit periodic timer */
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_PERIODIC);

    /* Reset the SMPTE frame buffer to zeros */
    SMPTE_Encoder_Reset();

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
        }
        else if (opcode == SMPTE_REG_ENCCTL)
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
                    ulReply = SMPTE_ENCCTL_FPS24;
                    break;
                case 25:
                    ulReply = SMPTE_ENCCTL_FPS25;
                    break;
                case 30:
                    ulReply = (g_drop_frame) ? SMPTE_ENCCTL_FPS30D : SMPTE_ENCCTL_FPS30;
                    break;
                default:
                    ulReply = 0;
                    break;
                }

                if (g_txRunning)
                    ulReply |= SMPTE_ENCCTL_ENABLE;

                /* Send the reply word back */
                transaction2.count = 1;
                transaction2.txBuf = (Ptr)&ulReply;
                transaction2.rxBuf = (Ptr)&ulDummy;

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
                switch(SMPTE_ENCCTL_FPS(ulRequest))
                {
                case SMPTE_ENCCTL_FPS24:
                    g_frame_rate = 24;
                    g_drop_frame = false;
                    break;
                case SMPTE_ENCCTL_FPS25:
                    g_frame_rate = 25;
                    g_drop_frame = false;
                    break;
                case SMPTE_ENCCTL_FPS30:
                    g_frame_rate = 30;
                    g_drop_frame = false;
                    break;
                case SMPTE_ENCCTL_FPS30D:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;
                default:
                    g_frame_rate = 30;
                    g_drop_frame = true;
                    break;
                }

                /* Start the SMPTE generator if enable flag set */
                if (ulRequest & SMPTE_ENCCTL_ENABLE)
                {
                    /* Don't reset frame counts on resume */
                    if (!(ulRequest & SMPTE_ENCCTL_RESUME))
                    {
                        SMPTE_Encoder_Reset();
                    }

                    SMPTE_Decoder_Start();
                    SMPTE_Encoder_Start();
                }
                else
                {
                    SMPTE_Encoder_Stop();
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

    /* Set the starting time members in the smpte frame */
    ltc_time_to_frame(&g_txFrame, &g_txTime, LTC_TV_525_60, 0);
}

//*****************************************************************************
// Start the SMPTE Generator
//*****************************************************************************

int SMPTE_Encoder_Start(void)
{
    uint32_t clockrate;

    if (g_txRunning)
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

    g_txFrameCount = 0;
    g_txBitCount = 0;
    g_txHalfBit = 0;
    g_txRunning = true;

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
    if (!g_txRunning)
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

    g_txRunning = false;

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

Void Timer1BIntHandler(UArg arg)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
}

//*****************************************************************************
// SMPTE Decoder Hardware Receive Interrupt Handler
//*****************************************************************************

int SMPTE_Decoder_Start(void)
{
    g_firsttime = true;

    /* Zero out the starting time struct */
    memset(&g_rxTime, 0, sizeof(g_rxTime));

    /* Reset the frame buffer */
    ltc_frame_reset(&g_rxFrame);

    /* Enable the SMPTE input stream interrupts */
    GPIO_enableInt(Board_SMPTE_IN);

    /* Configure the GPIO to be CCP pins for the timer peripheral */
    //GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    //GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);

    /* Initialize Timer A to run as periodic up-count edge capture */
    //TimerConfigure(WTIMER0_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_A_CAP_TIME_UP);
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_A_PERIODIC_UP);

    /* To use the timer in Edge Time mode, it must be preloaded with initial
     * values.  If the prescaler is used, then it must be preloaded as well.
     * Since we want to use all 24-bits for the timer it will be loaded with
     * the maximum of 0xFFFF for the 16-bit wide split timers, and 0xFF to add
     * the additional 8-bits to the split timer with the prescaler.
     */
    TimerLoadSet(WTIMER0_BASE, TIMER_A, 0);
    //TimerPrescaleSet(WTIMER0_BASE, TIMER_A, 0xFF);

    // Configure Timer A to trigger on both edges
    //TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

    /* Clear the interrupt status flag.  This is done to make sure the
     * interrupt flag is cleared before we enable it.
     */
    //TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);

    /* Enable the Timer A and B interrupts for Capture Events */
    //TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT);

    /* Enable the interrupts for Timer A and Timer B on the processor (NVIC) */
    //IntEnable(INT_TIMER0A);

    /* Enable both Timer A and Timer B to begin the application */
    TimerEnable(WTIMER0_BASE, TIMER_A);

    return 0;
}

int SMPTE_Decoder_Stop(void)
{
    /* Enable the SMPTE input stream interrupts */
    GPIO_disableInt(Board_SMPTE_IN);

    /* Enable both Timer A and Timer B to begin the application */
    TimerDisable(WTIMER0_BASE, TIMER_A);

    /* Enable the interrupts for Timer A on the processor (NVIC) */
    //IntDisable(INT_TIMER0A);

    return 0;
}

//*****************************************************************************
// SMPTE Input Pin Toggle Interrupt Handler
//*****************************************************************************

//static int mode;
static int oneflg = 0;
static int count = 0;
static int drop = 0;
static int fps = 0;

#if 0
Void Timer0AIntHandler(UArg arg)
{
    uint32_t i, b, t;

    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);

    /* In Edge Time Mode, the prescaler is used for the most significant bits.
     * Therefore, it must be shifted by 16 before being added onto the final value.
     */
    t = TimerValueGet(WTIMER0_BASE, TIMER_A);

    /* Determine the pulse width time in uSec */
    //t /= 80;

    if (t >= ONE_TIME_MIN && t < ONE_TIME_MAX)
    {
        if (oneflg == 0)
        {
            oneflg = 1;
            return;
        }
        else
        {
            oneflg = 0;
            b = 1;
        }
    }
    else if (t >= ZERO_TIME_MIN && t < ZERO_TIME_MAX)
    {
        oneflg = 0;
        b = 0;

        if (t >= FPS24_REF - 10 && t <= FPS24_REF + 10)
        {
            fps = 0;
        }
        else if (t >= FPS25_REF - 10 && t <= FPS25_REF + 10)
        {
            fps = 1;
        }
        else if (t >= FPS30_REF - 10 && t <= FPS30_REF + 10)
        {
            fps = drop ? 2 : 3; // 29.97 / 30
        }
    }
    else
    {
        oneflg = 0;
        count = 0;
        return;
    }

    for (i=0; i < 9; i ++)
    {
        code[i] = (code[i] >> 1) | ((code[i + 1] & 1) ? 0x80 : 0);
    }

    code[9] = (code[9] >> 1) | (b ? 0x80 : 0);

    count++;

    if ((code[8] == 0xFC) && (code[9] == 0xBF) && (count >= 80))
    {
        //parse_code();

        /* Get time members in the smpte frame */
        ltc_frame_to_time(&g_rxTime, &g_rxFrame, 0);

        count = 0;
    }
}

Void Timer0BIntHandler(UArg arg)
{
    uint32_t status = TimerIntStatus(WTIMER0_BASE, false);

    /* Clear the timer interrupt flag */
    TimerIntClear(WTIMER0_BASE, status);
}
#endif


void gpioStreamInHwi(unsigned int index)
{
    uint32_t i;
    uint32_t b;
    uint32_t t, d;
    uint32_t bitState;
    static uint32_t tlast = 0;

    /* Clear the interrupt source */
    GPIO_clearInt(Board_SMPTE_IN);

    /* First edge, can't time yet */
    if (g_firsttime)
    {
        tlast = TimerValueGet(WTIMER0_BASE, TIMER_A);
        g_firsttime = false;
        return;
    }

    /* GPIO pin interrupt occurred, read bit state */
    bitState = GPIO_read(Board_SMPTE_IN);

    if (bitState)
        GPIO_write(Board_STAT_LED, PIN_HIGH);
    else
        GPIO_write(Board_STAT_LED, PIN_LOW);

    /* Determine the pulse width time in uSec */
    d = TimerValueGet(WTIMER0_BASE, TIMER_A);
    t = d - tlast;
    tlast = d;

    /* Resets timeout timer */
    HWREG(WTIMER0_BASE + TIMER_O_TAV) = 0;

    if (t >= ONE_TIME_MIN && t < ONE_TIME_MAX)
    {
        if (oneflg == 0)
        {
            oneflg = 1;
            return;
        }
        else
        {
            oneflg = 0;
            b = 1;
        }
    }
    else if (t >= ZERO_TIME_MIN && t < ZERO_TIME_MAX)
    {
        oneflg = 0;
        b = 0;

        if (t >= FPS24_REF - 10 && t <= FPS24_REF + 10)
        {
            fps = 0;
        }
        else if (t >= FPS25_REF - 10 && t <= FPS25_REF + 10)
        {
            fps = 1;
        }
        else if (t >= FPS30_REF - 10 && t <= FPS30_REF + 10)
        {
            fps = drop ? 2 : 3; // 29.97 / 30
        }
    }
    else
    {
        oneflg = 0;
        count = 0;
        return;
    }

    for (i=0; i < 9; i ++)
    {
        code[i] = (code[i] >> 1) | ((code[i + 1] & 1) ? 0x80 : 0);
    }

    code[9] = (code[9] >> 1) | (b ? 0x80 : 0);

    count++;

    if ((code[8] == 0xFC) && (code[9] == 0xBF) && (count >= 80))
    {
        //parse_code();

        /* Get time members in the smpte frame */
        ltc_frame_to_time(&g_rxTime, &g_rxFrame, 0);

        count = 0;
    }
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
