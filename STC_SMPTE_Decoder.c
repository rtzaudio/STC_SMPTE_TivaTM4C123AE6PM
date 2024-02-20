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

/*** Data Types and Constants ***/

#define DEBUG_SMPTE     0

/* SMPTE 80-bit frame buffer */
typedef struct ltcframe_t {
    uint64_t data;              /* 64-bits data */
    uint16_t sync;              /* 16-bits sync */
} ltcframe_t;                   /* 80-bit frame */

/* Access as struct members or raw bit form */
typedef union LTCFrameWord_t {
    LTCFrame    ltc;            /* members form */
    ltcframe_t  raw;            /* raw bit form */
} LTCFrameWord;

/*** SMPTE Decoder variables ***/

bool g_decoderEnabled = false;
bool g_bPostInterrupts = false;

SMPTETimecode g_rxTime;

volatile uint32_t g_uiPeriod = 0;
volatile uint32_t g_uiHighCount = 0;
volatile uint32_t g_uiLowCount = 0;
volatile uint32_t g_uiAveragePeriod = 0;
volatile uint32_t g_rxBitCount = 0;
volatile int32_t  g_nBitState = 0;
volatile bool     g_bFirstTransition = false;

const uint64_t sync_word_fwd = 0b0011111111111101;
const uint64_t sync_word_rev = 0b1011111111111100;

static ltcframe_t g_smpteWord;

/* Hwi_Struct for timer interrupt handlers */
static Hwi_Struct wtimer0AHwiStruct;
static Hwi_Struct wtimer0BHwiStruct;

static Mailbox_Handle mailboxWord = NULL;

/*** External Data Items ***/

extern SYSCFG g_cfg;
extern uint32_t g_systemClock;

/*** Static Function Prototypes ***/

static Void DecodeTaskFxn(UArg arg0, UArg arg1);
static Void WTimer0AHwi(UArg arg);
static Void WTimer0BHwi(UArg arg);
static void HandleEdgeChange(void);

static uint64_t reverseBits64(uint64_t x);

//*****************************************************************************
//********************** SMPTE DECODER SUPPORT ********************************
//*****************************************************************************

void SMPTE_initDecoder(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;
    Task_Params taskParams;
    Mailbox_Params mboxParams;

    /* Create INT_WTIMER0 hardware interrupt handlers */

    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&(wtimer0AHwiStruct), INT_WTIMER0A, WTimer0AHwi, &hwiParams, &eb);
    if (Error_check(&eb)) {
        System_abort("Couldn't construct WTIMER0A error hwi");
    }

    /* Create INT_WTIMER0B hardware interrupt handler */

    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&(wtimer0BHwiStruct), INT_WTIMER0B, WTimer0BHwi, &hwiParams, &eb);
    if (Error_check(&eb)) {
        System_abort("Couldn't construct WTIMER0B error hwi");
    }

    /* Create SMPTE packet decoder mailbox */
    Error_init(&eb);
    Mailbox_Params_init(&mboxParams);
    mailboxWord = Mailbox_create(sizeof(LTCFrameWord), 32, &mboxParams, &eb);
    if (mailboxWord == NULL) {
        System_abort("Mailbox create failed\nAborting...");
    }

    /* Create the SMPTE packet decoder task */
    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 10;
    Task_create((Task_FuncPtr)DecodeTaskFxn, &taskParams, &eb);
}

//*****************************************************************************
// SMPTE Input Edge Timing Interrupts (32-BIT WIDE TIMER IMPLEMENTATION)
//*****************************************************************************

uint64_t reverseBits64(uint64_t x)
{
    unsigned int s = sizeof(x) * 8;
    uint64_t mask = ~((uint64_t)0);

    while ((s >>= 1) > 0)
    {
        mask ^= mask << s;
        x = ((x >> s) & mask) | ((x << s) & ~mask);
    }

    return x;
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
#if (DEBUG_SMPTE > 0)
    uint8_t secs = 0;
#endif
    LTCFrameWord word;

    /* Initialize and start edge decode interrupts */
    SMPTE_Decoder_Start();

    /*
     * Loop waiting for SMPTE word packets to arrive
     */

    while (true)
    {
        /* Wait for an 80-bit timecode word */
        if (!Mailbox_pend(mailboxWord, &word, 100))
        {
            GPIO_write(Board_STAT_LED, Board_LED_ON);
            continue;
        }

        /* Toggle the LED on each packet received */
        GPIO_toggle(Board_STAT_LED);

        /* Set direction indicator pin */
        if (word.ltc.sync_word == sync_word_rev)
            GPIO_write(Board_DIRECTION, PIN_HIGH);
        else
            GPIO_write(Board_DIRECTION, PIN_LOW);

        /* Reverse all 64-bits in the data part of the SMPTE frame */
        word.raw.data = reverseBits64(word.raw.data);

        /* Serialize access to SMPTE data */
        IArg key = GateMutex_enter(gateMutex0);
        /* Now extract any time and other data from the packet */
        g_rxTime.frame = (uint8_t)(word.ltc.frame_units + (word.ltc.frame_tens * 10));
        g_rxTime.secs  = (uint8_t)(word.ltc.secs_units  + (word.ltc.secs_tens  * 10));
        g_rxTime.mins  = (uint8_t)(word.ltc.mins_units  + (word.ltc.mins_tens  * 10));
        g_rxTime.hours = (uint8_t)(word.ltc.hours_units + (word.ltc.hours_tens * 10));
        /* Release the gate mutex */
        GateMutex_leave(gateMutex0, key);

        /* Assert the interrupt line */
        if (g_bPostInterrupts)
        {
            /* Notify host a packet is ready */
            GPIO_write(Board_SMPTE_INT_N, PIN_LOW);
        }

#if (DEBUG_SMPTE > 0)
        if (secs != tc.secs)
        {
            secs = tc.secs;

            System_printf("%2u:%2u:%2u:%2u\n", tc.hours, tc.mins, tc.secs, tc.frame);
            System_flush();
        }
#endif
    }
}

//*****************************************************************************
// Initialize and reset all decoder global variables
//*****************************************************************************

Void SMPTE_Decoder_Reset(void)
{
    g_smpteWord.data = (uint64_t)0;
    g_smpteWord.sync = (uint16_t)0;

    g_nBitState = 0;
    g_bFirstTransition = false;

    g_uiPeriod    = 0;
    g_uiLowCount  = 0;
    g_uiHighCount = 0;
    g_rxBitCount  = 0;
}

//*****************************************************************************
// Initialize and start the SMPTE decoder edge timer interrupts
//*****************************************************************************

int SMPTE_Decoder_Start(void)
{
    g_decoderEnabled = true;
    g_bPostInterrupts = false;

    GPIO_write(Board_STAT_LED, Board_LED_ON);
    GPIO_write(Board_FRAME_SYNC, PIN_LOW);
    GPIO_write(Board_DIRECTION, PIN_LOW);

    SMPTE_Decoder_Reset();

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

    return 1;
}

//*****************************************************************************
// Stop the SMPTE Decoder
//*****************************************************************************

int SMPTE_Decoder_Stop(void)
{
    g_bPostInterrupts = false;
    g_decoderEnabled = false;

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

    return 1;
}

//*****************************************************************************
// SMPTE Input Edge Timing Interrupts (32-BIT WIDE TIMER IMPLEMENTATION)
//*****************************************************************************

/* Rising Edge Interrupt (Start of Pulse) */
Void WTimer0AHwi(UArg arg)
{
    /* Echo high pin change to SYNC pin */
    GPIO_write(Board_FRAME_SYNC, PIN_HIGH);

    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);

    /* Store the start time */
    g_uiHighCount = TimerValueGet(WTIMER0_BASE, TIMER_A);

    /* Call the edge change interrupt handler */
    HandleEdgeChange();
}

/* Falling Edge Interrupt (End of Pulse) */
Void WTimer0BHwi(UArg arg)
{
    /* Echo low pin change to SYNC pin */
    GPIO_write(Board_FRAME_SYNC, PIN_LOW);

    /* Clear the timer interrupt */
    TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);

    /* Store the end time */
    g_uiLowCount = TimerValueGet(WTIMER0_BASE, TIMER_B);

    /* Call the edge change interrupt handler */
    HandleEdgeChange();
}

//*****************************************************************************
// Handle high and low hardware level edge change interrupts while
// shifting in bits of the SMPTE data into our 80-bit packet buffer.
//*****************************************************************************

void HandleEdgeChange(void)
{
    bool new_bit_flag = false;

    /* Calculate the pulse period from rising edge to falling edge */
    if (g_uiLowCount > g_uiHighCount)
        g_uiPeriod = g_uiLowCount - g_uiHighCount;
    else
        g_uiPeriod = g_uiHighCount - g_uiLowCount;

    /* Calculate the average pulse period */
    g_uiAveragePeriod = (g_uiPeriod >> 2);

    /* Normally we'd divide the period count by 80 here to get the pulse
     * time in microseconds. However, we scale all the other timing
     * constants by 80 instead to avoid this division. Now look at the
     * period and decide if the bit is a one or zero.
     */
    if ((g_uiPeriod >= ONE_TIME_MIN) && (g_uiPeriod < ONE_TIME_MAX))
    {
        if (g_bFirstTransition)
        {
            /* First bit transition */
            g_bFirstTransition = false;
        }
        else
        {
            // Second bit transition
            g_nBitState = 1;
            g_bFirstTransition = true;
            new_bit_flag = true;
        }
    }
    else if ((g_uiPeriod >= ZERO_TIME_MIN) && (g_uiPeriod < ZERO_TIME_MAX))
    {
        g_nBitState = 0;
        g_bFirstTransition = true;
        new_bit_flag = true;
    }
    else
    {
        /* Either it's the first bit change, or data timing is wrong! */
        new_bit_flag = false;
    }

    if (new_bit_flag)
    {
        /* Shift the 16-bit sync word bits */
        g_smpteWord.sync = g_smpteWord.sync << 1;

        /* Carry bit-63 into sync bit-0 if needed, otherwise it's a zero bit */
        if (g_smpteWord.data & 0x8000000000000000)
            g_smpteWord.sync |= 1;

        /* Shift the 80-bit frame word bits */
        g_smpteWord.data = g_smpteWord.data << 1;

        /* Add in new smpte word bit, either zero or a one */
        if (g_nBitState)
            g_smpteWord.data |= 1;

        /* The 16-bit SMPTE sync word must be at the end of the frame
         * to consider it a valid 80-bit SMPTE frame.
         */
        if (++g_rxBitCount >= LTC_FRAME_BIT_COUNT)
        {
            if (g_smpteWord.sync == sync_word_fwd)
            {
                /* Post the 64-bit SMPTE word to decode task */
                Mailbox_post(mailboxWord, &g_smpteWord, BIOS_NO_WAIT);

                /* Reset bit counter and buffer */
                g_rxBitCount = 0;
            }
            else if (g_smpteWord.sync == sync_word_rev)
            {
                /* Post the 64-bit SMPTE word to decode task */
                Mailbox_post(mailboxWord, &g_smpteWord, BIOS_NO_WAIT);

                /* Reset bit counter and buffer */
                g_rxBitCount = 0;
            }
        }
    }
}

/* End-Of-File */
