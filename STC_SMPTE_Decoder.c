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

/* SMPTE 80-bit frame buffer */
typedef struct ltcframe_t {
    uint64_t data;              /* 64-bits data */
    uint16_t sync;              /* 16-bits sync */
} ltcframe_t;                   /* 80-bit frame */

typedef union LTCFrameWord_t {
    LTCFrame    ltc;            /* members form */
    ltcframe_t  raw;            /* raw bit form */
} LTCFrameWord;

/*** SMPTE Decoder variables ***/

bool g_decoderEnabled = false;

volatile uint32_t g_uiPeriod = 0;
volatile uint32_t g_uiHighCount = 0;
volatile uint32_t g_uiLowCount = 0;
volatile uint32_t g_uiAveragePeriod = 0;
volatile uint32_t g_rxBitCount = 0;

volatile bool first_transition = false;
volatile int new_bit = 0;

const uint64_t sync_word_fwd = 0b0011111111111101;
const uint64_t sync_word_rev = 0b1011111111111100;
const uint64_t sync_mask = 0b1111111111111111;

/* Hwi_Struct for timer interrupt handlers */
static Hwi_Struct wtimer0AHwiStruct;
static Hwi_Struct wtimer0BHwiStruct;

static Mailbox_Handle mailboxWord = NULL;

static SMPTETimecode g_rxTime;
static ltcframe_t smpte_word;

/*** External Data Items ***/

extern SYSCFG g_cfg;
extern  uint32_t g_systemClock;

/*** Static Function Prototypes ***/

static Void DecodeTaskFxn(UArg arg0, UArg arg1);
static Void WTimer0AHwi(UArg arg);
static Void WTimer0BHwi(UArg arg);
static void HandleEdgeChange(void);

static uint8_t decode_part(uint64_t bit_string, int start_bit, int stop_bit);
static uint64_t reverse_bit_order(uint64_t v, int significant_bits);

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

    Hwi_construct(&(wtimer0AHwiStruct),
                  INT_WTIMER0A,
                  WTimer0AHwi,
                  &hwiParams,
                  &eb);

    if (Error_check(&eb)) {
        System_abort("Couldn't construct WTIMER0A error hwi");
    }

    /* Create INT_WTIMER0B hardware interrupt handler */

    Error_init(&eb);
    Hwi_Params_init(&hwiParams);

    Hwi_construct(&(wtimer0BHwiStruct),
                  INT_WTIMER0B,
                  WTimer0BHwi,
                  &hwiParams,
                  &eb);

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

Void DecodeTaskFxn(UArg arg0, UArg arg1)
{
    LTCFrameWord word;

    /* Initialize and start edge decode interrupts */
    SMPTE_Decoder_Start();

    /*
     * Loop waiting for SMPTE word packets to arrive
     */

    while (true)
    {
        /* Wait for a 64-bit timecode word */
        if (!Mailbox_pend(mailboxWord, &word, 100))
        {
            GPIO_write(Board_STAT_LED, Board_LED_ON);
            continue;
        }

        /* Toggle the LED on each packet received */
        GPIO_toggle(Board_STAT_LED);

        //uint64_t reversed_bit_string = reverse_bit_order(word, 61);

        g_rxTime.frame = decode_part(word.raw.data, 0, 3)   + (decode_part(word.raw.data, 8, 9) * 10);
        g_rxTime.secs  = decode_part(word.raw.data, 16, 19) + (decode_part(word.raw.data, 24, 26) * 10);
        g_rxTime.mins  = decode_part(word.raw.data, 32, 35) + (decode_part(word.raw.data, 40, 42) * 10);
        g_rxTime.hours = decode_part(word.raw.data, 48, 51) + (decode_part(word.raw.data, 56, 57) * 10);
    }
}

//*****************************************************************************
// Initialize and reset all decoder global variables
//*****************************************************************************

Void SMPTE_Decoder_Reset(void)
{
    /* Zero out the starting time struct */
    memset(&g_rxTime, 0, sizeof(g_rxTime));

    smpte_word.data = (uint64_t)0;
    smpte_word.sync = (uint16_t)0;

    first_transition = false;
    new_bit = 0;

    g_rxBitCount = 0;
    g_uiPeriod = 0;
    g_uiLowCount = 0;
    g_uiHighCount = 0;
}

//*****************************************************************************
// Initialize and start the SMPTE decoder edge timer interrupts
//*****************************************************************************

int SMPTE_Decoder_Start(void)
{
    g_decoderEnabled = true;

    GPIO_write(Board_STAT_LED, Board_LED_ON);
    GPIO_write(Board_FRAME_SYNC, PIN_LOW);

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

    HandleEdgeChange();
}

//*****************************************************************************
// Handle high and low hardware level edge change interrupts while
// shifting in bits of the SMPTE data into our 80-bit packet buffer.
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
        /* Shift the 16-bit sync word bits */
        smpte_word.sync = smpte_word.sync << 1;

        /* Carry bit-63 into sync bit-0 if needed, otherwise it's a zero bit */
        if (smpte_word.data & 0x8000000000000000)
            smpte_word.sync |= 1;

        /* Shift the 80-bit frame word bits */
        smpte_word.data = smpte_word.data << 1;

        /* Add in new smpte word bit, either zero or a one */
        if (new_bit)
            smpte_word.data |= 1;

        /* The 16-bit SMPTE sync word must be at the end of the frame
         * to consider it a valid 80-bit SMPTE frame.
         */
        if (++g_rxBitCount >= LTC_FRAME_BIT_COUNT)
        {
            if (smpte_word.sync == sync_word_fwd)
            {
                /* Post the 64-bit SMPTE word to decode task */
                Mailbox_post(mailboxWord, &smpte_word, BIOS_NO_WAIT);

                /* Reset bit counter and buffer */
                g_rxBitCount = 0;
            }
            else if (smpte_word.sync == sync_word_rev)
            {
                /* Post the 64-bit SMPTE word to decode task */
                Mailbox_post(mailboxWord, &smpte_word, BIOS_NO_WAIT);

                /* Reset bit counter and buffer */
                g_rxBitCount = 0;
            }
        }
    }
}

#if 0
void decode_ltc_bit(int new_bit)
{
    // Shift one bit
    bit_string = (bit_string << 1);

    // a bit shift adds a zero, if a 1 is needed add 1
    if (new_bit == 1)
        bit_string |= 1;

    // if the current_word matches the sync word
    if ((bit_string & sync_mask) == sync_word_fwd)
    {
        bit_string = 0;
        //since_sync_found = 0;
        bit_index = -1;         //bit 79

        //a full frame has passed
        //ltc_frame_complete();
    }
    else //if (since_sync_found < 33333)
    {
        //bit index in LTC message
        bit_index++;

        //info above bit 60 is ignored.
        if (bit_index == 60)
        {
            //bit string length = index + 1 = 61
            uint64_t reversed_bit_string = reverse_bit_order(bit_string, 61);

            //see https://en.wikipedia.org/wiki/Linear_timecode

            //This decodes only time info, user fields are ignored
            g_rxTime.frame = decode_part(reversed_bit_string, 0, 3)   + 10 * decode_part(reversed_bit_string, 8, 9);
            g_rxTime.secs  = decode_part(reversed_bit_string, 16, 19) + 10 * decode_part(reversed_bit_string, 24, 26);
            g_rxTime.mins  = decode_part(reversed_bit_string, 32, 35) + 10 * decode_part(reversed_bit_string, 40, 42);
            g_rxTime.hours = decode_part(reversed_bit_string, 48, 51) + 10 * decode_part(reversed_bit_string, 56, 57);

            bit_string = 0;
        }
    }
}
#endif


// Reverses the bit order of the bit string in v, keeping only the given amount
// of bits
//
// Adapted from the code here: http://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
//
// The following is true:
//  uint64_t a = 0b0011001;
//  uint64_t expected_result = 0b1001100;
//  expected_result==reverse_bit_order(a,7);

uint64_t reverse_bit_order(uint64_t v, int significant_bits)
{
    uint64_t r = v;             // r will be reversed bits of v; first get LSB of v
    int s = sizeof(v) * 8 - 1;  // extra shift needed at end

    for (v >>= 1; v; v >>= 1)
    {
        r <<= 1;
        r |= v & 1;
        s--;
    }

    r <<= s;

    return r >> (64 - significant_bits);
}

// Decode a part of a bit string in word_value
// The value of bit string starting at start_bit to and including stop_bit is returned.
//
// The following is true:
//  uint64_t a = 0b1011001;
//  uint64_t expected_result = 0b1011;
//  expected_result == decode_part(a,3,6);

uint8_t decode_part(uint64_t bit_string, int start_bit, int stop_bit)
{
  // This shift puts the start bit on the first place

  uint64_t shifted_bit_string = bit_string >> start_bit;

  // Create a bit-mask of the required length
  // including stop bit so add 1

  int bit_string_slice_length = stop_bit - start_bit + 1;

  uint64_t bit_mask = (1 << bit_string_slice_length) - 1;

  // Apply the mask, effectively ignoring all other bits

  uint64_t masked_bit_string = shifted_bit_string & bit_mask;

  return (uint8_t)masked_bit_string;
}

//*****************************************************************************
//
//*****************************************************************************


#if 0
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
