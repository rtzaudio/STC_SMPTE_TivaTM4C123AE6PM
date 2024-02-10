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

uint64_t bit_string = 0;
int bit_index = 0;

//the sync word is available in bit 64 - 79,
//see https://en.wikipedia.org/wiki/Linear_timecode

const uint64_t sync_word_fwd = 0b0011111111111101;
const uint64_t sync_word_rev = 0b1011111111111100;
const uint64_t sync_mask = 0b1111111111111111;

static void HandleEdgeChange(void);

void ltc_frame_complete(void);
void decode_ltc_bit(int new_bit);
int decode_part(uint64_t bit_string, int start_bit,int stop_bit);
uint64_t reverse_bit_order(uint64_t v, int significant_bits);

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
        //decode_ltc_bit(new_bit);

#if 1
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

        /* Must see the SMPTE sync word at the end of frame to consider it a valid
         * packet. Parse out the frame members into our buffer if sync word found.
         */

        if (++g_rxBitCount >= LTC_FRAME_BIT_COUNT)
        {
            /* Toggle the LED on each packet received */
            GPIO_toggle(Board_STAT_LED);

            if (g_rxFrame.sync_word == sync_word_fwd)
            {
                //bit string length = index + 1 = 61
                uint64_t reversed_bit_string = reverse_bit_order(w64, 61);

                //This decodes only time info, user fields are ignored
                g_rxTime.frame = decode_part(reversed_bit_string, 0, 3)   + 10 * decode_part(reversed_bit_string, 8, 9);
                g_rxTime.secs  = decode_part(reversed_bit_string, 16, 19) + 10 * decode_part(reversed_bit_string, 24, 26);
                g_rxTime.mins  = decode_part(reversed_bit_string, 32, 35) + 10 * decode_part(reversed_bit_string, 40, 42);
                g_rxTime.hours = decode_part(reversed_bit_string, 48, 51) + 10 * decode_part(reversed_bit_string, 56, 57);

                /* Reset the pulse bit counter */
                g_rxBitCount = 0;
            }
            else if (g_rxFrame.sync_word == sync_word_rev)
            {
                /* Reset the pulse bit counter */
                g_rxBitCount = 0;
            }
        }
#endif
    }
}



// This is called every time a frame is complete (when bit 79 has been decoded).
void ltc_frame_complete(void)
{
    /* Toggle the LED on each packet received */
    GPIO_toggle(Board_STAT_LED);

/*
  if(current_frame_info.frames == 29){
    out_state_1Hz = !out_state_1Hz;
    digitalWrite(out_pin_1Hz,out_state_1Hz);
    Serial.print("1Hz ");
  }

  out_state_30Hz = !out_state_30Hz;
  digitalWrite(out_pin_30Hz,out_state_30Hz);

  int elapsed_micros = since_complete;

  digitalWrite(led_pin,!digitalRead(led_pin));

  Serial.printf("%d %02d:%02d:%02d.%02d\n",elapsed_micros, current_frame_info.hours,current_frame_info.minutes,current_frame_info.seconds,current_frame_info.frames);
  since_complete = 0;
  */
}

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
        ltc_frame_complete();
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

int decode_part(uint64_t bit_string, int start_bit,int stop_bit)
{
  // This shift puts the start bit on the first place
  uint64_t shifted_bit_string = bit_string >> start_bit;

  // Create a bit-mask of the required length
  // including stop bit so add 1
  int bit_string_slice_length = stop_bit - start_bit + 1;

  uint64_t bit_mask = (1<<bit_string_slice_length) - 1;

  // Apply the mask, effectively ignoring all other bits
  uint64_t masked_bit_string = shifted_bit_string & bit_mask;

  return  (int)masked_bit_string;
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
