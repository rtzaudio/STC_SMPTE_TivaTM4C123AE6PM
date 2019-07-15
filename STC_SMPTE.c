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

volatile int8_t g_state = 0;
volatile uint32_t g_frame_bitnum = 0;

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
    //if (encoder)
    //    return;
    strcpy(g_smpte_time.timezone, timezone);

    //g_smpte_time.years  = 8;
    //g_smpte_time.months = 12;
    //g_smpte_time.days   = 31;

    g_smpte_time.hours  = 23;
    g_smpte_time.mins   = 59;
    g_smpte_time.secs   = 59;
    g_smpte_time.frame  = 0;

    memset(&g_smpte_ones, 0xFF, sizeof(LTCFrame));
    memset(&g_smpte_zeros, 0x00, sizeof(LTCFrame));

    ltc_frame_reset(&g_smpte_frame);

    ltc_time_to_frame(&g_smpte_frame, &g_smpte_time, LTC_TV_525_60, 0);

    g_state = g_frame_bitnum = 0;

    /* Enable the signal out relay to connect the SMPTE
     * output signal to channel 24 on the tape machine.
     */
    GPIO_write(Board_RELAY, Board_RELAY_ON);
    Task_sleep(100);

    // Turn the LED on to indicate active
    GPIO_write(Board_STAT_LED, Board_LED_ON);

    // SMPTE output pin low initially
    GPIO_write(Board_SMPTE_OUT, PIN_LOW);

    // Set the TIMER1B load value to 1ms.
    TimerLoadSet(WTIMER1_BASE, TIMER_A, g_systemClock / 4800);  //4800);
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

Void Timer1AIntHandler(UArg arg)
{
    // Clear the timer interrupt flag.
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    uint8_t* pframe = (uint8_t*)&g_smpte_frame;
    uint8_t data;
    uint32_t i, s;

    switch(g_state)
    {
    case 0:
        GPIO_toggle(Board_SMPTE_OUT);
        ++g_state;
        break;

    case 1:
        i = g_frame_bitnum / 8;
        s = g_frame_bitnum % 8;

        data = pframe[i];

        if ((data >> s) & 0x01)
        {
            GPIO_toggle(Board_SMPTE_OUT);
            g_state = 5;
        }
        else
        {
            g_state = 3;
        }
        break;

    case 2:
        GPIO_toggle(Board_SMPTE_OUT);
        ++g_state;
        break;

    case 3:
        GPIO_toggle(Board_SMPTE_OUT);
        ++g_state;
        break;

    case 5:
        GPIO_toggle(Board_SMPTE_OUT);
        ++g_state;
        break;

    case 6:
        GPIO_toggle(Board_SMPTE_OUT);
    case 4:
        ++g_frame_bitnum;
        /* Is this the last bit of the SMPTE frame? */
        if (g_frame_bitnum >= LTC_FRAME_BIT_COUNT)
        {
            /* Yes, then increment the frame time code step */
            ltc_frame_increment(&g_smpte_frame, 30, LTC_TV_625_50, 0);

            /* reset the frame bit counter */
            g_frame_bitnum = 0;

            GPIO_toggle(Board_STAT_LED);
        }
        g_state = 0;
    }
}

Void Timer1BIntHandler(UArg arg)
{
    // Clear the timer interrupt flag.
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
}


#if 0


volatile uint32_t bit_time;
volatile bool valid_tc_word;
volatile bool ones_bit_count;
volatile bool tc_sync;
volatile bool write_tc_out;
volatile bool drop_frame_flag;

volatile uint8_t total_bits;
volatile uint8_t current_bit;
volatile uint8_t sync_count;

volatile uint8_t tc[8];
volatile int8_t timeCode[11];


// toggleCaptureEdge
//TCCR1B ^= _BV(ICES1);
//bit_time = ICR1;
// resetTimer1
//TCNT1 = 0;

// get rid of anything way outside the norm
if ((bit_time < ONE_TIME_MIN) || (bit_time > ZERO_TIME_MAX))
{
    total_bits = 0;
}
else
{
    // only count the second ones plus
    if (ones_bit_count == true)
    {
        ones_bit_count = false;
    }
    else
    {
        if (bit_time > ZERO_TIME_MIN)
        {
            current_bit = 0;
            sync_count = 0;
        }
        else //if (bit_time < one_time_max)
        {
            ones_bit_count = true;
            current_bit = 1;

            sync_count++;

            if (sync_count == 12) // part of the last two bytes of a timecode word
            {
                sync_count = 0;
                tc_sync = true;
                total_bits = END_SYNC_POSITION;
            }
        }

        if (total_bits <= END_DATA_POSITION) // timecode runs least to most so we need
        {                                    // to shift things around
            int n;

            tc[0] = tc[0] >> 1;

            for(n=1; n < 8; n++)
            {
                if(tc[n] & 1)
                    tc[n-1] |= 0x80;

                tc[n] = tc[n] >> 1;
            }

            if (current_bit == 1)
                tc[7] |= 0x80;
        }
        total_bits++;
    }

    if (total_bits == END_SMPTE_POSITION)   // we have the 80th bit
    {
        total_bits = 0;

        if (tc_sync)
        {
            tc_sync = false;
            valid_tc_word = true;
        }
    }

    if (valid_tc_word)
    {
        valid_tc_word = false;

        timeCode[10] = (tc[0]&0x0F)+0x30;       // frames
        timeCode[9]  = (tc[1]&0x03)+0x30;        // 10's of frames
        timeCode[8]  =  '.';
        timeCode[7]  = (tc[2]&0x0F)+0x30;        // seconds
        timeCode[6]  = (tc[3]&0x07)+0x30;        // 10's of seconds
        timeCode[5]  =  ':';
        timeCode[4]  = (tc[4]&0x0F)+0x30;        // minutes
        timeCode[3]  = (tc[5]&0x07)+0x30;        // 10's of minutes
        timeCode[2]  = ':';
        timeCode[1]  = (tc[6]&0x0F)+0x30;        // hours
        timeCode[0]  = (tc[7]&0x03)+0x30;        // 10's of hours

        //drop_frame_flag = bit_is_set(tc[1], 2);

        write_tc_out = true;
    }
}


void setup()
{
 beginSerial (115200);
 pinMode(icpPin, INPUT);                  // ICP pin (digital pin 8 on arduino) as input

 bit_time = 0;
 valid_tc_word = false;
 ones_bit_count = false;
 tc_sync = false;
 write_tc_out = false;
 drop_frame_flag = false;
 total_bits =  0;
 current_bit =  0;
 sync_count =  0;

 //Serial.println("Finished setup ");
 //delay (1000);

 TCCR1A = B00000000; // clear all
 TCCR1B = B11000010; // ICNC1 noise reduction + ICES1 start on rising edge + CS11 divide by 8
 TCCR1C = B00000000; // clear all
 TIMSK1 = B00100000; // ICIE1 enable the icp

 TCNT1 = 0; // clear timer1
}

void loop()
{
   if (write_tc_out)
   {
     write_tc_out = false;
     if (drop_frame_flag)
       Serial.print("TC-[df] ");
     else
       Serial.print("TC-[nd] ");
     Serial.print((char*)timeCode);
     Serial.print("\r");
   }
}

/////////////////////////////////////////////////////////////////////////////////////////////

For pal: 25 and 24 Fps

#define ONE_TIME_MAX          588 // these values are setup for NTSC video
#define ONE_TIME_MIN          422 // PAL would be around 1000 for 0 and 500 for 1
#define ZERO_TIME_MAX          1080 // 80bits times 29.97 frames per sec
#define ZERO_TIME_MIN          922 // equals 833 (divide by 8 clock pulses)

For User bit :

       userBit[9] = ((tc[0]&0xF0)>>4)+0x30; // user bits 8
       userBit[8] = ((tc[1]&0xF0)>>4)+0x30; // user bits 7
       userBit[7] = ((tc[2]&0xF0)>>4)+0x 30; // user bits 6
       userBit[6] = ((tc[3]&0xF0)>>4)+0x30; // user bits 5
       userBit[5] = '-';
       userBit[4] = ((tc[4]&0xF0)>>4)+0x30; // user bits 4
       userBit[3] = ((tc[5]&0xF0)>>4)+0x30; // user bits 3
       userBit[2] = '-';
       userBit[1] = ((tc[6]&0xF0)>>4)+0x30; // user bits 2
       userBit[0] = ((tc[7]&0xF0)>>4)+0x30; // user bits 1
#endif

/* End-Of-File */
