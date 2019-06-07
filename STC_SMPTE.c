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

#include <ti/sysbios/hal/Timer.h>

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

/* Constants */

#define ONE_TIME_MAX          475   // these values are setup for NTSC video
#define ONE_TIME_MIN          300   // PAL would be around 1000 for 0 and 500 for 1
#define ZERO_TIME_MAX         875   // 80bits times 29.97 frames per sec
#define ZERO_TIME_MIN         700   // equals 833 (divide by 8 clock pulses)

#define END_DATA_POSITION      63
#define END_SYNC_POSITION      77
#define end_smpte_position     80

/* Global Data Items */

SYSPARMS g_sys;

volatile unsigned int bit_time;
volatile boolean valid_tc_word;
volatile boolean ones_bit_count;
volatile boolean tc_sync;
volatile boolean write_tc_out;
volatile boolean drop_frame_flag;

volatile uchar8_t total_bits;
volatile uchar8_t current_bit;
volatile uchar8_t sync_count;

volatile byte tc[8];
volatile char timeCode[11];

/* Static Function Prototypes */

Int main();
Void SlaveTask(UArg a0, UArg a1);

Void timerFunc(UArg arg);

//*****************************************************************************
// Main Program Entry Point
//*****************************************************************************

Int main()
{
    Error_Block eb;
    Task_Params taskParams;

    Board_initGeneral();
    Board_initGPIO();
    Board_initSPI();

    /* Now start the main application button polling task */

    Error_init(&eb);

    Task_Params_init(&taskParams);

    taskParams.stackSize = 1248;
    taskParams.priority  = 10;

    if (Task_create(SlaveTask, &taskParams, &eb) == NULL)
        System_abort("SlaveTask!\n");

    BIOS_start();    /* does not return */

    return(0);
}

//*****************************************************************************
// Set default runtime values
//*****************************************************************************

void InitSysDefaults(SYSPARMS* p)
{
    /* default parameters */
    p->version  = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    p->build    = FIRMWARE_BUILD;
    p->debug    = 0;
    p->sysflags = 0;
}

//*****************************************************************************
// Write system parameters from our global settings buffer to EEPROM.
//
// Returns:  0 = Sucess
//          -1 = Error writing EEPROM data
//*****************************************************************************

int32_t SysParamsWrite(SYSPARMS* sp)
{
    int32_t rc = 0;

    sp->version = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    sp->build   = FIRMWARE_BUILD;
    sp->magic   = MAGIC;

    rc = EEPROMProgram((uint32_t *)sp, 0, sizeof(SYSPARMS));

    System_printf("Writing System Parameters (size=%d)\n", sizeof(SYSPARMS));
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

int32_t SysParamsRead(SYSPARMS* sp)
{
    InitSysDefaults(sp);

    EEPROMRead((uint32_t *)sp, 0, sizeof(SYSPARMS));

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

    System_printf("System Parameters Loaded (size=%d)\n", sizeof(SYSPARMS));
    System_flush();

    return 0;
}

//*****************************************************************************
// The main application initialization, setup and button controler task.
//*****************************************************************************

Void SlaveTask(UArg a0, UArg a1)
{
    Timer_Handle hTimer;
    Timer_Params timerParams;
    Error_Block eb;

    /* Initialize the default servo and program data values */
    memset(&g_sys, 0, sizeof(SYSPARMS));

    /* Read system parameters from EEPROM */
    //SysParamsRead(&g_sys);
    InitSysDefaults(&g_sys);

    /* Configure a Timer to interrupt every 100ms
     * timerFunc() provides Hwi load and posts a Swi and Semaphore
     * to provide Swi and Task loads and adjusts the loads every 5 seconds.
     */

    Timer_Params_init(&timerParams);

    timerParams.startMode  = Timer_StartMode_AUTO;
    timerParams.period     = 250000;         /* 100,000 uSecs = 100ms */
    timerParams.periodType = Timer_PeriodType_MICROSECS;
    timerParams.arg        = 1;

    Error_init(&eb);

    hTimer = Timer_create(Timer_ANY, timerFunc, &timerParams, &eb);

    if (hTimer == NULL) {
        System_abort("Timer create failed");
    }

    /****************************************************************
     * Enter the main application button processing loop forever.
     ****************************************************************/

    Timer_start(hTimer);

    for(;;)
    {
        /* No message, blink the LED */
        //GPIO_toggle(Board_STAT_LED);

        /* delay for 5ms and loop */
        Task_sleep(500);
    }
}


Void timerFunc(UArg arg)
{
    GPIO_toggle(Board_STAT_LED);
}



#if 0

#define icpPin 8        // ICP input pin on arduino
#define ONE_TIME_MAX          475 // these values are setup for NTSC video
#define ONE_TIME_MIN          300 // PAL would be around 1000 for 0 and 500 for 1
#define ZERO_TIME_MAX         875 // 80bits times 29.97 frames per sec
#define ZERO_TIME_MIN         700 // equals 833 (divide by 8 clock pulses)

#define END_DATA_POSITION      63
#define END_SYNC_POSITION      77
#define END_SMPTE_POSITION     80

volatile unsigned int bit_time;
volatile boolean valid_tc_word;
volatile boolean ones_bit_count;
volatile boolean tc_sync;
volatile boolean write_tc_out;
volatile boolean drop_frame_flag;

volatile byte total_bits;
volatile byte current_bit;
volatile byte sync_count;

volatile byte tc[8];
volatile char timeCode[11];

/* ICR interrupt vector */
ISR(TIMER1_CAPT_vect)
{
 //toggleCaptureEdge
 TCCR1B ^= _BV(ICES1);

 bit_time = ICR1;

 //resetTimer1
 TCNT1 = 0;

 if ((bit_time < ONE_TIME_MIN) || (bit_time > ZERO_TIME_MAX)) // get rid of anything way outside the norm
 {
   //Serial.println(bit_time, DEC);
   total_bits = 0;
 }
 else
 {
   if (ones_bit_count == true) // only count the second ones pluse
     ones_bit_count = false;
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
       tc[0] = tc[0] >> 1;

       for(int n=1;n<8;n++)
       {
         if(tc[n] & 1)
           tc[n-1] |= 0x80;

         tc[n] = tc[n] >> 1;
       }

       if(current_bit == 1)
         tc[7] |= 0x80;
     }
     total_bits++;
   }

   if (total_bits == end_smpte_position) // we have the 80th bit
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

     timeCode[10] = (tc[0]&0x0F)+0x30;      // frames
     timeCode[9] = (tc[1]&0x03)+0x30;      // 10's of frames
     timeCode[8] =  '.';
     timeCode[7] = (tc[2]&0x0F)+0x30;      // seconds
     timeCode[6] = (tc[3]&0x07)+0x30;      // 10's of seconds
     timeCode[5] =  ':';
     timeCode[4] = (tc[4]&0x0F)+0x30;      // minutes
     timeCode[3] = (tc[5]&0x07)+0x30;      // 10's of minutes
     timeCode[2] = ':';
     timeCode[1] = (tc[6]&0x0F)+0x30;      // hours
     timeCode[0] = (tc[7]&0x03)+0x30;      // 10's of hours

     drop_frame_flag = bit_is_set(tc[1], 2);

     write_tc_out = true;
   }
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
       userBit[7] = ((tc[2]&0xF0)>>4)+0x30; // user bits 6
       userBit[6] = ((tc[3]&0xF0)>>4)+0x30; // user bits 5
       userBit[5] = '-';
       userBit[4] = ((tc[4]&0xF0)>>4)+0x30; // user bits 4
       userBit[3] = ((tc[5]&0xF0)>>4)+0x30; // user bits 3
       userBit[2] = '-';
       userBit[1] = ((tc[6]&0xF0)>>4)+0x30; // user bits 2
       userBit[0] = ((tc[7]&0xF0)>>4)+0x30; // user bits 1
#endif

/* End-Of-File */
