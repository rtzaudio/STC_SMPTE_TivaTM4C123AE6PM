/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2021-2026, RTZ Professional Audio, LLC
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
#include <xdc/runtime/Memory.h>

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
#include "STC_SMPTE_Queue.h"

/* Constants and Macros */

//*****************************************************************************
//
//*****************************************************************************

Bool LTCQueue_init(LTCFrameQueue *queue)
{
    Int i;
    Error_Block eb;
    Error_init(&eb);

    /* Allocate list management semaphores */

    queue->free_semaphore = Semaphore_create(LTC_FRAME_QUEUE_SIZE, NULL, &eb);

    if (queue->free_semaphore == NULL)
        System_abort("Free-Q semphore alloc failed!");

    queue->data_semaphore = Semaphore_create(0, NULL, &eb);

    if (queue->data_semaphore == NULL)
        System_abort("Data-Q list semphore alloc failed!");

    /* Allocate message frame buffer space for all queue elements in the linked list */

    queue->buf = (LTCFrameQueueElement*)Memory_alloc(NULL, sizeof(LTCFrameQueueElement) * LTC_FRAME_QUEUE_SIZE, 0, &eb);

    if (queue->buf == NULL)
        System_abort("TxBuf allocation failed");

    /* Create queues for free list and data ready */

    queue->free_queue = Queue_create(NULL, &eb);
    queue->data_queue = Queue_create(NULL, &eb);

    queue->freeCount = 0;

    LTCFrameQueueElement* msg = queue->buf;

    /* put all frame buffers in the free queue */

    for (i=0; i < LTC_FRAME_QUEUE_SIZE; i++, msg++)
    {
        Queue_enqueue(queue->free_queue, (Queue_Elem*)msg);

        queue->freeCount++;
    }

    queue->free_semaphore = Semaphore_create(LTC_FRAME_QUEUE_SIZE, NULL, NULL);
    queue->data_semaphore = Semaphore_create(0, NULL, NULL);

    return TRUE;
}

 //*****************************************************************************
 //
 //*****************************************************************************

Bool LTCQueue_pend(LTCFrameQueue *queue, LTCFrame* frame, UInt32 timeout)
{
    LTCFrameQueueElement* elem;

    if (Semaphore_pend(queue->data_semaphore, timeout))
    {
        /* Get pointer to next item in the queue */
        elem = Queue_get(queue->data_queue);

        /* perform the enqueue and increment numFreeMsgs atomically */
        UInt key = Hwi_disable();
        /* put message on freeQue */
        Queue_enqueue(queue->free_queue, (Queue_Elem*)elem);
        /* increement atomically */
        queue->freeCount++;
        /* re-enable ints */
        Hwi_restore(key);

        /* return message data */
        //memcpy(frame, &(elem->frame), sizeof(LTCFrameQueueElement));
        *frame = elem->frame;

        /* post the semaphore */
        Semaphore_post(queue->free_semaphore);

        return TRUE;
    }

    return FALSE;
 }

 //*****************************************************************************
 // This function posts a message to the transmit queue. A return FALSE value
 // indicates the timeout expired or a buffer never became available for
 // transmission within the timeout period specified.
 //*****************************************************************************

 Bool LTCQueue_post(LTCFrameQueue *queue, LTCFrame* frame, UInt32 timeout)
 {
     LTCFrameQueueElement* elem;

     /* Wait for a free queue element buffer and timeout if necessary */
     if (Semaphore_pend(queue->free_semaphore, timeout))
     {
         /* perform the dequeue and decrement free count atomically */
         UInt key = Hwi_disable();
         /* get element buffer from the free queue */
         elem = Queue_dequeue(queue->free_queue);
         /* decrement the numFreeMsgs */
         queue->freeCount--;
         /* re-enable ints */
         Hwi_restore(key);

         //memcpy(&(elem->frame), frame, sizeof(LTCFrameQueueElement));
         elem->frame = *frame;

         /* Add message to end of queue */
         Queue_put(queue->data_queue, (Queue_Elem *)elem);

         /* post the semaphore */
         Semaphore_post(queue->data_semaphore);

         return TRUE;      /* success */
     }

     return FALSE;         /* error */
}

/* End-Of-File */
