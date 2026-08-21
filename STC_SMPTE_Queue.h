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

/* VERSION INFO - The min build specifies the minimum build required
 * that does NOT force a default reset of all the config parameters
 * at run time. For instance, if the config loads build 5 and the minimum
 * is set to 3, then it will reset config for anything less than build 3.
 * Likewise, versions 3 or higher would load and use the config values from
 * eprom as normal. This provides a means to force run time config defaults
 * to be reset or not.
 */

#ifndef STC_SMPTE_QUEUE_H
#define STC_SMPTE_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#include "libltc/ltc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*** LTCFrame Queue Elements ***********************************************/

#define LTC_FRAME_QUEUE_SIZE    8

typedef struct _LTCFrameQueueElement {
    Queue_Elem  elem;
    LTCFrame    frame;
} LTCFrameQueueElement;

typedef struct _LTCFrameQueue {
    /* queues and semaphores */
    Queue_Handle            free_queue;
    Queue_Handle            data_queue;
    Semaphore_Handle        free_semaphore;
    Semaphore_Handle        data_semaphore;
    LTCFrameQueueElement*   buf;                /* array of queue elements */
    uint32_t                freeCount;
} LTCFrameQueue;

/*** IPC FUNCTION PROTOTYPES ***********************************************/

#ifdef __cplusplus
}
#endif

#endif /* SMPTE_QUEUE_H */

/* End-Of-File */
