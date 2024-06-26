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

#include "libltc\ltc.h"

#define FIRMWARE_VER        2           /* firmware version */
#define FIRMWARE_REV        33        	/* firmware revision */
#define FIRMWARE_BUILD      1           /* firmware build number */
#define FIRMWARE_MIN_BUILD  1           /* min build req'd to force reset */

#define MAGIC               0xCEB0FACE  /* magic number for EEPROM data */
#define MAKEREV(v, r)       ((v << 16) | (r & 0xFFFF))

#define UNDEFINED           ((uint32_t)(-1))

#define TIMEOUT_SPI			500		/* Timeout for SPI communications */

/* pulse times in microseconds
 * pulse width: 416.7us(30fps)
 * 80bit x 30frame/s --> 416.7us/bit
 */
#define MAX_DELTA                   25
#define EXPECTED_DURATION_SINGLE    208
#define EXPECTED_DURATION_DOUBLE    (EXPECTED_DURATION_SINGLE * 2)

#ifdef PAL_24FPS
/* For PAL and 24 FPS */
#define ONE_TIME_MAX        (588 * 80)  /* these values are setup for NTSC video */
#define ONE_TIME_MIN        (422 * 80)  /* PAL would be around 1000 for 0 and 500 for 1 */
#define ZERO_TIME_MAX       (1080 * 80) /* 80bits times 29.97 frames per sec */
#define ZERO_TIME_MIN       (922 * 80)  /* equals 833 (divide by 8 clock pulses) */
#else
/* these values are setup for NTSC video */
#define ONE_TIME_MAX        ((EXPECTED_DURATION_SINGLE + MAX_DELTA) * 80)
#define ONE_TIME_MIN        ((EXPECTED_DURATION_SINGLE - MAX_DELTA) * 80)
#define ZERO_TIME_MAX       ((EXPECTED_DURATION_DOUBLE + MAX_DELTA) * 80)
#define ZERO_TIME_MIN       ((EXPECTED_DURATION_DOUBLE - MAX_DELTA) * 80)
#endif

#define FPS24_REF           521
#define FPS25_REF           500
#define FPS30_REF           417

/*** System Structures *****************************************************/

/* This structure contains runtime and program configuration data that is
 * stored and read from EEPROM. The structure size must be 4 byte aligned.
 */

typedef struct _SYSCFG
{
	uint32_t magic;
	uint32_t version;
	uint32_t build;
    /*** GLOBAL PARAMETERS ***/
	uint32_t debug;
    uint32_t sysflags;
    char timezone[6];
    int32_t frame_rate;
    bool drop_frame;
} SYSCFG;

/*** Function Prototypes ***************************************************/

Int main();
void InitSysDefaults(SYSCFG* p);
int32_t SysParamsWrite(SYSCFG* sp);
int32_t SysParamsRead(SYSCFG* sp);
Void SPI_SlaveTask(UArg a0, UArg a1);

int SMPTE_Encoder_Start();
int SMPTE_Encoder_Stop(void);
void SMPTE_Encoder_Reset(void);
Void WTimer1AIntHandler(UArg arg);
Void WTimer1BIntHandler(UArg arg);

void SMPTE_initDecoder(void);
int SMPTE_Decoder_Start();
int SMPTE_Decoder_Stop(void);
void SMPTE_Decoder_Reset(void);

/* End-Of-File */
