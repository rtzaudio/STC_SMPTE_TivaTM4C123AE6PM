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

/* Standard includes */
#include <file.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>

/* Standard Stellaris includes */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"
#include "driverlib/qei.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

/*** Global Constants ******************************************************/

/* VERSION INFO - The min build specifies the minimum build required
 * that does NOT force a default reset of all the config parameters
 * at run time. For instance, if the config loads build 5 and the minimum
 * is set to 3, then it will reset config for anything less than build 3.
 * Likewise, versions 3 or higher would load and use the config values from
 * eprom as normal. This provides a means to force run time config defaults
 * to be reset or not.
 */
#define FIRMWARE_VER        2           /* firmware version */
#define FIRMWARE_REV        33        	/* firmware revision */
#define FIRMWARE_BUILD      1           /* firmware build number */
#define FIRMWARE_MIN_BUILD  1           /* min build req'd to force reset */

#define MAGIC               0xCEB0FACE  /* magic number for EEPROM data */
#define MAKEREV(v, r)       ((v << 16) | (r & 0xFFFF))

#define UNDEFINED           ((uint32_t)(-1))

#define TIMEOUT_SPI			500		/* Timeout for SPI communications */

#ifdef PAL_24FPS
/* For PAL and 24 FPS */
#define ONE_TIME_MAX        588     // these values are setup for NTSC video
#define ONE_TIME_MIN        422     // PAL would be around 1000 for 0 and 500 for 1
#define ZERO_TIME_MAX       1080    // 80bits times 29.97 frames per sec
#define ZERO_TIME_MIN       922     // equals 833 (divide by 8 clock pulses)
#else
#define ONE_TIME_MAX        475     // these values are setup for NTSC video
#define ONE_TIME_MIN        300     // PAL would be around 1000 for 0 and 500 for 1
#define ZERO_TIME_MAX       875     // 80bits times 29.97 frames per sec
#define ZERO_TIME_MIN       700     // equals 833 (divide by 8 clock pulses)
#endif

#define END_DATA_POSITION   63
#define END_SYNC_POSITION   77
#define END_SMPTE_POSITION  80

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
} SYSCFG;

/*** SPI Slave Registers ***************************************************/

/* All registers are 16-bits with the upper word containing the command
 * and flag bits. The lower 8-bits contains any associated data byte.
 *
 *   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *   | R | F | F | F | C | C | C | C | B | B | B | B | B | B | B | B |
 *   | W | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *     |   |       |   |           |   |                           |
 *     |   +---+---+   +-----+-----+   +-------------+-------------+
 *     |       |             |                       |
 *    R/W     RSVD          REG                  DATA/FLAGS
 */

/* SMPTE Controller Registers */
#define SMPTE_REG_MODE      0           /* Mode Register   (RW, 8-bit  */
#define SMPTE_REG_CONF      1           /* Config Register (RW, 8-bit) */
#define SMPTE_REG_STAT      2           /* Status Register (RO, 8-bit) */
#define SMPTE_REG_DATA      3           /* Data Register   (RO, 8-bit) */

#define SMPTE_REG_MASK      0x0F00      /* C0-C3 register op-code      */
#define SMPTE_F_READ        0x8000      /* Bit-16 1=read 0=write reg   */

/* SMPTE_REG_MODE Register (HI-BYTE) */
#define SMPTE_MODE_DISABLE  0           /* SMPTE controller disabled   */
#define SMPTE_MODE_MASTER   1           /* SMPTE master mode           */
#define SMPTE_MODE_SLAVE    2           /* SMPTE slave mode            */

#define SMPTE_MODE_MASK     0x03
#define SMPTE_MODE_SEL(x)   (((x) & SMPTE_MODE_MASK) << 8)

/* SMPTE_REG_CONF Register (LO-BYTE) */
#define SMPTE_FPS_24        0           /* Generator set for 24 FPS    */
#define SMPTE_FPS_25        1           /* Generator set for 25 FPS    */
#define SMPTE_FPS_30        2           /* Generator set for 30 FPS    */
#define SMPTE_FPS_30D       3           /* Set for 30 FPS drop frame   */

#define SMPTE_FPS_MASK      0x03
#define SMPTE_CONF_FPS(x)   (((x) & SMPTE_FPS_MASK))

/*** Macros & Function Prototypes ******************************************/

/* End-Of-File */
