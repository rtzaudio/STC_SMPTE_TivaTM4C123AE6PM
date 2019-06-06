/*
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
 */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "STC_SMPTE_TivaTM4C123AE6PMI.h"

#define Board_initGeneral           STC_SMPTE_initGeneral
#define Board_initGPIO              STC_SMPTE_initGPIO
#define Board_initSPI               STC_SMPTE_initSPI
#define Board_initUART              STC_SMPTE_initUART
#define Board_initWatchdog          STC_SMPTE_initWatchdog

#define Board_SPI0                  STC_SMPTE_SPI0		// SSI-0 : SPI SUPPLY & TAKEUP SERVO DAC
#define Board_SPI1                  STC_SMPTE_SPI1		// SSI-1 : SPI MCP23S17SO TRANSPORT SWITCHES & LAMPS
#define Board_SPI2                  STC_SMPTE_SPI2		// SSI-2 : SPI MCP23S17SO SOLENOID, CONFIG DIP SWITCH & TAPE OUT

#define Board_UART_IPC              STC_SMPTE_UART0		// RS-232 IPC on edge connector to timer/counter board
#define Board_UART_TTY              STC_SMPTE_UART1		// RS-232 for TTY terminal console
#define Board_UART_EXP              STC_SMPTE_UART5		// RS-232 on expansion port connector P1

#define Board_WATCHDOG0             STC_SMPTE_WATCHDOG0

#define Board_LED_ON                STC_SMPTE_LED_ON
#define Board_LED_OFF               STC_SMPTE_LED_OFF

/* GPIO Pin Mappings */
#define Board_CS_SPI0				STC_SMPTE_SSI0FSS				/* PA3 */


#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
