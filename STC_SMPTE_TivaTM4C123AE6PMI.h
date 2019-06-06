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
/** ============================================================================
 *  @file       STC_SMPTE.h
 *
 *  @brief      STC_SMPTE Board Specific APIs
 *
 *  The STC_SMPTE header file should be included in an application as follows:
 *  @code
 *  #include <STC_SMPTE.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __STC_SMPTE_TM4C123AE6PMI_H
#define __STC_SMPTE_TM4C123AE6PMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on STC_SMPTE are active high. */
#define STC_SMPTE_LED_OFF   ( 0)
#define STC_SMPTE_LED_ON    (~0)

#define PIN_LOW			    ( 0)
#define PIN_HIGH		    (~0)

/*** STC-SMPTE HARDWARE CONSTANTS *********************************************/


/*******************************************************************************
 * Functions and Constants
 ******************************************************************************/

/*!
 *  @def    STC_SMPTE_GPIOName
 *  @brief  Enum of LED names on the STC_SMPTE dev board
 */
typedef enum STC_SMPTE_GPIOName {
    STC_SMPTE_BOOTLOAD = 0,         /* 0 = PA7 */
    STC_SMPTE_IN,                   /* 1 = PC4 */
	STC_SMPTE_INT_N,	            /* 2 = PA6 */
    STC_SMPTE_OUT,                  /* 3 = PB6 */
    STC_SMPTE_MUTE,                 /* 4 = PB7 */
	STC_SMPTE_STAT_LED,             /* 5 = PC7 */
	STC_SMPTE_SLOWCODE,             /* 6 = PE0 */
    STC_SMPTE_FRAMESYNC,            /* 7 = PE1 */
    STC_SMPTE_DIRECTION,            /* 8 = PE2 */
    STC_SMPTE_CHANGE,               /* 9 = PE3 */

    STC_SMPTE_GPIOCOUNT
} STC_SMPTE_GPIOName;

#if 0
/*!
 *  @def    STC_SMPTE_I2CName
 *  @brief  Enum of I2C names on the STC_SMPTE dev board
 */
typedef enum STC_SMPTE_I2CName {
    STC_SMPTE_I2C0 = 0,

    STC_SMPTE_I2CCOUNT
} STC_SMPTE_I2CName;

/*!
 *  @def    STC_SMPTE_PWMName
 *  @brief  Enum of PWM names on the STC_SMPTE dev board
 */
typedef enum STC_SMPTE_PWMName {
    STC_SMPTE_PWM0 = 0,

    STC_SMPTE_PWMCOUNT
} STC_SMPTE_PWMName;
#endif

/*!
 *  @def    STC_SMPTE_SPIName
 *  @brief  Enum of SPI names on the STC_SMPTE dev board
 */
typedef enum STC_SMPTE_SPIName {
    STC_SMPTE_SPI0 = 0,		/* SUPPLY & TAKEUP SERVO DAC */

    STC_SMPTE_SPICOUNT
} STC_SMPTE_SPIName;

/*!
 *  @def    STC_SMPTE_UARTName
 *  @brief  Enum of UARTs on the STC_SMPTE dev board
 */
typedef enum STC_SMPTE_UARTName {
    STC_SMPTE_UART0 = 0,		    /* debug UART to pin header */

    STC_SMPTE_UARTCOUNT
} STC_SMPTE_UARTName;

/*
 *  @def    STC_SMPTE_WatchdogName
 *  @brief  Enum of Watchdogs on the STC_SMPTE dev board
 */
typedef enum STC_SMPTE_WatchdogName {
    STC_SMPTE_WATCHDOG0 = 0,

    STC_SMPTE_WATCHDOGCOUNT
} STC_SMPTE_WatchdogName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 */
extern void STC_SMPTE_initGeneral(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern void STC_SMPTE_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void STC_SMPTE_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void STC_SMPTE_initPWM(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void STC_SMPTE_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void STC_SMPTE_initUART(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void STC_SMPTE_initWatchdog(void);

#ifdef __cplusplus
}
#endif

#endif /* __STC_SMPTE_TM4C123AE6PMI_H */
