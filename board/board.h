/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    board.h
 * @brief   Board initialization header file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*==================================================================*/
/*			  					Includes							*/
/*==================================================================*/
	#include "fsl_gpio.h"
/*==================================================================*/
/*			  		 Constants, defines and typedef					*/
/*==================================================================*/

	#define USE_UART3	1
	#define USE_UART3_DEBUG 2

	#define UART_TO_USE	USE_UART3

	/**
	 * @brief	The board name
	 */
	#define BOARD_NAME "board"

	#define LED_ON()	0u
	#define LED_OFF()	1u

	#define _LED_STAT_PIN_	1u
	#define _LED_STAT_PORT_	PORTE
	#define _LED_STAT_GPIO_	GPIOE
	#define _LED_STAT_CLK_	kCLOCK_PortE

	#define _PWR_PIN_	10u
	#define _PWR_PORT_	PORTC
	#define _PWR_GPIO_	GPIOC
	#define _PWR_CLK_	kCLOCK_PortC

/*==================================================================*/
/*			  				Extern variables						*/
/*==================================================================*/

/*==================================================================*/
/*			  		  Prototype of global functions 				*/
/*==================================================================*/


	#if defined(__cplusplus)
	extern "C" {
	#endif /* __cplusplus */

	/**
	 * @brief 	Initialize board specific settings.
	 */
	void BOARD_InitDebugConsole(void);

	/**
	 * Initialize board specific hardware.
	 */
	void BOARD_InitHardware(void);

	#define setStatusLed()		GPIO_PinWrite(_LED_STAT_GPIO_, _LED_STAT_PIN_, LED_ON())
	#define clearStatusLed()	GPIO_PinWrite(_LED_STAT_GPIO_, _LED_STAT_PIN_, LED_OFF())
	#define toggleStatusLed()	GPIO_PortToggle(_LED_STAT_GPIO_, 1u<<_LED_STAT_PIN_)

	#define togglePwrPin()	GPIO_TogglePinsOutput(_PWR_GPIO_, 1u<<_LED_PWR_PIN_)
	#define setPwrPin()		GPIO_PinWrite(_PWR_GPIO_, _PWR_PIN_, 1u)
	#define clearPwrPin()	GPIO_PinWrite(_PWR_GPIO_, _PWR_PIN_, 0u)

	#define readResetPin()	GPIO_PinRead(_RESET_PIN_GPIO_, _RESET_PIN_)

	#if defined(__cplusplus)
	}
	#endif /* __cplusplus */

#endif /* _BOARD_H_ */


