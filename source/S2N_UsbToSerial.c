/**
 * @file    S2N_UsbToSerial.c
 * @brief   Application entry point.
 */


/*==================================================================*/
/*			  					Includes							*/
/*==================================================================*/

	#include <stdio.h>
	#include "board.h"
	#include "peripherals.h"
	#include "pin_mux.h"
	#include "clock_config.h"
	#include "MK22F51212.h"
	#include "fsl_debug_console.h"

	#include "FreeRTOS.h"
	#include "task.h"

	#include <bridgeUsbSerial.h>


/*==================================================================*/
/*			  			Constants, local defines 					*/
/*==================================================================*/


/*==================================================================*/
/*			  					Variables 							*/
/*==================================================================*/


/*==================================================================*/
/*			  			Prototype of local functions				*/
/*==================================================================*/

	/**
	 * Toggle the status LED
	 */
	void ledTsk(void*);


/*==================================================================*/
/*			  				Body of the functions					*/
/*==================================================================*/

	/*
	 * @brief   Application entry point.
	 */
	int main(void)
	{
		BaseType_t status;

		/* Init board hardware. */
		BOARD_InitBootPins();
		BOARD_InitBootClocks();
		BOARD_InitBootPeripherals();

		/* Init FSL debug console. */
		BOARD_InitDebugConsole();
		BOARD_InitHardware();

		status = xTaskCreate(bridgeTask, "usbSerialTsk", configMINIMAL_STACK_SIZE + 60, NULL, 4, NULL);
		configASSERT(status);
		status = xTaskCreate(ledTsk, "ledTsk", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
		configASSERT(status);

		vTaskStartScheduler();
		for(;;)
		{

		}
		return 0 ;
	}

	void ledTsk(void*p)
	{
		for(;;)
		{
			toggleStatusLed();
			vTaskDelay(pdMS_TO_TICKS(500));
		}
	}
