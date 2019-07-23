/*
 * @file	config.c
 * @date	10 Jan. 2019
 * @author	Santiago Duque
 * @brief	Module of configuration
 */

/*==================================================================*/
/*			  					Includes							*/
/*==================================================================*/
	#include <bridgeUsbSerial.h>
	#include "board.h"
	#include <string.h>

	/* Virtual COM */
	#include "usb.h"
	#include "usb_device.h"
	#include "virtual_com.h"


/*==================================================================*/
/*			  			Constants, local defines 					*/
/*==================================================================*/

	#define COMMAND_SIZE(cmd)	(sizeof(cmd)-1)



	/**
	 * Data type to make a "dispatch table" for the configuration.
	 */
	typedef struct
	{
		const char *command;
		const int cmdSize;
		void (*commandHandler)(char*);
	} COMMAND_T;


/*==================================================================*/
/*			  					Variables 							*/
/*==================================================================*/

	static uint8_t rxBuffer[60];	// Received data
	static uint8_t rxLength = 0;	// Length of the received data
	static TimerHandle_t rxTimeout;	// To handle the timeout for the received data
	static SemaphoreHandle_t rxMutex;	//To handle the rxBuffer in a safe way
	static SemaphoreHandle_t rxSemphr;	//To process the received data

	static usb_device_handle usbhandle;

	uint8_t changeFlags = 0;
	char bufMsg[512]; //Must be greater than or equal to SIZE_VAR

/*==================================================================*/
/*			  			Prototype of local functions				*/
/*==================================================================*/


/*==================================================================*/
/*			  			Prototype of local functions				*/
/*==================================================================*/
/*==================================================================*/
/*			  			Prototype of local functions				*/
/*==================================================================*/

	/**
	 * Task that waits data to process
	 */
	void processDataTsk(void *);

	/**
	 * Callback function for timeout events
	 */
	static void rxTimeoutCallback(TimerHandle_t);

	static inline int executeCommand(char *command);
	static inline void echo(char *msg);

/*==================================================================*/
/*			  				Body of the functions					*/
/*==================================================================*/

	void bridgeTask(void *p)
	{
		uint8_t	dataByte;
		BaseType_t stat;

		rxMutex = xSemaphoreCreateMutex();
		configASSERT(rxMutex);

		rxSemphr = xSemaphoreCreateBinary();
		configASSERT(rxSemphr);

		/* One-shot timer */
		rxTimeout = xTimerCreate("rxTimeout", pdMS_TO_TICKS(10),
								 pdFALSE, NULL, rxTimeoutCallback);
		configASSERT(rxTimeout);

		usbhandle = USB_VcomInit();
		configASSERT(usbhandle);

		stat = xTaskCreate( processDataTsk,
						"cnfg_p_data",
						configMINIMAL_STACK_SIZE+100,
						NULL,
						USB_DEVICE_INTERRUPT_PRIORITY, //TODO: set the correct priority
						NULL);
		configASSERT(stat);

		for(;;)
		{

			if(USB_VcomReadBlocking(usbhandle, &dataByte, 1u) == kStatus_USB_Success)
			{
				if(xSemaphoreTake(rxMutex,0) == pdTRUE)
				{
					if(rxLength<sizeof(rxBuffer))
					{
						rxBuffer[rxLength++] = dataByte;
						xTimerReset(rxTimeout,0);
					}
					else
					{
						xTimerStop(rxTimeout,0);
						xSemaphoreGive(rxSemphr);
					}
					xSemaphoreGive(rxMutex); //Always give the taken mutexes
				}
			}
		}
	}


	void processDataTsk(void *p)
	{
		for(;;)
		{
			xSemaphoreTake(rxSemphr,portMAX_DELAY);
			xSemaphoreTake(rxMutex,portMAX_DELAY);

			rxBuffer[rxLength]= '\0'; //Set the null character
			if(!executeCommand((char*)rxBuffer))
				echo((char*)rxBuffer);
			rxLength = 0;
			xSemaphoreGive(rxMutex);
		}
	}

	static void rxTimeoutCallback(TimerHandle_t t)
	{
		xSemaphoreGive(rxSemphr);
	}


	static inline int executeCommand(char *command)
	{
		if(strcmp(command,"-->POWER_ON\r\n") == 0)
		{
			const char msg[]="OK\r\n";
			setPwrPin();
			vTaskDelay(pdMS_TO_TICKS(500));
			clearPwrPin();
			vTaskDelay(pdMS_TO_TICKS(500));
			USB_VcomWriteBlocking(usbhandle, msg, 4u);
			return true;
		}
		return false;
	}

	/**
	 * Waits for the message to start the configuration process.
	 */
	static inline void echo(char *msg)
	{
		USB_VcomWriteBlocking(usbhandle,(uint8_t*) msg, strlen(msg));
	}


/*==================================================================*/
/*			  	 Functions to use the USB API	 					*/
/*==================================================================*/
	void USB0_IRQHandler(void)
	{
	    USB_DeviceKhciIsrFunction(usbhandle);
	    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
	    exception return operation might vector to incorrect interrupt */
	    __DSB();
	}

	void USB_DeviceIsrEnable(void)
	{
		uint8_t irqNumber;
		uint8_t usbDeviceKhciIrq[] = USB_IRQS;
		irqNumber = usbDeviceKhciIrq[kUSB_ControllerKhci0 - kUSB_ControllerKhci0];
	/* Install isr, set priority, and enable IRQ. */
		NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
		EnableIRQ((IRQn_Type)irqNumber);
	}

	void USB_DeviceClockInit(void)
	{
		SystemCoreClockUpdate();
		CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
	}

/*==================================================================*/
/*			  				Undefinitions							*/
/*==================================================================*/
	#undef COMMAND_SIZE
