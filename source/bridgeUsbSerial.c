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

	#include "fsl_uart_freertos.h"


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

	static uint8_t rxBuffer[512];	// Received data
	static uint8_t rxLength = 0;	// Length of the received data
	static TimerHandle_t rxTimeout;	// To handle the timeout for the received data
	static SemaphoreHandle_t rxMutex;	//To handle the rxBuffer in a safe way
	static SemaphoreHandle_t rxSemphr;	//To process the received data

	static usb_device_handle usbhandle;



	/* Variables to implement the communication with the UART*/
	static TimerHandle_t tUartHand;
	static SemaphoreHandle_t uartMutex;
	static SemaphoreHandle_t uartSemph;

	static uint8_t uartBuffer[512];
	static size_t nBytesUartBuf = 0;
	static uart_rtos_handle_t 	handle;
	static uart_handle_t		t_handle;



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
	static inline void echo(char *msg, size_t len);


	/* Callback function to handle the reception timeout */
	static void timeoutUart(TimerHandle_t t);

	/* Task to handle the data reception from the UART */
	static void rxUartTsk(void *);

	/* Wait for a complete rx frame to send it to the M95 driver */
	static void rxFrameTsk(void *);


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


		stat = xTaskCreate(rxUartTsk, "uartTsk", 200, NULL, 4, NULL); //TODO priority
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
				echo((char*)rxBuffer, rxLength);
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
//			setPwrPin();
			clearPwrPin();
			vTaskDelay(pdMS_TO_TICKS(500));
//			clearPwrPin();
			setPwrPin();
			vTaskDelay(pdMS_TO_TICKS(500));
			return true;
		}
		return false;
	}

	/**
	 * Waits for the message to start the configuration process.
	 */
	static inline void echo(char *msg, size_t len)
	{
		UART_RTOS_Send(&handle, (uint8_t*) msg, len);
	}


	static void timeoutUart(TimerHandle_t t)
	{
		xSemaphoreGive(uartSemph);
	}

	static void rxUartTsk(void *p)
	{
		uint8_t bgBuffer[300];
		uart_rtos_config_t uart_config = {
											.base = UART0,
											.srcclk = CLOCK_GetFreq(UART0_CLK_SRC),
											.baudrate = 115200,
											.parity = kUART_ParityDisabled,
											.stopbits = kUART_OneStopBit,
											.buffer = bgBuffer,
											.buffer_size = sizeof(bgBuffer)
									  	 };
		NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
		uint8_t uartbyte[1];
		size_t n;


		tUartHand = xTimerCreate("uart timer", pdMS_TO_TICKS(20), pdFALSE, NULL, timeoutUart);
		configASSERT(tUartHand);

		uartMutex = xSemaphoreCreateMutex();
		configASSERT(uartMutex);

		uartSemph = xSemaphoreCreateBinary();
		configASSERT(uartSemph);

		if(UART_RTOS_Init(&handle, &t_handle, &uart_config) != 0)
			vTaskSuspend(NULL);

		if(xTaskCreate(rxFrameTsk, "rxFrameTsk", configMINIMAL_STACK_SIZE, NULL, 3, NULL) != pdPASS)
			vTaskSuspend(NULL);

		for(;;)
		{
			//TODO Avoid data corruption
			if(UART_RTOS_Receive(&handle, uartbyte, 1, &n) == kStatus_Success)
			{
				if(xSemaphoreTake(uartMutex,0) == pdTRUE)
				{
					if(nBytesUartBuf<sizeof(uartBuffer) - 1u)
					{
						uartBuffer[nBytesUartBuf++] = *uartbyte;
						xTimerReset(tUartHand,0);
					}
					else
					{
						xTimerStop(tUartHand,0);
						xSemaphoreGive(uartSemph);
					}

					xSemaphoreGive(uartMutex); //Always give the taken mutexes
				}
			}
		}

	}

	static void rxFrameTsk(void *p)
	{
		for(;;)
		{
			xSemaphoreTake(uartSemph, portMAX_DELAY);
			xSemaphoreTake(uartMutex,portMAX_DELAY);
			USB_VcomWriteBlocking(usbhandle, uartBuffer, nBytesUartBuf);
			nBytesUartBuf = 0;

			xSemaphoreGive(uartMutex);
		}
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
