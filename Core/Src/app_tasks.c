/*
 * app_tasks.c
 *
 *  Created on: 2021. ápr. 8.
 *      Author: Robert_Klajko
 */

#include "app_tasks.h"

TaskHandle_t xHandle_ButtonTestSignal 	= NULL;
TaskHandle_t xHandle_LEDswitcher 		= NULL;
TaskHandle_t xHandle_ADCvoltageRead 	= NULL;
TaskHandle_t xHandle_TX_UART_msg 		= NULL;
TaskHandle_t xHandle_ButtonRead 		= NULL;

ADC_HandleTypeDef ADC_Handle_ADCvoltageRead;

const TickType_t xSignal_High_Duration = 641 / portTICK_PERIOD_MS;
const TickType_t xSignal_Low_Duration = 45 / portTICK_PERIOD_MS;


void Create_App_Tasks(void)
{
	BaseType_t xReturned;

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			xTaskButtonTestSignal,      /* Function that implements the task. */
			"ButtonTestSignal",         /* Text name for the task. */
			STANDARD_TASK_STACK_SIZE,	/* Stack size in words, not bytes. */
			( void * ) 1,    			/* Parameter passed into the task. */
			PRIO_ButtonTestSignal, 		/* Priority at which the task is created. */
			&xHandle_ButtonTestSignal );/* Used to pass out the created task's handle. */

	if( xReturned != pdPASS )
	{
		Error_Handler();
	}

	xReturned = xTaskCreate(
			xTaskLEDswitcher,
			"xTaskLEDswitcher",
			STANDARD_TASK_STACK_SIZE,
			( void * ) 1,
			PRIO_LEDswitcher,
			&xHandle_LEDswitcher );

	if( xReturned != pdPASS )
	{
		Error_Handler();
	}


	xReturned = xTaskCreate(
			xTaskADCvoltageRead,
			"xTaskADCvoltageRead",
			STANDARD_TASK_STACK_SIZE,
			( void * ) 1,
			PRIO_ADCvoltageRead,
			&xHandle_ADCvoltageRead );
	if( xReturned != pdPASS )
	{
		Error_Handler();
	}

	xReturned = xTaskCreate(
			xTaskTX_UART_msg,
			"xTaskTX_UART_msg",
			TX_UART_MSG_STACK_SIZE,
			( void * ) 1,
			PRIO_TX_UART_msg,
			&xHandle_TX_UART_msg );
	if( xReturned != pdPASS )
	{
		Error_Handler();
	}
	xReturned = xTaskCreate(
			xTaskButtonRead,
			"xTaskButtonRead",
			STANDARD_TASK_STACK_SIZE,
			( void * ) 1,
			PRIO_ButtonRead,
			&xHandle_ButtonRead );
	if( xReturned != pdPASS )
	{
		Error_Handler();
	}
}

void Set_ADC_Handle(ADC_HandleTypeDef new_ADC_Handle)
{
	ADC_Handle_ADCvoltageRead=new_ADC_Handle;
}

void xTaskButtonTestSignal(void *pvParameters)
{
	/* Infinite loop */
	HAL_GPIO_WritePin(Signal_Port, Signal_Pin, GPIO_PIN_SET);
	for(;;)
	{
		HAL_GPIO_WritePin(Signal_Port, Signal_Pin, GPIO_PIN_SET);
		vTaskDelay(xSignal_High_Duration);
		HAL_GPIO_WritePin(Signal_Port, Signal_Pin, GPIO_PIN_RESET);
		vTaskDelay(xSignal_Low_Duration);
	}
}

void xTaskLEDswitcher(void *pvParameters)
{
	uint32_t semaCount_b;
	uint32_t prevCount;
	unionFloatUint8_t LedData;
	messageFrame_t LedMessageFrame;
	semaCount_b = uxSemaphoreGetCount(xButtonBinarySemaphore);
	prevCount = semaCount_b;
	for(;;)
	{
		semaCount_b = uxSemaphoreGetCount(xButtonBinarySemaphore);
		if (semaCount_b != prevCount)
		{
			prevCount = semaCount_b;
			if (semaCount_b > 0)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
				memset((char *)LedData.u8, 0, sizeof(LedData.u8));
				LedData.u8[0]=FRAME_STATUS_ON;
				buildFrameToSend(FRAME_CMD_LED_STATUS, LedData, LedMessageFrame);
				if(pdTRUE == xQueueSend(UART_Queue_Handle, &LedMessageFrame, QUEUE_SEND_WAIT))
				{
					vTaskDelay(0);
				}
				else
				{
					sizeof_Data = sprintf((char *)Data, "L Q1 Send ERR");
					xThread_Safe_UART_Transmit(Data, sizeof_Data);
					xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
				}
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
				memset((char *)LedData.u8, 0, sizeof(LedData.u8));
				LedData.u8[0]=FRAME_STATUS_OFF;
				buildFrameToSend(FRAME_CMD_LED_STATUS, LedData, LedMessageFrame);
				if(pdTRUE == xQueueSend(UART_Queue_Handle, &LedMessageFrame, QUEUE_SEND_WAIT))
				{
					vTaskDelay(0);
				}
				else
				{
					sizeof_Data = sprintf((char *)Data, "L Q0 Send ERR");
					xThread_Safe_UART_Transmit(Data, sizeof_Data);
					xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
				}
			}
			vTaskDelay(1);
		}
		vTaskDelay(1);
	}
}

void xTaskADCvoltageRead(void *pvParameters)
{
	uint8_t txt[TXT_BUFFER_LENGTH];
	uint8_t cycle;
	uint32_t raw=0;
	unionFloatUint8_t AdcData;
	messageFrame_t AdcMessageFrame;
	float fMilliVolt=0.0;
	int size;
	size=sprintf((char *)txt, "ADC task started");
	xThread_Safe_UART_Transmit(txt, size);
	xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
	/* ADC in single channel continuous conversion mode, EOC flag at the end of all conversions */
	HAL_ADC_Start(&ADC_Handle_ADCvoltageRead);
	for(;;)
	{
		cycle=0;
		raw=0;
		while(cycle<4)
		{
			if (HAL_ADC_PollForConversion(&ADC_Handle_ADCvoltageRead, 2) == HAL_OK)
			{
				raw = raw + HAL_ADC_GetValue(&ADC_Handle_ADCvoltageRead);
				cycle++;
			}
			else
			{
				size=sprintf((char *)txt, "ADC Poll ERROR - %u", cycle);
				xThread_Safe_UART_Transmit(txt, size);
				xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
			}
			vTaskDelay(1);
		}
		raw=raw>>2;
		if(INCLUDE_RAW_ADC_IN_MESSAGE)
		{
			memset((char *)AdcData.u8, 0, sizeof(AdcData.u8));
			AdcData.f=(float)raw*1.0;
			buildFrameToSend(FRAME_CMD_ADC_RAW, AdcData, AdcMessageFrame);
			if(pdTRUE == xQueueSend(UART_Queue_Handle, &AdcMessageFrame, QUEUE_SEND_WAIT))
			{
				vTaskDelay(1);
			}
			else
			{
				size=sprintf((char *)txt, "ADC Q Send ERROR");
				xThread_Safe_UART_Transmit(txt, size);
				xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
			}
		}
		memset((char *)AdcData.u8, 0, sizeof(AdcData.u8));
		fMilliVolt=(float)((ADC_VOLTAGE_REF_MILLIVOLT * raw)/ADC_RESOLUTION);
		AdcData.f=fMilliVolt;
		buildFrameToSend(FRAME_CMD_ADC_VALUE, AdcData, AdcMessageFrame);
		if(pdTRUE == xQueueSend(UART_Queue_Handle, &AdcMessageFrame, QUEUE_SEND_WAIT))
		{
			vTaskDelay(1);
		}
		else
		{
			size=sprintf((char *)txt, "ADC Q Send ERROR");
			xThread_Safe_UART_Transmit(txt, size);
			xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
		}
		//HAL_ADC_Stop(&ADC_Handle_ADCvoltageRead);
		vTaskDelay(ADC_Voltage_Poll_Delay);
	}
}

void xTaskTX_UART_msg(void *pvParameters)
{
	messageFrame_t receivedMessage;
	for(;;)
	{
		if(xQueueReceive(UART_Queue_Handle, &receivedMessage, ( TickType_t ) QUEUE_REC_WAIT) == pdPASS)
		{
			xThread_Safe_UART_Transmit((uint8_t *)receivedMessage, sizeof(messageFrame_t));
		}
		vTaskDelay(1);
	}
}

void xTaskButtonRead(void *pvParameters)
{
	uint32_t semaCount_b;
	unionFloatUint8_t BtnData;
	messageFrame_t BtnMessageFrame;

	for(;;)
	{
		semaCount_b = uxSemaphoreGetCount(xButtonBinarySemaphore);
		if(HAL_GPIO_ReadPin(Button_Port, Button_Pin))
		{
			/* Button released */
			if (semaCount_b > 0)
			{
				/* Semaphore not yet acquired */
				if( xSemaphoreTake( xButtonBinarySemaphore, SEMAPHORE_TAKE_WAIT ) == pdTRUE )
				{
					semaCount_b = uxSemaphoreGetCount(xButtonBinarySemaphore);
					memset((char *)BtnData.u8, 0, sizeof(BtnData.u8));
					BtnData.u8[0]=FRAME_STATUS_OFF;
					buildFrameToSend(FRAME_CMD_BTN_STATUS, BtnData, BtnMessageFrame);
					if(pdTRUE == xQueueSend(UART_Queue_Handle, &BtnMessageFrame, QUEUE_SEND_WAIT))
					{
						vTaskDelay(0);
					}
					else
					{
						sizeof_Data = sprintf((char *)Data, "B Q0 Send ERR");
						xThread_Safe_UART_Transmit(Data, sizeof_Data);
						xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
					}
				}
			}
		}
		else
		{
			/* Button pushed */
			if (semaCount_b == 0)
			{
				/* Semaphore not yet released */
				if( xSemaphoreGive( xButtonBinarySemaphore ) == pdTRUE )
				{
					semaCount_b = uxSemaphoreGetCount(xButtonBinarySemaphore);
					memset((char *)BtnData.u8, 0, sizeof(BtnData.u8));
					BtnData.u8[0]=FRAME_STATUS_ON;
					buildFrameToSend(FRAME_CMD_BTN_STATUS, BtnData, BtnMessageFrame);
					if(pdTRUE == xQueueSend(UART_Queue_Handle, &BtnMessageFrame, QUEUE_SEND_WAIT))
					{
						vTaskDelay(0);
					}
					else
					{
						sizeof_Data = sprintf((char *)Data, "B Q1 Send ERR");
						xThread_Safe_UART_Transmit(Data, sizeof_Data);
						xThread_Safe_UART_Transmit(crlf, sizeof_crlf);
					}
				}
			}

		}
		vTaskDelay(Button_Debounce_Delay);
	}
}
