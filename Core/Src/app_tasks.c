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
TaskHandle_t xHandle_buttonRead 		= NULL;

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

}
