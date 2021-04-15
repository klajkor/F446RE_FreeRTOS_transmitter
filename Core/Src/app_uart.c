/*
 * app_uart.c
 *
 *  Created on: 2021. Apr. 15.
 *      Author: Robert_Klajko
 */

#include "app_uart.h"


void Set_TX_UART_Handle(UART_HandleTypeDef *new_UART_Handle)
{
	UART_Handle_TX_Uart=new_UART_Handle;
}

BaseType_t xThread_Safe_UART_Transmit(uint8_t *pTransmitData, uint8_t data_size, UART_HandleTypeDef *TX_Uart)
{
	BaseType_t Return_Value;
	if ( NULL == pTransmitData || data_size == 0U )
	{
		return  pdFAIL;
	}
	if (xSemaphoreTake(UART_Mutex_Handle, portMAX_DELAY) == pdTRUE)
	{
		if(HAL_UART_Transmit(TX_Uart, pTransmitData, data_size, UART_TRANSMIT_MAX_DELAY) == HAL_OK)
		{
			Return_Value=pdPASS;
		}
		else
		{
			Return_Value=pdFAIL;
		}
		xSemaphoreGive(UART_Mutex_Handle);
	}
	else
	{
		Return_Value=pdFAIL;
	}
	return Return_Value;
}


