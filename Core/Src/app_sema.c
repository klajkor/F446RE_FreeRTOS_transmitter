/*
 * app_sema.c
 *
 * Define and create FreeRTOS Semaphores and Mutexes
 *
 *  Created on: 2021. Apr 7.
 *      Author: Robert_Klajko
 */
#include "app_sema.h"

void Create_Mutexes(void)
{
	/* Create Mutex for Thread Safe UART transmit */
	UART_Mutex_Handle=xSemaphoreCreateMutex();
	if (UART_Mutex_Handle == 0)
	{
		Error_Handler();
	}
}

