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

void Create_Semaphores(void)
{
	/* Create Binary Semaphore for Push Button */
	xButtonBinarySemaphore = xSemaphoreCreateBinary();
	if( xButtonBinarySemaphore == NULL )
	{
		/* There was insufficient FreeRTOS heap available for the semaphore to
	        be created successfully. */
		Error_Handler();
	}
	xSemaphoreGive(xButtonBinarySemaphore);
	if (uxSemaphoreGetCount(xButtonBinarySemaphore) !=1 )
	{
		Error_Handler();
	}
}
