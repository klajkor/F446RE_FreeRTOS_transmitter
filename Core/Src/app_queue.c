/*
 * app_queue.c
 *
 * Define and create FreeRTOS Queues
 *
 *  Created on: 2021. Apr 7.
 *      Author: Robert_Klajko
 */

#include "app_queue.h"

void Create_Queues(void)
{
	/* Create queue for UART messages */
	UART_Queue_Handle=xQueueCreate( UART_QUEUE_LENGTH, sizeof( messageFrame_t) );
	if (UART_Queue_Handle == 0)
	{
		Error_Handler();
	}
}
