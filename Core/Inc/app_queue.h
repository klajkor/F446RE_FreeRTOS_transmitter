/*
 * app_queue.h
 *
 * Define and create FreeRTOS Queues
 *
 *  Created on: 2021. Apr 7.
 *      Author: Robert_Klajko
 */

#ifndef INC_APP_QUEUE_H_
#define INC_APP_QUEUE_H_

#include "main.h"
#include "main_defs.h"

#include <stdio.h>
#include <stdint.h>

#include "protocol.h"
#include "FreeRTOS.h"
#include "queue.h"

#define UART_QUEUE_LENGTH			( ( UBaseType_t ) 20 )

QueueHandle_t UART_Queue_Handle;

void Create_Queues(void);

#endif /* INC_APP_QUEUE_H_ */
