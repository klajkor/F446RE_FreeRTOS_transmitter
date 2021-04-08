/*
 * app_sema.h
 *
 * Define and create FreeRTOS Semaphores and Mutexes
 *
 *  Created on: 2021. Apr 7.
 *      Author: Robert_Klajko
 */

#ifndef INC_APP_SEMA_H_
#define INC_APP_SEMA_H_

#include "main.h"

#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

#define SEMAPHORE_TAKE_WAIT	( ( TickType_t ) 10 )

SemaphoreHandle_t UART_Mutex_Handle;
SemaphoreHandle_t xButtonBinarySemaphore;

void Create_App_Mutexes(void);

void Create_App_Semaphores(void);

#endif /* INC_APP_SEMA_H_ */
