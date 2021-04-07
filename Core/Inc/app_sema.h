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

SemaphoreHandle_t UART_Mutex_Handle;

void Create_Mutexes(void);

#endif /* INC_APP_SEMA_H_ */
