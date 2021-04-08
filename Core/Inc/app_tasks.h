/*
 * app_tasks.h
 *
 *  Created on: 2021. Apr 8.
 *      Author: Robert_Klajko
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

#include "main.h"

#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#define STANDARD_TASK_STACK_SIZE 	( ( configSTACK_DEPTH_TYPE ) 128)
#define PRIO_ButtonTestSignal 		( ( UBaseType_t ) 10 )
#define Signal_Port 				( GPIOC )
#define Signal_Pin  				( GPIO_PIN_1 ) // CN8 A4 pin


void Create_App_Tasks(void);

void xTaskButtonTestSignal(void *pvParameters);

#endif /* INC_APP_TASKS_H_ */
