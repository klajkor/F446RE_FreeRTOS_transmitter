/*
 * app_tasks.h
 *
 *  Created on: 2021. Apr 8.
 *      Author: Robert_Klajko
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

#include "main.h"
#include "main_defs.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "protocol.h"
#include "app_sema.h"
#include "app_queue.h"

#define STANDARD_TASK_STACK_SIZE 	( ( configSTACK_DEPTH_TYPE ) 128 * 2)
#define TX_UART_MSG_STACK_SIZE 		( ( configSTACK_DEPTH_TYPE ) 128 * 4)

#define PRIO_ButtonTestSignal 		( ( UBaseType_t ) 8 )  // Lowest priority task
#define PRIO_LEDswitcher 			( ( UBaseType_t ) 16 )
#define PRIO_ADCvoltageRead			( ( UBaseType_t ) 32 )
#define PRIO_TX_UART_msg			( ( UBaseType_t ) 40 )
#define PRIO_ButtonRead				( ( UBaseType_t ) 48 )  // Highest priority task
#define Signal_Port 				( GPIOC )
#define Signal_Pin  				( GPIO_PIN_1 ) // CN8 A4 pin


void Create_App_Tasks(void);
void Set_ADC_Handle(ADC_HandleTypeDef new_ADC_Handle);
void xTaskButtonTestSignal(void *pvParameters);
void xTaskLEDswitcher(void *pvParameters);
void xTaskADCvoltageRead(void *pvParameters);
void xTaskTX_UART_msg(void *pvParameters);
void xTaskButtonRead(void *pvParameters);

#endif /* INC_APP_TASKS_H_ */
