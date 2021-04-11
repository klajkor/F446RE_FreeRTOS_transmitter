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

#define STANDARD_TASK_STACK_SIZE 	( ( configSTACK_DEPTH_TYPE ) 128)
#define PRIO_ButtonTestSignal 		( ( UBaseType_t ) 10 )
#define PRIO_LEDswitcher 			( ( UBaseType_t ) 20 )
#define PRIO_ADCvoltageRead			( ( UBaseType_t ) 30 )
#define PRIO_TX_UART_msg			( ( UBaseType_t ) 40 )
#define PRIO_buttonRead				( ( UBaseType_t ) 50 )
#define Signal_Port 				( GPIOC )
#define Signal_Pin  				( GPIO_PIN_1 ) // CN8 A4 pin


void Create_App_Tasks(void);
void Set_ADC_Handle(ADC_HandleTypeDef new_ADC_Handle);
void xTaskButtonTestSignal(void *pvParameters);
void xTaskLEDswitcher(void *pvParameters);
void xTaskADCvoltageRead(void *pvParameters);

#endif /* INC_APP_TASKS_H_ */
