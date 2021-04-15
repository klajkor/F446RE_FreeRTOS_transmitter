/*
 * main_defs.h
 *
 *  Created on: 2021 Apr 04
 *      Author: Robert_Klajko
 */

#ifndef INC_MAIN_DEFS_H_
#define INC_MAIN_DEFS_H_

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"

#define Button_Port 				( GPIOC )
#define Button_Pin  				( GPIO_PIN_0 ) // CN8 A5 pin
//#define Signal_High_Duration 		( 241U ) // in millisec
//#define Signal_Low_Duration 		( 33U ) // in millisec
#define Button_Debounce_Delay 		( 80U ) // in millisec
#define ADC_CH1_Port 				( GPIOA )
#define ADC_CH1_Pin  				( GPIO_PIN_0 ) // CN8 A0 pin
#define ADC_Voltage_Poll_Delay 		( 5000U ) // in millisec
#define INCLUDE_RAW_ADC_IN_MESSAGE 	( 0 )
#define QUEUE_SEND_WAIT 			( 10U )
#define QUEUE_REC_WAIT 				( 10U )

//#define DAC_Port GPIOA
//#define DAC_Pin  GPIO_PIN_4 // CN8 A2 pin

#define UART_TRANSMIT_MAX_DELAY		( 2000U ) // in millisec

#define TXT_BUFFER_LENGTH			( 32U )
#define DATA_BUFFER_LENGTH			( 32U )

#define ADC_VOLTAGE_REF_MILLIVOLT	( ( uint16_t) 3300 ) // in mV
#define ADC_RESOLUTION				( ( uint16_t) 0xFFF ) // 12 bit ADC

uint16_t size;
uint8_t Data[DATA_BUFFER_LENGTH];
uint8_t crlf[3];
int sizeof_crlf;
int sizeof_Data;

UART_HandleTypeDef *UART_Handle_TX_Uart;

#endif /* INC_MAIN_DEFS_H_ */


