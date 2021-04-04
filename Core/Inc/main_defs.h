/*
 * main_defs.h
 *
 *  Created on: 2021 Apr 04
 *      Author: Robert_Klajko
 */

#ifndef INC_MAIN_DEFS_H_
#define INC_MAIN_DEFS_H_

#define Button_Port 				( GPIOC )
#define Button_Pin  				( GPIO_PIN_0 ) // CN8 A5 pin
#define Signal_Port 				( GPIOC )
#define Signal_Pin  				( GPIO_PIN_1 ) // CN8 A4 pin
#define Signal_High_Duration 		( 241U ) // in millisec
#define Signal_Low_Duration 		( 33U ) // in millisec
#define Button_Debounce_Delay 		( 80U ) // in millisec
#define ADC_CH1_Port 				( GPIOA )
#define ADC_CH1_Pin  				( GPIO_PIN_0 ) // CN8 A0 pin
#define ADC_Voltage_Poll_Delay 		( 2000U ) // in millisec
#define INCLUDE_RAW_ADC_IN_MESSAGE 	( 0 )
#define QUEUE_SEND_WAIT 			( 10U )
#define QUEUE_REC_WAIT 				( 10U )

//#define DAC_Port GPIOA
//#define DAC_Pin  GPIO_PIN_4 // CN8 A2 pin

#define UART_QUEUE_LENGTH			( ( UBaseType_t ) 20 )
#define UART_TRANSMIT_MAX_DELAY		( 2000U ) // in millisec

#define TXT_BUFFER_LENGTH			( 32U )
#define DATA_BUFFER_LENGTH			( 32U )

#define ADC_VOLTAGE_REF_MILLIVOLT	( ( uint16_t) 3300 ) // in mV
#define ADC_RESOLUTION				( ( uint16_t) 0xFFF ) // 12 bit ADC

#endif /* INC_MAIN_DEFS_H_ */
