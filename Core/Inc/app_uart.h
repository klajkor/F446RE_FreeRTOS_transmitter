/*
 * app_uart.h
 *
 *  Created on: 2021. Apr. 15.
 *      Author: Robert_Klajko
 */

#ifndef INC_APP_UART_H_
#define INC_APP_UART_H_

#include "main.h"
#include "main_defs.h"

#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "app_sema.h"

void Set_TX_UART_Handle(UART_HandleTypeDef *new_UART_Handle);
BaseType_t xThread_Safe_UART_Transmit(uint8_t *pTransmitData, uint8_t data_size, UART_HandleTypeDef *TX_uart);

#endif /* INC_APP_UART_H_ */
