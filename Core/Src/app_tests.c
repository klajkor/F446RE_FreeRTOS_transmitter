/*
 * app_tests.c
 *
 *  Created on: Apr 12, 2021
 *      Author: Robert_Klajko
 */

#include "app_tests.h"

void testADC1(ADC_HandleTypeDef Test_ADC_Handle, UART_HandleTypeDef Test_uart)
{
	uint8_t txt[TXT_BUFFER_LENGTH];
	uint8_t testrun;
	uint8_t i;
	uint32_t raw=0;
	int sizeof_txt;

	uint32_t milliVolt=0;

	sizeof_txt = sprintf((char *)txt, "ADC TEST loop started");
	HAL_UART_Transmit(&Test_uart, txt,  sizeof_txt, UART_TRANSMIT_MAX_DELAY);
	HAL_UART_Transmit(&Test_uart, crlf, sizeof_crlf, UART_TRANSMIT_MAX_DELAY);
	HAL_ADC_Start(&Test_ADC_Handle);
	for(testrun=1;testrun<=10;testrun++)
	{
		//HAL_ADC_Start(&Test_ADC_Handle);
		raw=0;
		for(i=0;i<4;i++)
		{
			if (HAL_ADC_PollForConversion(&Test_ADC_Handle, 10) == HAL_OK)
			{
				sizeof_txt = sprintf((char *)txt, "(%u) ADC Test Poll OK - %u", testrun, i);
				HAL_UART_Transmit(&Test_uart, txt,  sizeof_txt, UART_TRANSMIT_MAX_DELAY);
				HAL_UART_Transmit(&Test_uart, crlf, sizeof_crlf, UART_TRANSMIT_MAX_DELAY);
				raw = raw + HAL_ADC_GetValue(&Test_ADC_Handle);

			}
			else
			{
				sizeof_txt = sprintf((char *)txt, "(%u) ADC Test Poll ERROR - %u", testrun, i);
				HAL_UART_Transmit(&Test_uart, txt,  sizeof_txt, UART_TRANSMIT_MAX_DELAY);
				HAL_UART_Transmit(&Test_uart, crlf, sizeof_crlf, UART_TRANSMIT_MAX_DELAY);
				i--;
			}
			HAL_Delay(1);

		}
		raw = raw >> 2;
		sizeof_txt = sprintf((char *)txt, "(%u) raw: %lu", testrun, raw);
		HAL_UART_Transmit(&Test_uart, txt,  sizeof_txt, UART_TRANSMIT_MAX_DELAY);
		HAL_UART_Transmit(&Test_uart, crlf, sizeof_crlf, UART_TRANSMIT_MAX_DELAY);
		milliVolt=(uint32_t)((ADC_VOLTAGE_REF_MILLIVOLT * raw)/ADC_RESOLUTION);
		sizeof_txt = sprintf((char *)txt, "(%u) %lu mV", testrun, milliVolt);
		HAL_UART_Transmit(&Test_uart, txt,  sizeof_txt, UART_TRANSMIT_MAX_DELAY);
		HAL_UART_Transmit(&Test_uart, crlf, sizeof_crlf, UART_TRANSMIT_MAX_DELAY);
		//HAL_ADC_Stop(&Test_ADC_Handle);
		HAL_Delay(500);
	}
	HAL_ADC_Stop(&Test_ADC_Handle);
	sizeof_txt = sprintf((char *)txt, "ADC TEST loop completed");
	HAL_UART_Transmit(&Test_uart, txt,  sizeof_txt, UART_TRANSMIT_MAX_DELAY);
	HAL_UART_Transmit(&Test_uart, crlf, sizeof_crlf, UART_TRANSMIT_MAX_DELAY);
}

