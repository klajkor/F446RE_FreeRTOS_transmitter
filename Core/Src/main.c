/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "crc.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Button_Port GPIOC
#define Button_Pin  GPIO_PIN_0 // CN8 A5 pin
#define Signal_Port GPIOC
#define Signal_Pin  GPIO_PIN_1 // CN8 A4 pin
#define Signal_High_Duration 241U // in millisec
#define Signal_Low_Duration 33U // in millisec
#define Button_Debounce_Delay 80U  // in millisec
#define ADC_CH1_Port GPIOA
#define ADC_CH1_Pin  GPIO_PIN_0 // CN8 A0 pin
#define ADC_Voltage_Poll_Delay 7531U // in millisec
#define INCLUDE_RAW_ADC_IN_MESSAGE 0
#define QUEUE_SEND_WAIT 10
#define QUEUE_REC_WAIT 10

//#define DAC_Port GPIOA
//#define DAC_Pin  GPIO_PIN_4 // CN8 A2 pin


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for buttonRead */
osThreadId_t buttonReadHandle;
const osThreadAttr_t buttonRead_attributes = {
		.name = "buttonRead",
		.priority = (osPriority_t) osPriorityHigh,
		.stack_size = 128 * 4
};
/* Definitions for LEDswitcher */
osThreadId_t LEDswitcherHandle;
const osThreadAttr_t LEDswitcher_attributes = {
		.name = "LEDswitcher",
		.priority = (osPriority_t) osPriorityBelowNormal,
		.stack_size = 128 * 4
};
/* Definitions for transmitUARTmes */
osThreadId_t transmitUARTmesHandle;
const osThreadAttr_t transmitUARTmes_attributes = {
		.name = "transmitUARTmes",
		.priority = (osPriority_t) osPriorityAboveNormal,
		.stack_size = 128 * 4
};
/* Definitions for ADCvoltageRead */
osThreadId_t ADCvoltageReadHandle;
const osThreadAttr_t ADCvoltageRead_attributes = {
		.name = "ADCvoltageRead",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 256 * 4
};
/* Definitions for ButtonTestSigna */
osThreadId_t ButtonTestSignaHandle;
const osThreadAttr_t ButtonTestSigna_attributes = {
		.name = "ButtonTestSigna",
		.priority = (osPriority_t) osPriorityBelowNormal,
		.stack_size = 128 * 4
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
		.name = "myQueue01"
};
/* Definitions for ButtonSemaphore */
osSemaphoreId_t ButtonSemaphoreHandle;
const osSemaphoreAttr_t ButtonSemaphore_attributes = {
		.name = "ButtonSemaphore"
};
/* USER CODE BEGIN PV */
uint16_t size;
uint8_t Data[32];
char crlf[3];
osStatus_t buttonSemaphoreStatus;


QueueHandle_t UART_Queue_Handle;
SemaphoreHandle_t UART_Mutex_Handle;

//unionFloatUint8_t testData;
//messageFrame_t messageFrame;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartButtonRead(void *argument);
void StartLEDswitcher(void *argument);
void StartTransmitUARTmessage(void *argument);
void StartADCvoltageRead(void *argument);
void StartButtonTestSignal(void *argument);

/* USER CODE BEGIN PFP */

void testADC1(void);
BaseType_t xThread_Safe_UART_Transmit(uint8_t *pTransmitData, uint8_t data_size);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	sprintf(crlf, "\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf), HAL_MAX_DELAY);

	/* Create queue for UART messages */
	UART_Queue_Handle=xQueueCreate( 20, sizeof( messageFrame_t) );
	if (UART_Queue_Handle == 0)
	{
		Error_Handler();
	}
	/* Create Mutex for Thread Safe UART transmit */
	UART_Mutex_Handle=xSemaphoreCreateMutex();
	if (UART_Mutex_Handle == 0)
	{
		Error_Handler();
	}

	/* Test ADC conversion before the FreeRTOS kernel starts */
	testADC1();

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of ButtonSemaphore */
	ButtonSemaphoreHandle = osSemaphoreNew(1, 1, &ButtonSemaphore_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	buttonSemaphoreStatus = osSemaphoreAcquire (ButtonSemaphoreHandle, 10U);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of myQueue01 */
	myQueue01Handle = osMessageQueueNew (4, sizeof(uint16_t), &myQueue01_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of buttonRead */
	buttonReadHandle = osThreadNew(StartButtonRead, NULL, &buttonRead_attributes);

	/* creation of LEDswitcher */
	LEDswitcherHandle = osThreadNew(StartLEDswitcher, NULL, &LEDswitcher_attributes);

	/* creation of transmitUARTmes */
	transmitUARTmesHandle = osThreadNew(StartTransmitUARTmessage, NULL, &transmitUARTmes_attributes);

	/* creation of ADCvoltageRead */
	ADCvoltageReadHandle = osThreadNew(StartADCvoltageRead, NULL, &ADCvoltageRead_attributes);

	/* creation of ButtonTestSigna */
	ButtonTestSignaHandle = osThreadNew(StartButtonTestSignal, NULL, &ButtonTestSigna_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void testADC1(void)
{
	uint8_t txt[32];
	uint8_t testrun,i;
	uint32_t raw=0;
	int size;
	uint32_t milliVolt=0;

	size = sprintf((char *)txt, "ADC TEST loop started");
	HAL_UART_Transmit(&huart2, txt, size, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf), HAL_MAX_DELAY);
	HAL_ADC_Start(&hadc1);
	for(testrun=1;testrun<=10;testrun++)
	{
		//HAL_ADC_Start(&hadc1);
		raw=0;
		for(i=0;i<4;i++)
		{
			if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
			{
				size = sprintf((char *)txt, "(%u) ADC Test Poll OK - %u", testrun, i);
				HAL_UART_Transmit(&huart2, txt, size, HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf), HAL_MAX_DELAY);
				raw = raw + HAL_ADC_GetValue(&hadc1);

			}
			else
			{
				size = sprintf((char *)txt, "(%u) ADC Test Poll ERROR - %u", testrun, i);
				HAL_UART_Transmit(&huart2, txt, size, HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf), HAL_MAX_DELAY);
				i--;
			}
			HAL_Delay(1);

		}
		raw = raw >> 2;
		size = sprintf((char *)txt, "(%u) raw: %lu", testrun, raw);
		HAL_UART_Transmit(&huart2, (uint8_t*)txt, size, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf), HAL_MAX_DELAY);
		milliVolt=(uint32_t)((3300*raw)/4095);
		size = sprintf((char *)txt, "(%u) %lu mV", testrun, milliVolt);
		HAL_UART_Transmit(&huart2, (uint8_t*)txt, size, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf), HAL_MAX_DELAY);
		//HAL_ADC_Stop(&hadc1);
		HAL_Delay(500);
	}
	HAL_ADC_Stop(&hadc1);
	size = sprintf((char *)txt, "ADC TEST loop completed");
	HAL_UART_Transmit(&huart2, txt, size, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)crlf, sizeof(crlf), HAL_MAX_DELAY);
}

BaseType_t xThread_Safe_UART_Transmit(uint8_t *pTransmitData, uint8_t data_size)
{
	BaseType_t Return_Value;
	if ((pTransmitData == NULL) || (data_size == 0U))
	{
		return  pdFAIL;
	}
	if (xSemaphoreTake(UART_Mutex_Handle, portMAX_DELAY) == pdTRUE)
	{
		if(HAL_UART_Transmit(&huart2, pTransmitData, data_size, HAL_MAX_DELAY) == HAL_OK)
		{
			Return_Value=pdPASS;
		}
		else
		{
			Return_Value=pdFAIL;
		}
		xSemaphoreGive(UART_Mutex_Handle);
	}
	else
	{
		Return_Value=pdFAIL;
	}
	return Return_Value;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartButtonRead */
/**
 * @brief  Function implementing the buttonRead thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButtonRead */
void StartButtonRead(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	uint32_t semaCount_b;
	unionFloatUint8_t BtnData;
	messageFrame_t BtnMessageFrame;

	for(;;)
	{
		semaCount_b = osSemaphoreGetCount(ButtonSemaphoreHandle);
		if(HAL_GPIO_ReadPin(Button_Port, Button_Pin))
		{
			/* Button released */
			if (semaCount_b > 0)
			{
				/* Semaphore not yet acquired */
				buttonSemaphoreStatus = osSemaphoreAcquire(ButtonSemaphoreHandle, 10U);
				semaCount_b = osSemaphoreGetCount(ButtonSemaphoreHandle);
				memset((char *)BtnData.u8, 0, sizeof(BtnData.u8));
				BtnData.u8[0]=FRAME_STATUS_OFF;
				buildFrameToSend(FRAME_CMD_BTN_STATUS, BtnData, BtnMessageFrame);
				if(pdTRUE == xQueueSend(UART_Queue_Handle, &BtnMessageFrame, QUEUE_SEND_WAIT))
				{
					osDelay(0);
				}
				else
				{
					sprintf((char *)Data, "B Q0 Send ERR");
					xThread_Safe_UART_Transmit(Data, strlen((char *)Data));
					xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
				}
			}
		}
		else
		{
			/* Button pushed */
			if (semaCount_b == 0)
			{
				/* Semaphore not yet released */
				buttonSemaphoreStatus = osSemaphoreRelease(ButtonSemaphoreHandle);
				semaCount_b = osSemaphoreGetCount(ButtonSemaphoreHandle);
				memset((char *)BtnData.u8, 0, sizeof(BtnData.u8));
				BtnData.u8[0]=FRAME_STATUS_ON;
				buildFrameToSend(FRAME_CMD_BTN_STATUS, BtnData, BtnMessageFrame);
				if(pdTRUE == xQueueSend(UART_Queue_Handle, &BtnMessageFrame, QUEUE_SEND_WAIT))
				{
					osDelay(0);
				}
				else
				{
					sprintf((char *)Data, "B Q1 Send ERR");
					xThread_Safe_UART_Transmit(Data, strlen((char *)Data));
					xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
				}
			}

		}
		osDelay(Button_Debounce_Delay);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLEDswitcher */
/**
 * @brief Function implementing the LEDswitcher thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDswitcher */
void StartLEDswitcher(void *argument)
{
	/* USER CODE BEGIN StartLEDswitcher */
	/* Infinite loop */
	uint32_t semaCount, prevCount;
	unionFloatUint8_t LedData;
	messageFrame_t LedMessageFrame;
	semaCount = osSemaphoreGetCount(ButtonSemaphoreHandle);
	prevCount = semaCount;
	for(;;)
	{
		semaCount = osSemaphoreGetCount(ButtonSemaphoreHandle);
		if (semaCount != prevCount)
		{
			prevCount = semaCount;
			if (semaCount > 0)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
				memset((char *)LedData.u8, 0, sizeof(LedData.u8));
				LedData.u8[0]=FRAME_STATUS_ON;
				buildFrameToSend(FRAME_CMD_LED_STATUS, LedData, LedMessageFrame);
				if(pdTRUE == xQueueSend(UART_Queue_Handle, &LedMessageFrame, QUEUE_SEND_WAIT))
				{
					osDelay(0);
				}
				else
				{
					sprintf((char *)Data, "L Q1 Send ERR");
					xThread_Safe_UART_Transmit(Data, strlen((char *)Data));
					xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
				}
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
				memset((char *)LedData.u8, 0, sizeof(LedData.u8));
				LedData.u8[0]=FRAME_STATUS_OFF;
				buildFrameToSend(FRAME_CMD_LED_STATUS, LedData, LedMessageFrame);
				if(pdTRUE == xQueueSend(UART_Queue_Handle, &LedMessageFrame, QUEUE_SEND_WAIT))
				{
					osDelay(0);
				}
				else
				{
					sprintf((char *)Data, "L Q0 Send ERR");
					xThread_Safe_UART_Transmit(Data, strlen((char *)Data));
					xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
				}
			}
			osDelay(1);
		}
		osDelay(1);
	}
	/* USER CODE END StartLEDswitcher */
}

/* USER CODE BEGIN Header_StartTransmitUARTmessage */
/**
 * @brief Function implementing the transmitUARTmes thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTransmitUARTmessage */
void StartTransmitUARTmessage(void *argument)
{
	/* USER CODE BEGIN StartTransmitUARTmessage */
	/* Infinite loop */
	messageFrame_t receivedMessage;
	for(;;)
	{
		if(xQueueReceive(UART_Queue_Handle, &receivedMessage, ( TickType_t ) QUEUE_REC_WAIT) == pdPASS)
		{
			xThread_Safe_UART_Transmit((uint8_t *)receivedMessage, sizeof(messageFrame_t));
		}
		osDelay(1);
	}
	/* USER CODE END StartTransmitUARTmessage */
}

/* USER CODE BEGIN Header_StartADCvoltageRead */
/**
 * @brief Function implementing the ADCvoltageRead thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADCvoltageRead */
void StartADCvoltageRead(void *argument)
{
	/* USER CODE BEGIN StartADCvoltageRead */
	/* Infinite loop */
	uint8_t txt[32];
	uint8_t cycle;
	uint32_t raw=0;
	unionFloatUint8_t AdcData;
	messageFrame_t AdcMessageFrame;
	float fMilliVolt=0.0;
	int size;
	size=sprintf((char *)txt, "ADC task started");
	xThread_Safe_UART_Transmit(txt, size);
	xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
	/* ADC in single channel continuous conversion mode, EOC flag at the end of all conversions */
	HAL_ADC_Start(&hadc1);
	for(;;)
	{
		cycle=0;
		raw=0;
		while(cycle<4)
		{
			if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK)
			{
				raw = raw + HAL_ADC_GetValue(&hadc1);
				cycle++;
			}
			else
			{
				size=sprintf((char *)txt, "ADC Poll ERROR - %u", cycle);
				xThread_Safe_UART_Transmit(txt, size);
				xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
			}
			osDelay(1);
		}
		raw=raw>>2;
		if(INCLUDE_RAW_ADC_IN_MESSAGE)
		{
			memset((char *)AdcData.u8, 0, sizeof(AdcData.u8));
			AdcData.f=(float)raw*1.0;
			buildFrameToSend(FRAME_CMD_ADC_RAW, AdcData, AdcMessageFrame);
			if(pdTRUE == xQueueSend(UART_Queue_Handle, &AdcMessageFrame, QUEUE_SEND_WAIT))
			{
				osDelay(1);
			}
			else
			{
				size=sprintf((char *)txt, "ADC Q Send ERROR");
				xThread_Safe_UART_Transmit(txt, size);
				xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
			}
		}
		memset((char *)AdcData.u8, 0, sizeof(AdcData.u8));
		fMilliVolt=(float)((3300*raw)/4095);
		AdcData.f=fMilliVolt;
		buildFrameToSend(FRAME_CMD_ADC_VALUE, AdcData, AdcMessageFrame);
		if(pdTRUE == xQueueSend(UART_Queue_Handle, &AdcMessageFrame, QUEUE_SEND_WAIT))
		{
			osDelay(1);
		}
		else
		{
			size=sprintf((char *)txt, "ADC Q Send ERROR");
			xThread_Safe_UART_Transmit(txt, size);
			xThread_Safe_UART_Transmit((uint8_t*)crlf, strlen(crlf));
		}
		//HAL_ADC_Stop(&hadc1);
		osDelay(ADC_Voltage_Poll_Delay);
	}
	/* USER CODE END StartADCvoltageRead */
}

/* USER CODE BEGIN Header_StartButtonTestSignal */
/**
 * @brief Function implementing the ButtonTestSigna thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButtonTestSignal */
void StartButtonTestSignal(void *argument)
{
	/* USER CODE BEGIN StartButtonTestSignal */
	/* Infinite loop */
	HAL_GPIO_WritePin(Signal_Port, Signal_Pin, GPIO_PIN_SET);
	for(;;)
	{
		HAL_GPIO_WritePin(Signal_Port, Signal_Pin, GPIO_PIN_SET);
		osDelay(Signal_High_Duration);
		HAL_GPIO_WritePin(Signal_Port, Signal_Pin, GPIO_PIN_RESET);
		osDelay(Signal_Low_Duration);
	}
	/* USER CODE END StartButtonTestSignal */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
