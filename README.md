# STM32F4xx FreeRTOS transmitter demo
Demonstration of different FreeRTOS capabilities via a simple code example

## Business requirements
* Read the state of a button (CN8 A5 pin, GPIO port C, pin 0)
* Switch on the LED if button is pressed
* Switch off the LED if button is released
* Send the button state info to host pc via UART
* Measure a voltage source connected to analog input (CN8 A0 pin) every 200 ms
* Send the measurements to host via UART

## Solution Design - Tasks
* Button state read
* ADC voltage read
* LED switcher
* Transmit UART message
* Button test signal (helper task)

## Solution Design – Inter-task Communication
* Button -> LED: Semaphore
* Button -> UART: Semaphore
* ADC -> UART: Queue

## Solution Design – Task Priorities
* Button state read - high
* Transmit UART message - above normal
* ADC voltage read - normal
* LED switcher - below normal
* Button test signal - below normal


## Technical details
* Board: STM32 Nucleo-F446RE 
* Toolchain: STM32CubeIDE Version: 1.5.1 (C) 2020 STMicroelectronics ALL RIGHTS RESERVED
* Middleware: FreeRTOS v10.2.1 CMSIS v2 Copyright (C) Amazon Web Services, Inc. or its affiliates. All rights reserved.

