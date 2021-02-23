# STM32F4xx FreeRTOS transmitter demo
Demonstration of different FreeRTOS capabilities via a simple code example

## Business requirements
* Read the state of a button
* Switch on the LED if button is pressed
* Send the button state info to host pc via UART - TBD
* Measure a voltage source connected to analog input - TBD
* Send the measurements to host via UART - TBD

## Solution Design - Tasks
* Button state read
* ADC voltage read
* LED switcher
* Transmit UART message

## Solution Design – Inter-task Communication
* Button -> LED: Semaphore
* Button -> UART: Semaphore
* ADC -> UART: Message buffer? Queue? - TBC


## Technical details
* Board: STM32 Nucleo-F446RE 
* Toolchain: STM32CubeIDE Version: 1.5.1 (C) 2020 STMicroelectronics ALL RIGHTS RESERVED
* Middleware: FreeRTOS v10.2.1 CMSIS v2

