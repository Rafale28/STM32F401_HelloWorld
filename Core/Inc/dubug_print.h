#ifndef __DEBUG_PRINT_H
#define __DEBUG_PRINT_H

#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"

#define UART_INITIALIZE_OK 0x0;
#define UART_INITIALIZE_ERROR 0xFFFFFFFF;

UART_HandleTypeDef huart2;

uint32_t DEBUG_PRINT_Init(void);
static uint32_t USART2_UART_Init(void);
int debug_printf(const char*, ...);

#endif