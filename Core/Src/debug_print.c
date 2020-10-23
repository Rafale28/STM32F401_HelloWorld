#include "dubug_print.h"

uint32_t DEBUG_PRINT_Init(void) {
    uint32_t ret;
    ret = USART2_UART_Init();
    return ret;
}

static uint32_t USART2_UART_Init(void)
{
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
      return UART_INITIALIZE_ERROR;
  }
  return UART_INITIALIZE_OK;
}

int debug_printf(const char* format, ...) {
  char str[256];
  int ssize;
  va_list arg;
  va_start (arg, format);
  ssize = vsprintf(str, format, arg);
  va_end(arg);
  HAL_UART_Transmit(&huart2, (uint8_t *)str, ssize, 0xF);
  return ssize;
}