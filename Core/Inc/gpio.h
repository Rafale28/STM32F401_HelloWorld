#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_hal.h"

#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

void GPIO_Init(void);
void ledToggle();
void ledWrite(GPIO_PinState);
GPIO_PinState userSwRead();

#endif