#ifndef __UART_DMA_START_H
#define __UART_DMA_START_H

#include "stm32f0xx_hal.h"
#include "usart.h"

void USER_UART_DMA_start(void);

#define D_CON_RS485_Pin GPIO_PIN_1
#define D_CON_RS485_GPIO_Port GPIOB

#endif
