
#include "uart_dma_start.h"


void USER_UART_DMA_start(void)
{
   extern volatile char c;
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	HAL_UART_Receive_DMA(&huart3,&c,1);
}


