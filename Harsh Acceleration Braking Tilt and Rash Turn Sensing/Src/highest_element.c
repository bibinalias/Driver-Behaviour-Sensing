/*
 * highest_element.c
 *
 *  Created on: 21-Aug-2020
 *      Author: Bibin
 */
#include "highest_element.h"
#include "usart.h"
#include <math.h>

uint8_t USER_Highest_Element_of_an_Array_int16_t(int16_t* array,uint8_t no_of_element)
{
	extern volatile uint8_t memrx,uarttx_req;
	extern char out_string[200];
	uint16_t max;
	uint8_t order;
	max=abs(array[0]);
	order=0;
	for(uint8_t j=1;j<no_of_element;j++)
	{
		if(abs(array[j]) > max)
		{
			max=abs(array[j]);
			order=j;
		}
	}
	sprintf(out_string," \r\nMax Value = %d		order = %d\r\n",array[order],order);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	return order;
}

uint8_t USER_Highest_Element_of_an_Array_int8_t(int8_t* array,uint8_t no_of_element)
{
	extern volatile uint8_t memrx,uarttx_req;
	extern char out_string[200];
	uint8_t max;
	uint8_t order;



	max=abs(array[0]);
	order=0;
	for(uint8_t j=1;j<no_of_element;j++)
	{
		if(abs(array[j]) > max)
		{
			max=abs(array[j]);
			order=j;
		}
	}
	sprintf(out_string," \r\nMax Value = %d		order = %d\r\n",array[order],order);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	return order;
}

