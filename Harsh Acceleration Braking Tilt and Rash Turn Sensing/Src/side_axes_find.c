/*
 * side_axes_find.c
 *
 *  Created on: 21-Sep-2020
 *      Author: Bibin
 */
#include "side_axes_find.h"
#include "lsm6ds3_l.h"
#include "status.h"
#include "usart.h"
extern enum AXES forward_axes,gravity_axes,side_axes;
extern char out_string[200];
extern enum  Sign sign_of_gravity,sign_of_acceleration,sign_of_side_axes;
extern volatile uint8_t  uarttx_req;

void USER_Side_Axes_Find(void)
{
	int8_t i;

	struct axes_with_sign side_axes_of_gravity_axes[4];

	switch (gravity_axes)
	{
	case X:
		if (sign_of_gravity == PLUS)
		{
			side_axes_of_gravity_axes[0].axes = Y;
			side_axes_of_gravity_axes[0].sign = MINUS;
			side_axes_of_gravity_axes[1].axes = Z;
			side_axes_of_gravity_axes[1].sign = MINUS;
			side_axes_of_gravity_axes[2].axes = Y;
			side_axes_of_gravity_axes[2].sign = PLUS;
			side_axes_of_gravity_axes[3].axes = Z;
			side_axes_of_gravity_axes[3].sign = PLUS;


		}
		else if (sign_of_gravity == MINUS)
		{
			side_axes_of_gravity_axes[0].axes = Y;
			side_axes_of_gravity_axes[0].sign = MINUS;
			side_axes_of_gravity_axes[1].axes = Z;
			side_axes_of_gravity_axes[1].sign = PLUS;
			side_axes_of_gravity_axes[2].axes = Y;
			side_axes_of_gravity_axes[2].sign = PLUS;
			side_axes_of_gravity_axes[3].axes = Z;
			side_axes_of_gravity_axes[3].sign = MINUS;

		}
		break;
	case Y:
		if (sign_of_gravity == PLUS)
		{
			side_axes_of_gravity_axes[0].axes = X;
			side_axes_of_gravity_axes[0].sign = MINUS;
			side_axes_of_gravity_axes[1].axes = Z;
			side_axes_of_gravity_axes[1].sign = PLUS;
			side_axes_of_gravity_axes[2].axes = X;
			side_axes_of_gravity_axes[2].sign = PLUS;
			side_axes_of_gravity_axes[3].axes = Z;
			side_axes_of_gravity_axes[3].sign = MINUS;
		}
		else if (sign_of_gravity == MINUS)
		{
			side_axes_of_gravity_axes[0].axes = X;
			side_axes_of_gravity_axes[0].sign = MINUS;
			side_axes_of_gravity_axes[1].axes = Z;
			side_axes_of_gravity_axes[1].sign = MINUS;
			side_axes_of_gravity_axes[2].axes = X;
			side_axes_of_gravity_axes[2].sign = PLUS;
			side_axes_of_gravity_axes[3].axes = Z;
			side_axes_of_gravity_axes[3].sign = PLUS;

		}
		break;
	case Z:
		if (sign_of_gravity == PLUS)
		{
			side_axes_of_gravity_axes[0].axes = X;
			side_axes_of_gravity_axes[0].sign = MINUS;
			side_axes_of_gravity_axes[1].axes = Y;
			side_axes_of_gravity_axes[1].sign = MINUS;
			side_axes_of_gravity_axes[2].axes = X;
			side_axes_of_gravity_axes[2].sign = PLUS;
			side_axes_of_gravity_axes[3].axes = Y;
			side_axes_of_gravity_axes[3].sign = PLUS;

		}
		else if (sign_of_gravity == MINUS)
		{
			side_axes_of_gravity_axes[0].axes = X;
			side_axes_of_gravity_axes[0].sign = MINUS;
			side_axes_of_gravity_axes[1].axes = Y;
			side_axes_of_gravity_axes[1].sign = PLUS;
			side_axes_of_gravity_axes[2].axes = X;
			side_axes_of_gravity_axes[2].sign = PLUS;
			side_axes_of_gravity_axes[3].axes = Y;
			side_axes_of_gravity_axes[3].sign = MINUS;
		}
		break;
	case AXES_NOT_SET:
		sprintf(out_string,"\r\nSome Logical Error in USER_Side_Axes_Find \r\n");


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


		break;
	default:sprintf(out_string,"\r\nSome Logical Error in USER_Side_Axes_Find \r\n");


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	}

	for (i = 0; !(side_axes_of_gravity_axes[i].axes == forward_axes && side_axes_of_gravity_axes[i].sign == sign_of_acceleration); i++)
	{

	}

	side_axes = side_axes_of_gravity_axes[(i + 3) % 4].axes;
	sign_of_side_axes = side_axes_of_gravity_axes[(i + 3) % 4].sign;

	sprintf(out_string,"\r\nSide Axes = %d , Sign of Side Axes = %d \r\n",side_axes,sign_of_side_axes);


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


}
