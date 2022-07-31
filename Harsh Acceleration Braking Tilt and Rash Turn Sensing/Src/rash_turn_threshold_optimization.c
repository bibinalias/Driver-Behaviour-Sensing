/*
 * rash_turn_threshold_optimization.c
 *
 *  Created on: 28-Sep-2020
 *      Author: Bibin
 */
#include "rash_turn_threshold_optimization.h"
#include "status.h"
#include "usart.h"
extern volatile uint8_t uarttx_req;
extern uint8_t rtt;
extern enum AXES gravity_axes;
extern double referance_tilt_angle[3];
extern float rtt_optimized;
extern char out_string[200];
void RASH_Turn_Threshold_Optimize(void)
{
	if(gravity_axes	!=	AXES_NOT_SET)
		{
			if(abs((int8_t)referance_tilt_angle[(gravity_axes+1)%3])	>	abs((int8_t)referance_tilt_angle[(gravity_axes+2)%3]))
			{
				rtt_optimized=rtt*cos(degToRad(referance_tilt_angle[(gravity_axes+1)%3]));
			}
			else
			{
				rtt_optimized=rtt*cos(degToRad(referance_tilt_angle[(gravity_axes+2)%3]));
			}
		}
		else
		{

			rtt_optimized=rtt;
		}

		sprintf(out_string,"\r\nRASH Turn Threshold is %d and Now it is Optimized to %d\r\n",rtt,(int8_t)rtt_optimized);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
}
