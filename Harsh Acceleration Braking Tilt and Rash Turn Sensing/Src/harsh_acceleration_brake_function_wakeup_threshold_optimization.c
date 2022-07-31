/*
 * harsh_acceleration_brake_function_wakeup_threshold_optimization.c
 *
 *  Created on: 01-Oct-2020
 *      Author: Bibin
 */
#include "harsh_acceleration_brake_function_wakeup_threshold_optimization.h"
#include "usart.h"
#include "status.h"

extern float mg_per_LSB;
extern volatile uint8_t uarttx_req;
extern uint16_t harsh_accel_brake_function_wakeup_threshold,harsh_accel_brake_function_wakeup_threshold_optimized;
extern char out_string[200];
extern enum AXES forward_axes;
extern double referance_tilt_angle[3];


uint8_t USER_Harsh_Acceleration_Brake_Function_Wakeup_Threshold_Optimize(void)
{
	uint8_t i;
	extern uint16_t harsh_accel_brake_function_wakeup_threshold_optimized;

	harsh_accel_brake_function_wakeup_threshold_optimized=harsh_accel_brake_function_wakeup_threshold*cos(degToRad(referance_tilt_angle[forward_axes]));
	for(i=0;i*pow(2,15)*mg_per_LSB/pow(2,6) < harsh_accel_brake_function_wakeup_threshold_optimized+1 ;i++)
	{

	}

	sprintf(out_string,"\r\nOptimized Harsh Acceleration and Brake Wakeup Threshold is %d mg and it is set to %d mg\r\n",harsh_accel_brake_function_wakeup_threshold_optimized,(int16_t)(--i*pow(2,15)*mg_per_LSB/pow(2,6)));
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	return i;
}
