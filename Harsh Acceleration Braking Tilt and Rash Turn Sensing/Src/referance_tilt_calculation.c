/*
 * referance_tilt_calculation.c
 *
 *  Created on: 16-Sep-2020
 *      Author: Bibin
 */
#include "referance_tilt_calculation.h"
#include "status.h"
#include "usart.h"

extern enum AXES forward_axes,gravity_axes,side_axes;
extern int16_t xl_axes_ref[3];
extern enum Referance_Tilt_function current_referance_tilt_function;
extern double referance_tilt_angle[3],referance_tilt_angle_Optimized[3];
extern char out_string[200];
extern volatile uint8_t  uarttx_req;
extern enum  Sign sign_of_gravity,sign_of_acceleration,sign_of_side_axes;



void USER_Referance_Tilt_Calculation(void)
{
	if(gravity_axes	==	AXES_NOT_SET)
	{
		USER_Referance_Tilt_Calculation_without_Gravity_axes();
	}
	else if(gravity_axes	!=	AXES_NOT_SET	&&	forward_axes == AXES_NOT_SET)
	{
		USER_Referance_Tilt_Calculation_without_Forward_axes();
	}
	else if(gravity_axes	!=	AXES_NOT_SET	&&	forward_axes!=AXES_NOT_SET)
	{
		USER_Referance_Tilt_Calculation_with_Forward_axes();
	}
}

void USER_Referance_Tilt_Calculation_without_Gravity_axes(void)
{
	memset(referance_tilt_angle,0,sizeof(referance_tilt_angle));
	current_referance_tilt_function =NO_GRAVITY_AXES;

	sprintf(out_string,"\r\nReferance Tilt Angle set to 0 for All AXES\r\n");


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
}
void USER_Referance_Tilt_Calculation_without_Forward_axes(void)
{

	switch(gravity_axes)
	{
	case X:
		referance_tilt_angle[X]=radToDeg(acos(xl_axes_ref[X]/sqrt(pow(xl_axes_ref[X],2)+pow(xl_axes_ref[Y],2)+pow(xl_axes_ref[Z],2))));
		referance_tilt_angle[Y]=radToDeg(atan2(xl_axes_ref[Y],sqrt(pow(xl_axes_ref[X],2)+pow(xl_axes_ref[Z],2))));
		referance_tilt_angle[Z]=radToDeg(atan2(xl_axes_ref[Z],sqrt(pow(xl_axes_ref[X],2)+pow(xl_axes_ref[Y],2))));

		break;
	case Y:
		referance_tilt_angle[Y]=radToDeg(acos(xl_axes_ref[Y]/sqrt(pow(xl_axes_ref[X],2)+pow(xl_axes_ref[Y],2)+pow(xl_axes_ref[Z],2))));
		referance_tilt_angle[X]=radToDeg(atan2(xl_axes_ref[X],sqrt(pow(xl_axes_ref[Y],2)+pow(xl_axes_ref[Z],2))));
		referance_tilt_angle[Z]=radToDeg(atan2(xl_axes_ref[Z],sqrt(pow(xl_axes_ref[X],2)+pow(xl_axes_ref[Y],2))));
		break;
	case Z:
		referance_tilt_angle[Z]=radToDeg(acos(xl_axes_ref[Z]/sqrt(pow(xl_axes_ref[X],2)+pow(xl_axes_ref[Y],2)+pow(xl_axes_ref[Z],2))));
		referance_tilt_angle[X]=radToDeg(atan2(xl_axes_ref[X],sqrt(pow(xl_axes_ref[Y],2)+pow(xl_axes_ref[Z],2))));
		referance_tilt_angle[Y]=radToDeg(atan2(xl_axes_ref[Y],sqrt(pow(xl_axes_ref[X],2)+pow(xl_axes_ref[Z],2))));

		break;
	case AXES_NOT_SET:
		sprintf(out_string,"\r\nPlease Set Device Orientation to find Reference Tilt Angle\r\n");


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		break;

	default:sprintf(out_string,"\r\nSome Logical Error\r\n");


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	}


	sprintf(out_string,"\r\nReferance of Gravity Angle = %d , Forward / Side Angle 1 = %d , Forward / Side Angle 2 = %d\r\n",(int16_t)referance_tilt_angle[gravity_axes],(int16_t)referance_tilt_angle[(gravity_axes+1)%3],(int16_t)referance_tilt_angle[(gravity_axes+2)%3]);


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	current_referance_tilt_function	=	ONLY_GRAVITY_AXES;

}


void USER_Referance_Tilt_Calculation_with_Forward_axes(void)
{

	referance_tilt_angle_Optimized[gravity_axes]	=	referance_tilt_angle[gravity_axes];
	referance_tilt_angle_Optimized[forward_axes]	=	sign_of_acceleration*referance_tilt_angle[forward_axes];
	referance_tilt_angle_Optimized[side_axes]		=	sign_of_side_axes*referance_tilt_angle[side_axes];

	sprintf(out_string,"\r\nReferance of Forward Angle = %d, Side Angle = %d , Gravity Angle = %d\r\n",(int16_t)referance_tilt_angle_Optimized[forward_axes],(int16_t)referance_tilt_angle_Optimized[side_axes],(int16_t)referance_tilt_angle_Optimized[gravity_axes]);

	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	current_referance_tilt_function	=	GRAVITY_AXES_PLUS_FORWARD_AXES;




}
