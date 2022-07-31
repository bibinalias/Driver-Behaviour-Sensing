/*
 * tilt_calculation.c
 *
 *  Created on: 18-Sep-2020
 *      Author: Bibin
 */

#include "tilt_calculation.h"
#include "status.h"
#include "usart.h"
#include "i2c.h"
#include "lsm6ds3_l.h"

extern enum  Sign sign_of_gravity,sign_of_acceleration,sign_of_side_axes;
extern volatile uint8_t  uarttx_req,memrx;
extern enum Referance_Tilt_function current_referance_tilt_function;
extern enum AXES forward_axes,gravity_axes,side_axes;
extern char out_string[200];
extern double referance_tilt_angle[3];
extern uint8_t tilt_threshold;

enum Tilt USER_Tilt_Calculation(void)
{
	enum Tilt tilt_status	=	NO_TILT;
	uint8_t status_reg;
	int16_t array_out_xl[3];
	double current_tilt_angle[3],tilt_angle[3];
	double tilt_angle_Optimized[3];
	static uint8_t tilt_i;


	wait:		memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,STATUS_REG,I2C_MEMADD_SIZE_8BIT,&status_reg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);



	if((status_reg&1) ==1)
	{

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTX_L_XL,I2C_MEMADD_SIZE_8BIT,array_out_xl,6)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
	}
	else
	{
		goto wait;
	}


	switch(gravity_axes)
	{
	case X:
		current_tilt_angle[X]=radToDeg(acos(array_out_xl[X]/sqrt(pow(array_out_xl[X],2)+pow(array_out_xl[Y],2)+pow(array_out_xl[Z],2))));
		current_tilt_angle[Y]=radToDeg(atan2(array_out_xl[Y],sqrt(pow(array_out_xl[X],2)+pow(array_out_xl[Z],2))));
		current_tilt_angle[Z]=radToDeg(atan2(array_out_xl[Z],sqrt(pow(array_out_xl[X],2)+pow(array_out_xl[Y],2))));

		break;
	case Y:
		current_tilt_angle[Y]=radToDeg(acos(array_out_xl[Y]/sqrt(pow(array_out_xl[X],2)+pow(array_out_xl[Y],2)+pow(array_out_xl[Z],2))));
		current_tilt_angle[X]=radToDeg(atan2(array_out_xl[X],sqrt(pow(array_out_xl[Y],2)+pow(array_out_xl[Z],2))));
		current_tilt_angle[Z]=radToDeg(atan2(array_out_xl[Z],sqrt(pow(array_out_xl[X],2)+pow(array_out_xl[Y],2))));
		break;

	case Z:
	case AXES_NOT_SET:
		current_tilt_angle[Z]=radToDeg(acos(array_out_xl[Z]/sqrt(pow(array_out_xl[X],2)+pow(array_out_xl[Y],2)+pow(array_out_xl[Z],2))));
		current_tilt_angle[X]=radToDeg(atan2(array_out_xl[X],sqrt(pow(array_out_xl[Y],2)+pow(array_out_xl[Z],2))));
		current_tilt_angle[Y]=radToDeg(atan2(array_out_xl[Y],sqrt(pow(array_out_xl[X],2)+pow(array_out_xl[Z],2))));

		break;

	default:
		sprintf(out_string,"\r\nSome Logical Error From USER_Tilt_Calculation()\r\n");
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	}


	tilt_angle[X]=current_tilt_angle[X]-referance_tilt_angle[X];
	tilt_angle[Y]=current_tilt_angle[Y]-referance_tilt_angle[Y];
	tilt_angle[Z]=current_tilt_angle[Z]-referance_tilt_angle[Z];

	if(current_referance_tilt_function 	==	GRAVITY_AXES_PLUS_FORWARD_AXES)
	{
		tilt_angle_Optimized[gravity_axes]	=	tilt_angle[gravity_axes];
		tilt_angle_Optimized[forward_axes]	=	sign_of_acceleration*tilt_angle[forward_axes];
		tilt_angle_Optimized[side_axes]		=	sign_of_side_axes*tilt_angle[side_axes];

	}
	else
	{

		tilt_angle_Optimized[gravity_axes]	=	0;
		tilt_angle_Optimized[forward_axes]	=	0;
		tilt_angle_Optimized[side_axes]		=	0;

	}

	if(abs((int16_t)tilt_angle[X]) > tilt_threshold || abs((int16_t)tilt_angle[Y]) > tilt_threshold || abs((int16_t)tilt_angle[Z]) > tilt_threshold)
	{
		tilt_i++;
	}
	else
	{
		tilt_i=0;
	}

	if(tilt_i	>	TILT_ITRATION_TRESHOLD)
	{
		tilt_status	=	TILT;

		sprintf(out_string,"\r\n TILT 		Tilt > %d degree \r\n",tilt_threshold);


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);



		switch(current_referance_tilt_function)
		{

		case ONLY_GRAVITY_AXES:
			sprintf(out_string,"\r\n Tilt Angle of Gravity Angle  = %d , Forward / Side Angle 1 = %d , Forward / Side Angle 2 = %d\r\n",(int16_t)tilt_angle[gravity_axes],(int16_t)tilt_angle[(gravity_axes+1)%3],(int16_t)tilt_angle[(gravity_axes+2)%3]);


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

			break;
		case GRAVITY_AXES_PLUS_FORWARD_AXES:


			sprintf(out_string,"\r\nForward Tilt Angle = %d, Side Tilt Angle = %d , Gravity Tilt Angle = %d\r\n",(int16_t)tilt_angle_Optimized[forward_axes],(int16_t)tilt_angle_Optimized[side_axes],(int16_t)tilt_angle_Optimized[gravity_axes]);

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			break;

		case NO_GRAVITY_AXES:
			sprintf(out_string,"\r\nTilt Angles X = %d , Y = %d , Z = %d\r\n",(int16_t)tilt_angle[X],(int16_t)tilt_angle[Y],(int16_t)tilt_angle[Z]);


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			break;

		default:sprintf(out_string,"\r\nSome Logical Error in tilt_calculation.c \r\n");


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


		}

		tilt_i=0;

	}
	return tilt_status;
}
