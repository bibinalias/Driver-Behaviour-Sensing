/*
 * accelerometer_referance_value_calculation.c
 *
 *  Created on: 21-Aug-2020
 *      Author: Bibin
 */
#include "accelerometer_referance_value_calculation.h"
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "status.h"

extern volatile uint8_t memrx,uarttx_req;
extern enum AXES gravity_axes;
extern char out_string[200];
extern int16_t xl_axes_ref[3];
extern enum  Sign sign_of_gravity;


void USER_Accelerometer_Referance_Value_Calculation(void)
{



	uint8_t status_reg=0,wake_up_src=0;
	int16_t array_out_xl[3]={0};
	struct out_xl out_xl_ref;

	retry:
	HAL_Delay(3);
	out_xl_ref.x=0;
	out_xl_ref.y=0;
	out_xl_ref.z=0;


	for(uint8_t loop=0;loop<NUMBER_OF_ACCELEROMETER_SAMPLES;loop++)
	{

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		if(  (wake_up_src&0x08) == 0x08)
		{

			goto retry;

		}



		wait:			memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,STATUS_REG,I2C_MEMADD_SIZE_8BIT,&status_reg,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);




		if(status_reg&1 ==1)
		{

			memrx=0;
			if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTX_L_XL,I2C_MEMADD_SIZE_8BIT,array_out_xl,6)!=HAL_OK)
			{
				Error_Handler();
			}

			while(memrx==0);





			out_xl_ref.x+=array_out_xl[X];
			out_xl_ref.y+=array_out_xl[Y];
			out_xl_ref.z+=array_out_xl[Z];
		}
		else
		{
			goto wait;
		}
		sprintf(out_string," \r\nX = %d , Y = %d , Z = %d \r\n",array_out_xl[X],array_out_xl[Y],array_out_xl[Z]);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);





		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		if(  (wake_up_src&0x08) == 0x08)
		{

			goto retry;

		}


	}

	out_xl_ref.x/=NUMBER_OF_ACCELEROMETER_SAMPLES;
	out_xl_ref.y/=NUMBER_OF_ACCELEROMETER_SAMPLES;
	out_xl_ref.z/=NUMBER_OF_ACCELEROMETER_SAMPLES;
	xl_axes_ref[X]=out_xl_ref.x;
	xl_axes_ref[Y]=out_xl_ref.y;
	xl_axes_ref[Z]=out_xl_ref.z;





	sprintf(out_string," \r\n Reference Values of Accelerometer     X-axis = %d	Y-axis = %d	Z-axis = %d\r\n",*(xl_axes_ref+X),*(xl_axes_ref+Y),*(xl_axes_ref+Z));
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);



	gravity_axes=USER_Highest_Element_of_an_Array_int16_t(xl_axes_ref,sizeof(xl_axes_ref)/sizeof(xl_axes_ref[X]));


	if(gravity_axes != AXES_NOT_SET)
	{

	if(xl_axes_ref[gravity_axes]>0)
	{
		sign_of_gravity=PLUS;
	}
	else
	{
		sign_of_gravity=MINUS;
	}

	}


	switch(gravity_axes)
	{
	case X:
		sprintf(out_string," \r\nX is Gravity Axes \r\n");
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		break;
	case Y:
		sprintf(out_string," \r\nY is Gravity Axes \r\n");
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		break;
	case Z:
		sprintf(out_string," \r\nZ is Gravity Axes \r\n");
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		break;
	case AXES_NOT_SET:	sprintf(out_string," \r\nNot able to find Gravity Axes Please try again in CALM condition ...........\r\n");
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	break;
	default:
		sprintf(out_string," \r\nSome Logical Error by USER_Accelerometer_Referance_Value_Calculation()...........\r\n");
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	}

	if(gravity_axes != AXES_NOT_SET)
	{
	sprintf(out_string," \r\nSign of Gravity	=	%d\r\n",sign_of_gravity);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	}

}
