/*
 * gyroscope_referance_value_calculation.c
 *
 *  Created on: 21-Aug-2020
 *      Author: Bibin
 */
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "gyroscope_referance_value_calculation.h"
#include "status.h"
	extern volatile uint8_t memrx,uarttx_req;
	extern char out_string[200];
	extern int16_t g_axes_ref[3];
void USER_Gyroscope_Referance_Value_Calculation(void)
{



	uint8_t status_reg=0,wake_up_src=0;
	int16_t out_g[3]={0};
	struct out_g out_g_ref;



	retry:
	HAL_Delay(3);
	out_g_ref.x=0;
	out_g_ref.y=0;
	out_g_ref.z=0;
	for(uint8_t loop=0;loop<NUMBER_OF_GYROSCOPE_SAMPLES;loop++)
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



		wait:		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,STATUS_REG,I2C_MEMADD_SIZE_8BIT,&status_reg,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);




		if(status_reg&2 ==2)
		{

			memrx=0;
			if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTX_L_G,I2C_MEMADD_SIZE_8BIT,out_g,6)!=HAL_OK)
			{
				Error_Handler();
			}

			while(memrx==0);





			out_g_ref.x+=out_g[X];
			out_g_ref.y+=out_g[Y];
			out_g_ref.z+=out_g[Z];
		}
		else
		{
			goto wait;


		}
		sprintf(out_string," \r\nX = %d , Y = %d , Z = %d \r\n",out_g[X],out_g[Y],out_g[Z]);
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

	out_g_ref.x/=NUMBER_OF_GYROSCOPE_SAMPLES;
	out_g_ref.y/=NUMBER_OF_GYROSCOPE_SAMPLES;
	out_g_ref.z/=NUMBER_OF_GYROSCOPE_SAMPLES;
	g_axes_ref[X]=out_g_ref.x;
	g_axes_ref[Y]=out_g_ref.y;
	g_axes_ref[Z]=out_g_ref.z;





	sprintf(out_string," \r\n Reference Values of Gyroscope     X-axis = %d	Y-axis = %d	 Z-axis = %d \r\n",*(g_axes_ref+X),*(g_axes_ref+Y),*(g_axes_ref+Z));
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);








}
