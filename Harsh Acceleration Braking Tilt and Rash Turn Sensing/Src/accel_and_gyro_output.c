/*
 * accel_and_gyro_output.c
 *
 *  Created on: 19-Aug-2020
 *      Author: Bibin
 */
#include "lsm6ds3_l.h"
#include "accel_and_gyro_output.h"
#include "i2c.h"
#include "usart.h"
#include <math.h>
#include "referance_tilt_calculation.h"


extern volatile uint8_t who_am_i,memrx,uarttx_req;
extern uint8_t activity_status;
extern float mg_per_LSB;
extern char out_string[200];

void USER_accel_and_gyro_output(void)
{


	uint8_t status_reg=0;
	int16_t outx_l_xl=0,outx_h_xl=0,outy_l_xl=0,outy_h_xl=0,outz_l_xl=0,outz_h_xl=0;
	int16_t outx_xl=0,outy_xl=0,outz_xl=0;
	int16_t  x_xl_mg,y_xl_mg,z_xl_mg;
	int16_t x_angle,y_angle,z_angle;


	wait:	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,STATUS_REG,I2C_MEMADD_SIZE_8BIT,&status_reg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);




	if(status_reg&1 ==1)
	{

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTX_L_XL,I2C_MEMADD_SIZE_8BIT,&outx_l_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTX_H_XL,I2C_MEMADD_SIZE_8BIT,&outx_h_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTY_L_XL,I2C_MEMADD_SIZE_8BIT,&outy_l_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTY_H_XL,I2C_MEMADD_SIZE_8BIT,&outy_h_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTZ_L_XL,I2C_MEMADD_SIZE_8BIT,&outz_l_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTZ_H_XL,I2C_MEMADD_SIZE_8BIT,&outz_h_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		outx_xl=(outx_h_xl<<8)|outx_l_xl;
		outy_xl=(outy_h_xl<<8)|outy_l_xl;
		outz_xl=(outz_h_xl<<8)|outz_l_xl;





		x_xl_mg=(int16_t) outx_xl * mg_per_LSB  ;
		y_xl_mg=(int16_t) outy_xl * mg_per_LSB  ;
		z_xl_mg=(int16_t) outz_xl * mg_per_LSB ;



		x_angle=(int16_t)(asin(outx_xl/sqrt(outx_xl*outx_xl+outy_xl*outy_xl+outz_xl*outz_xl))*180)/M_PI;
		y_angle=(int16_t)(asin(outy_xl/sqrt(outx_xl*outx_xl+outy_xl*outy_xl+outz_xl*outz_xl))*180)/M_PI;
		z_angle=(int16_t)(acos(outz_xl/sqrt(outx_xl*outx_xl+outy_xl*outy_xl+outz_xl*outz_xl))*180)/ M_PI;

		//z_angle=radToDeg(acos(outz_xl/sqrt(pow(outx_xl,2)+pow(outy_xl,2)+pow(outz_xl,2))));

	}
	else
	{
		goto wait;
	}




	sprintf(out_string,"angles =%d , %d , %d \r\n",x_angle,y_angle,z_angle);
	//sprintf(out_string," %d , %d , %d , %d ,%d , %d, %d\r\n",x_xl_mg,y_xl_mg,z_xl_mg,activity_status*1000,x_angle,y_angle,z_angle);
	//sprintf(out_string," %d , %d , %d\r\n",x_angle,y_angle,z_angle);
	//sprintf(out_string," %d\r\n",activity_status*1000);

	/*	x_angle=(int16_t)(asin(outx_xl/sqrt(outx_xl*outx_xl+outy_xl*outy_xl+outz_xl*outz_xl))*180)/M_PI;
	y_angle=(int16_t)(asin(outy_xl/sqrt(outx_xl*outx_xl+outy_xl*outy_xl+outz_xl*outz_xl))*180)/M_PI;
	z_angle=(int16_t)(acos(outz_xl/-sqrt(outx_xl*outx_xl+outy_xl*outy_xl+outz_xl*outz_xl))*180)/ M_PI;



	 */


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);




}
