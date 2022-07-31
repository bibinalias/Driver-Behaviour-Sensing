/*
 * gyroscope_output_data.c
 *
 *  Created on: 24-Sep-2020
 *      Author: Bibin
 */

#include "lsm6ds3_l.h"
#include "status.h"
#include "gyroscope_output_data.h"
#include "i2c.h"
#include "usart.h"
extern volatile uint8_t memrx,memtx,uarttx_req;
extern char out_string[200];
extern float mdps_per_LSB;
void USER_Gyroscope_Output_Data(void)
{
int16_t array_out_g[3];
uint8_t status_reg;

wait:memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,STATUS_REG,I2C_MEMADD_SIZE_8BIT,&status_reg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);




	if(status_reg&2 ==2)
	{

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTX_L_G,I2C_MEMADD_SIZE_8BIT,array_out_g,6)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

	}
	else
	{
	goto wait;
	}


	sprintf(out_string," Gyro Output %d , %d , %d \r\n",(int8_t)(array_out_g[X]*mdps_per_LSB/1000),(int8_t)(array_out_g[Y]*mdps_per_LSB/1000),(int8_t)(array_out_g[Z]*mdps_per_LSB/1000));


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
}
