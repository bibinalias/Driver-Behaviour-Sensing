/*
 * rash_turn_sense.c
 *
 *  Created on: 25-Sep-2020
 *      Author: Bibin
 */
#include "rash_turn_detection.h"
#include "status.h"
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"

extern enum AXES gravity_axes;
extern enum  Sign sign_of_gravity;
extern volatile uint8_t memrx,memtx,uarttx_req;
extern char out_string[200];
extern float mdps_per_LSB,rtt_optimized;
static uint8_t rash_turn_itration=0;
static float avg_yaw_dps=0;

enum Rash_Turn USER_Rash_Turn_Detection(void)
{
	int16_t array_out_g[3];
	uint8_t status_reg;
	float current_yaw_dps;
	enum Rash_Turn rash_turn_status =NO_RASH_TURN;

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

	if(gravity_axes!=AXES_NOT_SET)
	{
		current_yaw_dps=sign_of_gravity*array_out_g[gravity_axes]*mdps_per_LSB/1000;

		if(abs((int16_t)current_yaw_dps) > rtt_optimized)
		{
			rash_turn_itration++;
			avg_yaw_dps+=current_yaw_dps;

			if(rash_turn_itration > RASH_TURN_ITRATION_TRESHOLD)
			{
				avg_yaw_dps/=rash_turn_itration;

				if(avg_yaw_dps < 0)
				{
					rash_turn_status	=	RASH_TURN_RIGHT;
					sprintf(out_string,"\r\n\r\nRASH Right Turn Detected		Angle Rate is %d	>	%d\r\n\r\n",(int16_t)avg_yaw_dps,(int8_t)rtt_optimized);
				}
				else
				{
					rash_turn_status	=	RASH_TURN_LEFT;
					sprintf(out_string,"\r\n\r\nRASH Left Turn Detected		Angle Rate is %d	>	%d\r\n\r\n",(int16_t)avg_yaw_dps,(int8_t)rtt_optimized);
				}


				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				avg_yaw_dps=0;
				rash_turn_itration=0;
			}




		}
		else
		{
			avg_yaw_dps=0;
			rash_turn_itration=0;
		}


	}
	else
	{
		current_yaw_dps=array_out_g[Z]*mdps_per_LSB/1000;

		if(abs((int16_t)current_yaw_dps) > rtt_optimized)
		{
			rash_turn_itration++;
			avg_yaw_dps+=current_yaw_dps;

			if(rash_turn_itration > RASH_TURN_ITRATION_TRESHOLD)
			{
				avg_yaw_dps/=rash_turn_itration;

				if(avg_yaw_dps < 0)
				{
					rash_turn_status	=	RASH_TURN_RIGHT;
					sprintf(out_string,"\r\n\r\nRASH Right Turn Detected		Angle Rate is %d	>	%d\r\n\r\n",(int16_t)avg_yaw_dps,(int8_t)rtt_optimized);
				}
				else
				{
					rash_turn_status	=	RASH_TURN_LEFT;
					sprintf(out_string,"\r\n\r\nRASH Left Turn Detected		Angle Rate is %d	>	%d\r\n\r\n",(int16_t)avg_yaw_dps,(int8_t)rtt_optimized);
				}


				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				avg_yaw_dps=0;
				rash_turn_itration=0;
			}




		}
		else
		{
			avg_yaw_dps=0;
			rash_turn_itration=0;
		}

	}
	return rash_turn_status;
}


