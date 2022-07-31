/*
 * harsh_acceleration_harsh_brake_find.c
 *
 *  Created on: 11-Sep-2020
 *      Author: Bibin
 */
#include "harsh_acceleration_harsh_brake_find.h"
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "status.h"

extern float mg_per_LSB;
extern int16_t xl_axes_ref[3];
extern char out_string[200];
extern volatile uint8_t  uarttx_req,memrx;
extern enum AXES forward_axes;
extern enum  Sign sign_of_acceleration;
static uint8_t harsh_accel_itration=0,harsh_brake_itration=0,not_harsh_itration=0;



enum Acceleration_Brake USER_Harsh_Acceleration_Harsh_brake_Find(void)
{
	enum Acceleration_Brake current_acceleration_brake_status=NOTHING;
	uint8_t wake_up_src,status_reg;
	int8_t error_int_xl;
	int16_t array_out_xl[3],error_current_xl;


	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(  (wake_up_src&0x08) == 0x08)
	{




		wait:	memrx=0;
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




		switch (forward_axes)
		{

		case X: 	if((wake_up_src&0x04)==0x04)
		{
			error_current_xl	=	array_out_xl[X]	-	xl_axes_ref[X];
			if(error_current_xl>0)
			{
				error_int_xl=PLUS;

			}
			else
			{
				error_int_xl=MINUS;
			}

			sprintf(out_string,"\r\X - error_current_xl=%d\r\n",(int16_t)(error_current_xl*mg_per_LSB));


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

			if(error_int_xl==sign_of_acceleration)
			{
				harsh_accel_itration++;
				harsh_brake_itration=0;

			}
			else
			{
				harsh_accel_itration=0;
				harsh_brake_itration++;
			}

			if(harsh_accel_itration >= HARSH_ACCELERATION_HARSH_BRAKE_ITRATION_THRESHOLD)
			{
				harsh_accel_itration=0;
				current_acceleration_brake_status=HARSH_ACCELERATION;



			}
			if(harsh_brake_itration >= HARSH_ACCELERATION_HARSH_BRAKE_ITRATION_THRESHOLD)
			{
				harsh_brake_itration=0;

				current_acceleration_brake_status=HARSH_BRAKING;

			}

			not_harsh_itration=0;

		}
		else
		{
			not_harsh_itration++;


			if(not_harsh_itration	>= NOT_HARSH_ITRATION_THRESHOLD)
			{

				harsh_accel_itration=0;
				harsh_brake_itration=0;
				not_harsh_itration=0;
			}
		}
		break;

		case Y:	if((wake_up_src&0x02)==0x02)
		{
			error_current_xl=	array_out_xl[Y]	-	xl_axes_ref[Y];
			if(error_current_xl>0)
			{
				error_int_xl	=	PLUS;

			}
			else
			{
				error_int_xl	=	MINUS;
			}
			sprintf(out_string,"\r\Y - error_current_xl=%d\r\n",(int16_t)(error_current_xl*mg_per_LSB));


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

			if(error_int_xl	==	sign_of_acceleration)
			{
				harsh_accel_itration++;
				harsh_brake_itration=0;

			}
			else
			{
				harsh_accel_itration=0;
				harsh_brake_itration++;
			}

			if(harsh_accel_itration >= HARSH_ACCELERATION_HARSH_BRAKE_ITRATION_THRESHOLD)
			{
				harsh_accel_itration=0;
				current_acceleration_brake_status=HARSH_ACCELERATION;



			}
			if(harsh_brake_itration >= HARSH_ACCELERATION_HARSH_BRAKE_ITRATION_THRESHOLD)
			{
				harsh_brake_itration=0;

				current_acceleration_brake_status=HARSH_BRAKING;

			}


			not_harsh_itration=0;

			}
			else
			{
				not_harsh_itration++;
				if(not_harsh_itration	>= NOT_HARSH_ITRATION_THRESHOLD)
				{
					not_harsh_itration=0;
					harsh_accel_itration=0;
					harsh_brake_itration=0;
				}
			}
		break;
		case Z:	if((wake_up_src&0x01)==0x01)
		{
			error_current_xl	=	array_out_xl[Z]	-	xl_axes_ref[Z];
			if(error_current_xl>0)
			{
				error_int_xl=PLUS;

			}
			else
			{
				error_int_xl=MINUS;
			}

			sprintf(out_string,"\r\Z - error_current_xl=%d\r\n",(int16_t)(error_current_xl*mg_per_LSB));


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

			if(error_int_xl==sign_of_acceleration)
			{
				harsh_accel_itration++;
				harsh_brake_itration=0;

			}
			else
			{
				harsh_accel_itration=0;
				harsh_brake_itration++;
			}

			if(harsh_accel_itration >= HARSH_ACCELERATION_HARSH_BRAKE_ITRATION_THRESHOLD)
			{
				harsh_accel_itration=0;
				current_acceleration_brake_status=HARSH_ACCELERATION;



			}
			if(harsh_brake_itration >= HARSH_ACCELERATION_HARSH_BRAKE_ITRATION_THRESHOLD)
			{
				harsh_brake_itration=0;

				current_acceleration_brake_status=HARSH_BRAKING;

			}



			not_harsh_itration=0;

			}
			else
			{
				not_harsh_itration++;
				if(not_harsh_itration	>= NOT_HARSH_ITRATION_THRESHOLD)
				{
					not_harsh_itration=0;
					harsh_accel_itration=0;
					harsh_brake_itration=0;
				}
			}
		break;
		case AXES_NOT_SET:sprintf(out_string,"\r\n forward Axes is not Set\r\n");

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		break;

		default:sprintf(out_string,"\r\nSome Logical Error from USER_Harsh_Acceleration_Harsh_brake_Find()\r\n");


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
	else
	{
		not_harsh_itration=0;
		harsh_accel_itration=0;
		harsh_brake_itration=0;
	}


	if(current_acceleration_brake_status==HARSH_ACCELERATION)
	{
		sprintf(out_string,"\r\nHARSH Acceleration\r\n");


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	}
	if(current_acceleration_brake_status==HARSH_BRAKING)
	{
		sprintf(out_string,"\r\nHARSH Braking\r\n");


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	}

	return current_acceleration_brake_status;
}
