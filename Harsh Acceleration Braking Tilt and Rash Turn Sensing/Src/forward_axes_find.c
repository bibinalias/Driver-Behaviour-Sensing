/*
 * forward_axes_find.c
 *
 *  Created on: 08-Sep-2020
 *      Author: Bibin*/

#include "forward_axes_find.h"
#include "status.h"
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "highest_element.h"

extern int16_t xl_axes_ref[3];
extern  float total_error_xl_mg[3];
extern  int8_t total_error_int_xl[3];
extern char out_string[200];
extern enum Wakeup_Device_Mode device_mode;
extern enum AXES gravity_axes,forward_axes;
extern volatile uint8_t  uarttx_req,memrx;
extern float mg_per_LSB;
extern enum  Sign sign_of_acceleration;


void USER_Forward_axes_Find(void)
{

	uint8_t wake_up_src,status_reg;
	int8_t error_int_xl[3]={0};
	int16_t array_out_xl[3],error_current_xl[3]={0};

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(  (wake_up_src&0x08) == 0x08)
	{

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);



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
			if((wake_up_src&0x02) == 0x02)
			{



				error_current_xl[Y]	=	array_out_xl[Y]	-	xl_axes_ref[Y]	;
				if(error_current_xl[Y]>0)
				{
					error_int_xl[Y]=PLUS;
				}
				else
				{
					error_int_xl[Y]=MINUS;
				}


				sprintf(out_string,"\r\nY Acceleration/break	error_current_xl[Y]=%d,error_int_xl[Y]=%d\r\n",error_current_xl[Y],error_int_xl[Y]);

				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);

			}

			if((wake_up_src&0x01) == 0x01)
			{

				error_current_xl[Z]	=	array_out_xl[Z]	-	xl_axes_ref[Z];
				if(error_current_xl[Z]>0)
				{
					error_int_xl[Z]=PLUS;
				}
				else
				{
					error_int_xl[Z]=MINUS;
				}


				sprintf(out_string,"\r\nZ Acceleration/break	error_current_xl[Z]=%d,error_int_xl[Z]=%d\r\n",error_current_xl[Z],error_int_xl[Z]);

				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);



			}



			break;


		case Y:
			if((wake_up_src&0x04) == 0x04)
			{
				error_current_xl[X]	=	array_out_xl[X]	-	xl_axes_ref[X];
				if(error_current_xl[X]>0)
				{
					error_int_xl[X]=PLUS;
				}
				else
				{
					error_int_xl[X]=MINUS;
				}


				sprintf(out_string,"\r\nX Acceleration/break	error_current_xl[X]=%d,error_int_xl[X]=%d\r\n",error_current_xl[X],error_int_xl[X]);

				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);


			}
			if((wake_up_src&0x01) == 0x01)
			{

				error_current_xl[Z]	=	array_out_xl[Z]	-	xl_axes_ref[Z];
				if(error_current_xl[Z]>0)
				{
					error_int_xl[Z]=PLUS;
				}
				else
				{
					error_int_xl[Z]=MINUS;
				}


				sprintf(out_string,"\r\nZ Acceleration/break	error_current_xl[Z]=%d,error_int_xl[Z]=%d\r\n",error_current_xl[Z],error_int_xl[Z]);

				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);




			}



			break;


		case Z:
			if((wake_up_src&0x02) == 0x02)
			{

				error_current_xl[Y]	=	array_out_xl[Y]	-	xl_axes_ref[Y];
				if(error_current_xl[Y]>0)
				{
					error_int_xl[Y]=PLUS;
				}
				else
				{
					error_int_xl[Y]=MINUS;
				}


				sprintf(out_string,"\r\nY Acceleration/break	error_current_xl[Y]=%d,error_int_xl[Y]=%d\r\n",error_current_xl[Y],error_int_xl[Y]);

				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);


			}
			if((wake_up_src&0x04) == 0x04)
			{


				error_current_xl[X]	=	array_out_xl[X]	-	xl_axes_ref[X];
				if(error_current_xl[X]>0)
				{
					error_int_xl[X]=PLUS;
				}
				else
				{
					error_int_xl[X]=MINUS;
				}


				sprintf(out_string,"\r\nX Acceleration/break	error_current_xl[X]=%d,error_int_xl[X]=%d\r\n",error_current_xl[X],error_int_xl[X]);

				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);


			}




			break;


		case AXES_NOT_SET:
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nPlease Set Device Orientation by USER_Forward_axes_Find()\r\n",strlen("\r\nPlease Set Device Orientation by USER_Forward_axes_Find()\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			break;

		default:if(HAL_UART_Transmit_DMA(&huart3,"\r\nSomeLogical Error from USER_Forward_axes_Find()\r\n",strlen("\r\nSomeLogical Error from USER_Forward_axes_Find()\r\n"))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);



		}




		total_error_xl_mg[X]+=(error_current_xl[X]*mg_per_LSB);
		total_error_xl_mg[Y]+=(error_current_xl[Y]*mg_per_LSB);
		total_error_xl_mg[Z]+=(error_current_xl[Z]*mg_per_LSB);

		total_error_int_xl[X]+=error_int_xl[X];
		total_error_int_xl[Y]+=error_int_xl[Y];
		total_error_int_xl[Z]+=error_int_xl[Z];




		if((abs(total_error_int_xl[X])>FORWARD_AXES_TRESHOLD	||	abs(total_error_int_xl[Y])>FORWARD_AXES_TRESHOLD	||	abs(total_error_int_xl[Z])>FORWARD_AXES_TRESHOLD))
		{

			forward_axes=USER_Highest_Element_of_an_Array_int8_t(total_error_int_xl,sizeof(total_error_int_xl)/sizeof(total_error_int_xl[X]));

			if(total_error_int_xl[forward_axes]>0)
			{
				sign_of_acceleration=PLUS;
			}
			else if(total_error_int_xl[forward_axes]<0)
			{
				sign_of_acceleration=MINUS;
			}
			else
			{
				sign_of_acceleration=SIGN_NOT_SET;
			}


			sprintf(out_string,"\r\nDevice mode is now in Acceleration Mode\r\n\r\ntotal_error_xl_mg[X] = %d ,total_error_xl_mg[Y] = %d ,total_error_xl_mg[Z] = %d",(int)total_error_xl_mg[X],(int)total_error_xl_mg[Y],(int)total_error_xl_mg[Z]);

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);

			sprintf(out_string,"\r\ntotal_error_int_xl[X] = %d , total_error_int_xl[Y] = %d , total_error_int_xl[Z] = %d\r\nforward_axes=%d sign_of_acceleration=%d\r\n\r\n",total_error_int_xl[X],total_error_int_xl[Y],total_error_int_xl[Z],forward_axes,sign_of_acceleration);

			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);

			USER_Side_Axes_Find();

			device_mode=ACCELERATION;



		}




		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	}






}
