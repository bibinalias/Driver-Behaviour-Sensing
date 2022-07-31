/*
 * get_command.c
 *
 *  Created on: 19-Aug-2020
 *      Author: Bibin
 */
#include "i2c.h"
#include "lsm6ds3_l.h"
#include "usart.h"
#include "get_command.h"
#include "status.h"
extern double referance_tilt_angle[3],referance_tilt_angle_Optimized[3];
extern enum  Sign sign_of_gravity,sign_of_acceleration,sign_of_side_axes;
extern enum AXES forward_axes,gravity_axes,side_axes;
extern int16_t xl_axes_ref[3];
extern volatile char instruction_final[25];
extern char out_string[200];
extern volatile uint8_t memrx,memtx,uarttx_req,sleep_dur_device;
extern uint8_t who_am_i,tilt_threshold,rtt;
extern float mg_per_LSB,rtt_optimized;
extern uint16_t odr_xl_value,hbt,hat,harsh_accel_brake_function_wakeup_threshold;
extern enum Wakeup_Device_Mode device_mode;

void USER_GET_Command(void)
{


	uint8_t ctrl1_xl, wake_up_ths,wake_up_dur,tap_cfg,ctrl8_xl,sleep_dur;


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\n\r\n",strlen("\r\n\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,instruction_final,strlen(instruction_final))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	if(!strncmp(instruction_final+4,"RTT",3))
	{
		sprintf(out_string,"\r\nRASH Turn Threshold is %d and Now it is Optimized to %d\r\n",rtt,(int8_t)rtt_optimized);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"HAT",3))
	{
		sprintf(out_string,"\r\nHARSH Acceleration Threshold is %d mg\r\nFunction Wake-up Threshold Selected is %d mg\r\n",hat,harsh_accel_brake_function_wakeup_threshold);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

		memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"HBT",3))
	{
		sprintf(out_string,"\r\nHARSH Brake Threshold is %d mg\r\nFunction Wake-up Threshold Selected is %d mg\r\n",hbt,harsh_accel_brake_function_wakeup_threshold);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

		memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"TILT_THRESHOLD",14))
	{
		sprintf(out_string,"\r\n Tilt Angle Threshold is %d degree\r\n",tilt_threshold);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);


		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"DEVICE_MODE",11))
	{

		switch(device_mode)
		{
		case SLEEP:
			sprintf(out_string,"\r\nDevice in    SLEEP	Mode\r\n");

			break;
		case FORWARD_AXIS:sprintf(out_string,"\r\nDevice in    FORWARD AXES	Mode\r\n");

		break;
		case ACCELERATION:sprintf(out_string,"\r\nDevice in    ACCELERATION	Mode\r\n");


		break;
		case ORIENTATION:sprintf(out_string,"\r\nDevice in    ORIENTATION	Mode\r\n");

		break;
		default:sprintf(out_string,"\r\nSome Logical Error by DEVICE_MODE command\r\n");
		}
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"DEVICE_ORIENTATION",18))
	{
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);

		if(gravity_axes	==	AXES_NOT_SET)
		{

			sprintf(out_string,"\r\nReferance Value are 0 for All AXES of Accelerometer\r\nReferance Value are 0 for All AXES of Gyroscope\r\nReferance Tilt Angle are 0 for All AXES\r\n");

			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);


		}
		else if(gravity_axes	!=	AXES_NOT_SET	&&	forward_axes == AXES_NOT_SET)
		{
			sprintf(out_string,"\r\n X - 0	Y - 1	Z - 2 \r\nForward / side axes 1 of Accelerometer is %d and forward / Side axes 2 is  %d Gravity axes is %d\r\n",(gravity_axes+1)%3,(gravity_axes+2)%3,gravity_axes);

			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			sprintf(out_string,"\r\nReferance Value of Gravity axes = %d , Forward / Side axes 1 = %d , Forward / Side axes 2 = %d\r\n",xl_axes_ref[gravity_axes],xl_axes_ref[(gravity_axes+1)%3],xl_axes_ref[(gravity_axes+2)%3]);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);


			sprintf(out_string,"\r\nReferance of Gravity Angle = %d , Forward / Side Angle 1 = %d , Forward / Side Angle 2 = %d\r\n",(int16_t)referance_tilt_angle[gravity_axes],(int16_t)referance_tilt_angle[(gravity_axes+1)%3],(int16_t)referance_tilt_angle[(gravity_axes+2)%3]);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);

		}
		else if(gravity_axes	!=	AXES_NOT_SET	&&	forward_axes!=AXES_NOT_SET)
		{

			sprintf(out_string,"\r\n X - 0	Y - 1	Z - 2 \r\nForward axes of Accelerometer is %d and  Side axes is  %d Gravity axes is %d\r\n",forward_axes,side_axes,gravity_axes);

			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			sprintf(out_string," \r\n Reference Values of Accelerometer	Forward axis = %d		Side axis = %d	Gravity axis = %d \r\n",*(xl_axes_ref+forward_axes),*(xl_axes_ref+side_axes),*(xl_axes_ref+gravity_axes));

			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);

			sprintf(out_string,"\r\nReferance of Forward Angle = %d	Side Angle = %d	Gravity Angle = %d\r\n",(int16_t)referance_tilt_angle_Optimized[forward_axes],(int16_t)referance_tilt_angle_Optimized[side_axes],(int16_t)referance_tilt_angle_Optimized[gravity_axes]);


			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);

		}


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"SLEEP_DUR_DEVICE",16))
	{


		sprintf(out_string,"\r\n Duration for Device turn to Sleep Mode %d s\r\n",sleep_dur_device);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		memset(instruction_final,0,25);

	}

	if(!strncmp(instruction_final+4,"SLEEP_DUR",9))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&wake_up_dur,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);


		sleep_dur=wake_up_dur&0x0f;


		sprintf(out_string,"\r\n Duration for Inactivity detection is %d ms\r\n",sleep_dur==0 && who_am_i==0x6a?16000/odr_xl_value:(int)sleep_dur*512*1000/odr_xl_value);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);


		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		memset(instruction_final,0,25);



	}








	if(!strncmp(instruction_final+4,"FILTER_CUTOFF",13))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ctrl8_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);



		switch(ctrl8_xl&0b01100000)
		{
		case 0:




			sprintf(out_string,"\r\nSlope Filter Cut-Off Frequency is %d \r\n",(int)odr_xl_value/4);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);
			break;

		case 0b00100000:




			sprintf(out_string,"\r\nHigh-Pass Filter Cut-Off Frequency is %d \r\n",(int)odr_xl_value/100);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);
			break;

		case 0b01000000:


			sprintf(out_string,"\r\nHigh-Pass Filter Cut-Off Frequency is %d \r\n",(int)odr_xl_value/9);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);


			break;
		case 0b01100000:




			sprintf(out_string,"\r\nHigh-Pass Filter Cut-Off Frequency is %d \r\n",(int)odr_xl_value/400);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);


			break;



		}

	}

	if(!strncmp(instruction_final+4,"FILTER",6))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&tap_cfg,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		tap_cfg&=0b00010000;


		if(tap_cfg==0)
		{

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nthe wake-up feature implemented using the slope filter \r\n",strlen("\r\nthe wake-up feature implemented using the slope filter \r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

		}
		if(tap_cfg==0b00010000)
		{

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nthe wake-up feature implemented using the High-Pass  filter\r\n",strlen("\r\nthe wake-up feature implemented using the High-Pass  filter\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

		}


	}

	if(!strncmp(instruction_final+4,"ACCEL_SCALE",11))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		ctrl1_xl&=0b1100;



		switch(ctrl1_xl)
		{
		case 0:

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer full-scale value now is +/- 2 g \r\n",strlen("\r\nAccelerometer full-scale value now is +/- 8 g \r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);




			break;
		case 0B1000:

			mg_per_LSB =  0.122;
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer full-scale value now is +/- 4 g \r\n",strlen("\r\nAccelerometer full-scale value now is +/- 4 g \r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;
		case 0B1100:

			mg_per_LSB = 0.244;
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer full-scale value now is +/- 8 g \r\n",strlen("\r\nAccelerometer full-scale value now is +/- 8 g \r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;
		case 0B0100:

			mg_per_LSB =  0.488;
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer full-scale value now is +/- 16 g \r\n",strlen("\r\nAccelerometer full-scale value now is +/- 16 g \r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;



		}


	}




	if(!strncmp(instruction_final+4,"WK_THS",6))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&wake_up_ths,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);


		wake_up_ths&=0b00111111;


		sprintf(out_string,"\r\nActivity/Inactivity threshold Now is %d mg \r\n",((int)(wake_up_ths*32768*mg_per_LSB)/64));

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		memset(instruction_final,0,25);


	}

	if(!strncmp(instruction_final+4,"WAKE_DUR",8))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&wake_up_dur,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);


		wake_up_dur&=0b01100000;
		wake_up_dur>>=5;


		sprintf(out_string,"\r\n Duration for Activity detection is %d us\r\n",(int)wake_up_dur*1000*1000/odr_xl_value);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
		memset(instruction_final,0,25);


	}






	if(!strncmp(instruction_final+4,"ODR_XL",6))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		ctrl1_xl&=0xf0;
		ctrl1_xl>>=4;


		switch(ctrl1_xl)
		{
		case 0:


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer is in Powerdown Mode &Output data rate is 0 Hz\r\n",strlen("\r\nAccelerometer is in Powerdown Mode &Output data rate is 0 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);


			break;
		case 1:


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 13 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 13 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);


			break;
		case 2:


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 26 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 26 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);
			break;

		case 3:


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 52 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 52 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);
			break;

		case 4:


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 104 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 104 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);
			break;

		case 5:

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 208 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 208 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);


			break;
		case 6:

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 416 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 416 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;
		case 7:

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 833 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 833 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;
		case 8:

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 1660 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 1660 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;
		case 9:

			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 3330 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 3330 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;
		case 10:


			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 6660 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 6660 Hz\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;




		}




	}

}

