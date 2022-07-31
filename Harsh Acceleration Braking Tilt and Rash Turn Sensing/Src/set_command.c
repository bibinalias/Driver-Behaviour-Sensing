

#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "set_command.h"
#include "status.h"



extern enum Function_Exicution forward_axes_wakeup_init_function_status,acceleration_wakeup_init_function_status;
extern  float total_error_xl_mg[3];
extern int8_t total_error_int_xl[3];
extern volatile char instruction_final[25];
extern char out_string[200];
extern volatile uint8_t memrx,memtx,uarttx_req,sleep_dur_device;
extern float mg_per_LSB;
extern uint16_t odr_xl_value,hbt,hat,harsh_accel_brake_function_wakeup_threshold;
extern uint8_t who_am_i,tilt_threshold,rtt;
extern enum AXES forward_axes;
extern enum  Sign sign_of_acceleration;
extern enum Wakeup_Device_Mode device_mode;
extern enum Acceleration_Brake harsh_accel_brake_function_wakeup_mode;

void USER_SET_Command(void)
{

	uint8_t ctrl1_xl, wake_up_ths,wk_ths,wake_up_dur,wake_dur,tap_cfg,ctrl8_xl,sleep_dur,wake_up_src;


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


	if(!strncmp(instruction_final+4,"RTT:",4))
	{
		rtt=atoi(instruction_final+8)	<=	125	&& atoi(instruction_final+8) > 0?atoi(instruction_final+8)	:	rtt;
		RASH_Turn_Threshold_Optimize();
		memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"HAT:",4))
	{
		hat=atoi(instruction_final+8)	<	32768*mg_per_LSB &&	atoi(instruction_final+8) > 31 ?atoi(instruction_final+8)	:	hat;

		if(hat	<=	hbt)
		{
			harsh_accel_brake_function_wakeup_threshold	=	hat;
			harsh_accel_brake_function_wakeup_mode		=HARSH_ACCELERATION;
		}
		else
		{
			harsh_accel_brake_function_wakeup_threshold	=	hbt;
			harsh_accel_brake_function_wakeup_mode		=HARSH_BRAKING;
		}

		sprintf(out_string,"\r\nHARSH Acceleration Threshold is %d mg\r\nFunction Wake-up Threshold Selected is %d mg\r\n",hat,harsh_accel_brake_function_wakeup_threshold);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

		if(device_mode==ACCELERATION)
		{
			USER_Acceleration_Wakeup_Init();
		}
		memset(instruction_final,0,25);
	}
	if(!strncmp(instruction_final+4,"HBT:",4))
	{
		hbt=atoi(instruction_final+8)	<	32768*mg_per_LSB &&	atoi(instruction_final+8) > 31 ?atoi(instruction_final+8)	:	hbt;

		if(hat	<=	hbt)
		{
			harsh_accel_brake_function_wakeup_threshold	=	hat;
			harsh_accel_brake_function_wakeup_mode		=HARSH_ACCELERATION;
		}
		else
		{
			harsh_accel_brake_function_wakeup_threshold	=	hbt;
			harsh_accel_brake_function_wakeup_mode		=HARSH_BRAKING;
		}
		sprintf(out_string,"\r\nHARSH Brake Threshold is %d mg\r\nFunction Wake-up Threshold Selected is %d mg\r\n",hbt,harsh_accel_brake_function_wakeup_threshold);

		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

		if(device_mode==ACCELERATION)
		{
			USER_Acceleration_Wakeup_Init();
		}
		memset(instruction_final,0,25);
	}

	if(!strncmp(instruction_final+4,"TILT_THRESHOLD:",15))
	{
		tilt_threshold	=atoi(instruction_final+19)<=180	&&	atoi(instruction_final+19)>=0?atoi(instruction_final+19):tilt_threshold;

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
	if(!strncmp(instruction_final+4,"DEVICE_TO_SLEEP",15))
	{
		USER_Sleep_Enter();
		memset(instruction_final,0,25);
	}


	if(!strncmp(instruction_final+4,"DEVICE_ORIENTATION",18))
	{
		forward_axes=AXES_NOT_SET;
		sign_of_acceleration=SIGN_NOT_SET;

		USER_Device_Orientation_WakeUp_init();
		USER_Device_Orientation_Accelerometer_Init();
		//USER_Device_Orientation_Gyroscope_Init();
		USER_Accelerometer_Referance_Value_Calculation();
		//USER_Gyroscope_Referance_Value_Calculation();
		USER_Referance_Tilt_Calculation();
		RASH_Turn_Threshold_Optimize();
		//USER_Device_Orientation_Gyroscope_DeInit();
		USER_Device_Orientation_Accelerometer_DeInit();
		USER_Device_Orientation_WakeUp_DeInit();



		forward_axes_wakeup_init_function_status=NOT_EXICUTED;
		acceleration_wakeup_init_function_status==NOT_EXICUTED;
		acceleration_wakeup_init_function_status=NOT_EXICUTED;

		memset(total_error_int_xl,0,sizeof(total_error_int_xl));
		memset(total_error_xl_mg,0,sizeof(total_error_xl_mg));

		if(device_mode	==	ACCELERATION)
		{
			device_mode=FORWARD_AXIS;
		}


		memset(instruction_final,0,25);
	}

	if(!strncmp(instruction_final+4,"SLEEP_DUR_DEVICE:",17))
	{

		sleep_dur_device=atoi(instruction_final+21);

		if (sleep_dur_device>=0 && sleep_dur_device<256)
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





	}
	if(!strncmp(instruction_final+4,"SLEEP_DUR:",10))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&wake_up_dur,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);




		sleep_dur=atoi(instruction_final+14);

		if (sleep_dur>=0 && sleep_dur<16)
		{
			memtx=0;
			wake_up_dur&=~0b00001111;
			wake_up_dur|=sleep_dur;


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&wake_up_dur,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else
			{
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


			while(memtx==0);


		}





	}
	if(!strncmp(instruction_final+4,"FILTER_CUTOFF:",14))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ctrl8_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		ctrl8_xl&=~0b01100000;


		switch(atoi(instruction_final+18))
		{
		case 0:


			ctrl8_xl|=0b00000000;
			memtx=0;
			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ctrl8_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{

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
			}

			while(memtx==0);

			break;
		case 1:


			ctrl8_xl|=0b00100000;
			memtx=0;
			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ctrl8_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{

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
			}

			while(memtx==0);

			break;
		case 2:


			ctrl8_xl|=0b01000000;
			memtx=0;
			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ctrl8_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{

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
			}

			while(memtx==0);

			break;
		case 3:


			ctrl8_xl|=0b01100000;
			memtx=0;
			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ctrl8_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{

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
			}

			while(memtx==0);

			break;
		default:
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nIncurrect Cut-Off Frequency\r\n",strlen("\r\nIncurrect Cut-Off Frequency\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);


		}
		HAL_Delay((int)1000/odr_xl_value);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

	}
	if(!strncmp(instruction_final+4,"FILTER:",7))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&tap_cfg,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
		tap_cfg&=~0b00010000;


		if(!strncmp(instruction_final+11,"SLOPE",5))
		{
			tap_cfg|=0;

			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&tap_cfg,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else
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

		}
		if(!strncmp(instruction_final+11,"HIGH-PASS",9))
		{
			tap_cfg|=0b00010000;
			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&tap_cfg,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else
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
		HAL_Delay((int)(1000/odr_xl_value)+1);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);



	}

	if(!strncmp(instruction_final+4,"ACCEL_SCALE:",12))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);




		switch(atoi(instruction_final+16))
		{
		case 2:
			memtx=0;
			ctrl1_xl&=~0b00001100;
			ctrl1_xl|=0b00000000;


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				mg_per_LSB =0.061;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer full-scale value now is +/- 2 g \r\n",strlen("\r\nAccelerometer full-scale value now is +/- 8 g \r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 4:
			memtx=0;
			ctrl1_xl&=~0b00001100;
			ctrl1_xl|=0b00001000;


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else
			{
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
			}

			while(memtx==0);
			break;
		case 8:
			memtx=0;
			ctrl1_xl&=~0b00001100;
			ctrl1_xl|=0b00001100;


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
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
			}

			while(memtx==0);
			break;
		case 16:
			memtx=0;
			ctrl1_xl&=~0b00001100;
			ctrl1_xl|=0b00000100;


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else
			{
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
			}


			while(memtx==0);

			break;
		default:
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer full-scale value is incorrect\r\n",strlen("\r\n\r\nAccelerometer full-scale value is incorrect"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;


		}
		HAL_Delay((int)(1000/odr_xl_value)+1);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);



	}




	if(!strncmp(instruction_final+4,"WK_THS:",7))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&wake_up_ths,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);




		wk_ths=atoi(instruction_final+11);

		if (wk_ths>0 && wk_ths<64)
		{
			memtx=0;
			wake_up_ths&=~0b00111111;
			wake_up_ths|=wk_ths;


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&wake_up_ths,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else
			{
				sprintf(out_string,"\r\nActivity/Inactivity threshold Now is %d mg \r\n",((int)(wk_ths*32768*mg_per_LSB)/64));

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


			while(memtx==0);


		}





	}

	if(!strncmp(instruction_final+4,"WAKE_DUR:",9))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&wake_up_dur,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);




		wake_dur=atoi(instruction_final+13);

		if (wake_dur>=0 && wake_dur<4)
		{
			memtx=0;
			wake_up_dur&=~0b01100000;
			wake_up_dur|=(wake_dur<<5);


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&wake_up_dur,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else
			{
				sprintf(out_string,"\r\n Duration for Activity detection is %d us\r\n",(int)wake_dur*1000*1000/odr_xl_value);

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


			while(memtx==0);


		}





	}





	if(!strncmp(instruction_final+4,"ODR_XL:",7))
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);




		switch(atoi(instruction_final+11))
		{
		case 0:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0b00000000;


			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=0;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer is in Powerdown Mode &Output data rate is 0 Hz\r\n",strlen("\r\nAccelerometer is in Powerdown Mode &Output data rate is 0 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 13:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x10;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=13;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 13 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 13 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 26:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x20;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=26;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 26 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 26 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 52:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x30;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=52;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 52 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 52 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 104:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x40;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=104;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 104 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 104 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 208:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x50;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=208;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 208 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 208 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 416:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x60;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=416;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 416 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 416 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 833:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x70;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=833;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 833 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 833 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 1660:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x80;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=1660;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 1660 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 1660 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 3330:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0x90;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=3330;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 3330 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 3330 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;
		case 6660:
			memtx=0;
			ctrl1_xl&=~0xf0;
			ctrl1_xl|=0xa0;



			if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ctrl1_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}
			else

			{
				odr_xl_value=6660;
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
				uarttx_req=1;
				if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is 6660 Hz\r\n",strlen("\r\nAccelerometer Output data rate is 6660 Hz\r\n"))!=HAL_OK)
				{
					Error_Handler();
				}
				while(uarttx_req==1);
				HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
				memset(instruction_final,0,25);
			}

			while(memtx==0);

			break;

		default:
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output data rate is incorrect\r\n",strlen("\r\nAccelerometer Output data rate is incorrect\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);
			HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
			memset(instruction_final,0,25);

			break;


		}

		HAL_Delay(80);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);



	}






}
