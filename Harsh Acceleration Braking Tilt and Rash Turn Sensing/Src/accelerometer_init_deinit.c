/*
 * accelerometer_init.c
 *
 *  Created on: 20-Aug-2020
 *      Author: Bibin
 */


#include "accelerometer_init_deinit.h"
#include "i2c.h"
#include "lsm6ds3_l.h"
#include "usart.h"


extern volatile uint8_t memtx,uarttx_req;
extern uint8_t who_am_i;
extern volatile uint8_t memrx;
static uint8_t acc_ctrl9_xl,acc_ctrl4_c,acc_ctrl8_xl,acc_ctrl1_xl;


void USER_Device_Orientation_Accelerometer_Init(void)
{

	uint8_t hexcode;


	if(who_am_i==0x69)
	{


		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL9_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl9_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl9_xl&~0x38;
		hexcode|=0x38;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL9_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)   // Acc X, Y, Z axes enabled
		{
			Error_Handler();
		}

		while(memtx==0);


		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&acc_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl4_c&~0x88;
		hexcode|=0x88;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//bandwidth determined by setting BW_XL[1:0] in CTRL1_XL (10h) register.),Data-ready mask enable. If enabled, when switching from Power-Down to an
			//active mode, the accelerometer and gyroscope data-ready signals are masked
			//until the settling of the sensor filters is completed.
		{
			Error_Handler();
		}


		while(memtx==0);


	}

	if(who_am_i==0x6a)
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&acc_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl4_c&~0x08;
		hexcode|=0x08;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//Data-ready mask enable. If enabled, when switching from Power-Down to an
			//active mode, the accelerometer and gyroscope data-ready signals are masked
			//until the settling of the sensor filters is completed.
		{
			Error_Handler();
		}


		while(memtx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl8_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl8_xl&~0b11101100;
		hexcode|=0xe8;

		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//Accelerometer low-pass filter LPF2 selection.
			//cut off frequency =ODR/400
			//ODR/4 low pass filtered sent to composite filter)  ;LOW NOISE
		{
			Error_Handler();
		}


		while(memtx==0);


		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl1_xl&~0xff;
		hexcode|=0x60;
		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Turn on the accelerometer
				// ODR_XL = 416 Hz, FS_XL = 2g
			//Anti-aliasing filter bandwidth selection.
		{
			Error_Handler();
		}


		while(memtx==0);
	}
	if(who_am_i==0x69)
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl8_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl8_xl&~0b11100100;
		hexcode|=0xe4;


		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//if both the LPF2_XL_EN bit and the HP_SLOPE_XL_EN bit are set to 1, the LP digital
			//filter (LPF2) is applied;Accelerometer low-pass filter LPF2 selection.
			//cut off ODR/400
			//Accelerometer slope filter / high-pass filter selection.
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl1_xl&~0xff;
		hexcode|=0x63;
		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Turn on the accelerometer
				// ODR_XL = 416 Hz, FS_XL = 2g
			//Anti-aliasing filter bandwidth selection. value: 11 FOR 50Hzin lsm6ds3
		{
			Error_Handler();
		}


		while(memtx==0);
	}



	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output filter Initialized......\r\n",strlen("\r\nAccelerometer Output filter Initialized......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	HAL_Delay(770);

}





void USER_Forward_axes_and_Tilt_Accelerometer_Init(void)
{

	uint8_t hexcode;


	if(who_am_i==0x69)
	{


		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL9_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl9_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl9_xl&~0x38;
		hexcode|=0x38;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL9_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)   // Acc X, Y, Z axes enabled
		{
			Error_Handler();
		}

		while(memtx==0);


		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&acc_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl4_c&~0x88;
		hexcode|=0x88;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
			//bandwidth determined by setting BW_XL[1:0] in CTRL1_XL (10h) register.),Data-ready mask enable. If enabled, when switching from Power-Down to an
			//active mode, the accelerometer and gyroscope data-ready signals are masked
			//until the settling of the sensor filters is completed.
		{
			Error_Handler();
		}


		while(memtx==0);


	}

	if(who_am_i==0x6a)
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&acc_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl4_c&~0x08;
		hexcode|=0x08;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
			//Data-ready mask enable. If enabled, when switching from Power-Down to an
			//active mode, the accelerometer and gyroscope data-ready signals are masked
			//until the settling of the sensor filters is completed.
		{
			Error_Handler();
		}


		while(memtx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl8_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl8_xl&~0b11101100;
		hexcode|=0b11001000;

		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//Accelerometer low-pass filter LPF2 selection.
			//cut off frequency =ODR/9
			//ODR/4 low pass filtered sent to composite filter  ;LOW NOISE
		{
			Error_Handler();
		}


		while(memtx==0);


		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl1_xl&~0xff;
		hexcode|=0x60;
		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Turn on the accelerometer
																								// ODR_XL = 416 Hz, FS_XL = 2g

		{
			Error_Handler();
		}


		while(memtx==0);
	}
	if(who_am_i==0x69)
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl8_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl8_xl&~0b11100100;
		hexcode|=0b11000100;


		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//if both the LPF2_XL_EN bit and the HP_SLOPE_XL_EN bit are set to 1, the LP digital
			//filter (LPF2) is applied;Accelerometer low-pass filter LPF2 selection.
			//cut off ODR/9
			//Accelerometer slope filter / high-pass filter selection.
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=acc_ctrl1_xl&~0xff;
		hexcode|=0x63;
		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Turn on the accelerometer
				// ODR_XL = 416 Hz, FS_XL = 2g
			//Anti-aliasing filter bandwidth selection. value: 11 FOR 50Hzin lsm6ds3
		{
			Error_Handler();
		}


		while(memtx==0);
	}



	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output filter Initialized for Forward Axes and Tilt Recognition......\r\n",strlen("\r\nAccelerometer Output filter Initialized for Forward Axes and Tilt Recognition......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


	HAL_Delay(770);

}


















void USER_Device_Orientation_Accelerometer_DeInit(void)
{
	if(who_am_i==0x69)
	{
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL9_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl9_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memtx==0);




	}
	memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&acc_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);



		memtx=0;
		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl8_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);



	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&acc_ctrl1_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}


	while(memtx==0);


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nAccelerometer Output filter Initialized to Previous STATE......\r\n",strlen("\r\nAccelerometer Output filter Initialized to Previous STATE......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

}




