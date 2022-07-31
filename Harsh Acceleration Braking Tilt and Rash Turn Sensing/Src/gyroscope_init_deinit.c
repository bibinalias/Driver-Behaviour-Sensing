/*
 * gyroscope_init.c
 *
 *  Created on: 21-Aug-2020
 *      Author: Bibin
 */

#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "gyroscope_init_deinit.h"


static uint8_t gyro_ctrl10_c,gyro_ctrl7_g,gyro_ctrl4_c,gyro_ctrl6_c,gyro_ctrl2_g;

extern volatile uint8_t memtx,memrx,uarttx_req;
extern uint8_t who_am_i;
extern float mdps_per_LSB;

void USER_Device_Orientation_Gyroscope_Init(void)
{
	uint8_t hexcode;


	if(who_am_i==0x69)
	{

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl4_c&(~0x48);
		hexcode|=0x08;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//Data-ready mask enable. If enabled, when switching from Power-Down to an
			//active mode, the accelerometer and gyroscope data-ready signals are masked
			//until the settling of the sensor filters is completed.
			//Gyroscope sleep mode disable.
		{
			Error_Handler();
		}


		while(memtx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL10_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl10_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl10_c&~0x38;


		hexcode|=0x38;																// Gyro X, Y, Z axes enabled

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL10_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl7_g,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl7_g&~0xf8;
		hexcode|=0x60;						//high-performance operating mode enabled,HPF enabled,HPF Cutoff 2.07 Hz

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);
	}
	if(who_am_i==0x6a)
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl7_g,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl7_g&~0xf0;
		hexcode|=0x70;	//high-performance operating mode enabled,HPF enabled,HPF Cutoff 1.04 Hz
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl4_c&~0b01001010;
		hexcode|=0x0A;	//Gyroscope sleep mode disabled,Enable gyroscope digital LPF1.,Configuration 1 data available enable bit,
		//Data-ready mask enable. If enabled, when switching from Power-Down to an
		//active mode, the accelerometer and gyroscope data-ready signals are masked
		//until the settling of the sensor filters is completed.


		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL6_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl6_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl6_c&~0x03;
		hexcode|=0x02;//LPF Cutoff 4
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL6_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

	}
	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL2_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl2_g,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=gyro_ctrl2_g&0x1;
	hexcode|=0x12;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL2_G,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//// Gyro = 12.5Hz (High-Performance mode),125dps
	{
		Error_Handler();
	}


	while(memtx==0);



	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nGyroscope Output filter Initialized for Device Orientation......\r\n",strlen("\r\nGyroscope Output filter Initialized for Device Orientation......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	HAL_Delay(160);
}

void USER_Rash_Turn_Gyroscope_Init(void)
{
	uint8_t hexcode;


	if(who_am_i==0x69)
	{

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl4_c&(~0x48);
		hexcode|=0x08;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//Data-ready mask enable. If enabled, when switching from Power-Down to an
			//active mode, the accelerometer and gyroscope data-ready signals are masked
			//until the settling of the sensor filters is completed.
			//Gyroscope sleep mode disable.
		{
			Error_Handler();
		}


		while(memtx==0);
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL10_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl10_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl10_c&~0x38;


		hexcode|=0x38;																// Gyro X, Y, Z axes enabled

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL10_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl7_g,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl7_g&~0xf8;
		hexcode|=0x58;						//high-performance operating mode enabled,HPF enabled,HPF Cutoff 0.0324 Hz

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);
	}
	if(who_am_i==0x6a)
	{
		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl7_g,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl7_g&~0xf0;
		hexcode|=0x50;	//high-performance operating mode enabled,HPF enabled,HPF Cutoff 65 mHz
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl4_c&~0b01001010;
		hexcode|=0x0A;	//Gyroscope sleep mode disabled,Enable gyroscope digital LPF1.,Configuration 1 data available enable bit,
		//Data-ready mask enable. If enabled, when switching from Power-Down to an
		//active mode, the accelerometer and gyroscope data-ready signals are masked
		//until the settling of the sensor filters is completed.


		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL6_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl6_c,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=gyro_ctrl6_c&~0x03;
		hexcode|=0x02;//LPF Cutoff 4
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL6_C,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);

	}
	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL2_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl2_g,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=gyro_ctrl2_g&0x1;
	hexcode|=0x12;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL2_G,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)//// Gyro = 12.5Hz (High-Performance mode),125dps
	{
		Error_Handler();
	}
	else
	{
		mdps_per_LSB=4.375;
	}

	while(memtx==0);



	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nGyroscope Output filter Initialized for RASH Turn Sensing......\r\n",strlen("\r\nGyroscope Output filter Initialized for RASH Turn Sensing......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	HAL_Delay(160);
}


void USER_Device_Orientation_Gyroscope_DeInit(void)
{


	if(who_am_i==0x69)
	{
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL10_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl10_c,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl7_g,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);
	}

	if(who_am_i==0x6A)
	{
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL7_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl7_g,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL4_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl4_c,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL6_C,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl6_c,1)!=HAL_OK)
		{
			Error_Handler();
		}


		while(memtx==0);


	}
	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL2_G,I2C_MEMADD_SIZE_8BIT,&gyro_ctrl2_g,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memtx==0);



	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nGyroscope Output filter Initialized to Previous STATE......\r\n",strlen("\r\nGyroscope Output filter Initialized to Previous STATE......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


}
