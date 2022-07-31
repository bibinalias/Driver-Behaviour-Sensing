/*
 * device_orientation_init.c
 *
 *  Created on: 20-Aug-2020
 *      Author: Bibin
 */

#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "stm32f0xx_hal_def.h"
#include "device_orientation_wakeup_init_deinit.h"


extern volatile uint8_t memtx,memrx,uarttx_req;
extern uint8_t who_am_i;
static uint8_t ori_wake_ctrl1_xl,ori_wake_tap_cfg,ori_wake_wake_up_ths,ori_wake_ctrl8_xl,ori_wake_wake_up_dur,ori_wake_md1_cfg;

void USER_Device_Orientation_WakeUp_init(void)
{


	uint8_t hexcode;
	while(HAL_I2C_IsDeviceReady(&hi2c2,LSM6DS3,100,1000)!=HAL_OK);//check is Device Ready
	memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ori_wake_ctrl1_xl,1)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);

		hexcode=ori_wake_ctrl1_xl&~0xfc;
		hexcode|=0x60;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Turn on the accelerometer
		// ODR_XL = 416 Hz, FS_XL = 2g
	{
		Error_Handler();
	}

		while(memtx==0);

	if(who_am_i==0x69)
	{

		memrx=0;
				if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&ori_wake_tap_cfg,1)!=HAL_OK)
				{
					Error_Handler();
				}

				while(memrx==0);

				hexcode=ori_wake_tap_cfg&~0x11;
				hexcode|=0x11;

		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Apply HP filter; latch mode Enabled
		{
			Error_Handler();
		}


		while(memtx==0);

	}



	if(who_am_i==0x6A)
	{
		memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&ori_wake_tap_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=ori_wake_tap_cfg&~0x91;
	hexcode|=0x91;
		memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Interrupt Enabled ; Apply HP filter; latch mode Enabled;
		{
			Error_Handler();
		}
		while(memtx==0);


	}
	memrx=0;
			if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&ori_wake_wake_up_ths,1)!=HAL_OK)
			{
				Error_Handler();
			}

			while(memrx==0);

			hexcode=ori_wake_wake_up_ths&~0x3f;
			hexcode|=0x01;
	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Set wake-up threshold
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
			if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ori_wake_ctrl8_xl,1)!=HAL_OK)
			{
				Error_Handler();
			}

			while(memrx==0);

			if(who_am_i==0x69)
			{
				hexcode=ori_wake_ctrl8_xl&~0x60;

				hexcode|=0x60; //HPF cutoff odr/400
			}
			if(who_am_i==0x6a)
			{
				hexcode=ori_wake_ctrl8_xl&~0x68;

				hexcode|=0x60;//HPF cutoff odr/400;ODR/2 low pass filtered sent to composite filter
			}

	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
			if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&ori_wake_wake_up_dur,1)!=HAL_OK)
			{
				Error_Handler();
			}

			while(memrx==0);

			hexcode=ori_wake_wake_up_dur&~0x60;
			hexcode|=0;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// No duration for wake up
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
			if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&ori_wake_md1_cfg,1)!=HAL_OK)
			{
				Error_Handler();
			}

			while(memrx==0);

			hexcode=ori_wake_md1_cfg&~0x20;
			hexcode|=0x20;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Wake-up interrupt driven to INT1 pin
	{
		Error_Handler();
	}


	while(memtx==0);


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nDevice Orientation Function Started..... \r\n\r\nWake up Function Initiated......\r\n",strlen("\r\nDevice Orientation Function Started..... \r\n\r\nWake up Function Initiated......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);





}



void USER_Device_Orientation_WakeUp_DeInit(void)
{
	uint8_t wake_up_src;
	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&ori_wake_ctrl1_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}
	else

		while(memtx==0);





	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&ori_wake_tap_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}


	while(memtx==0);







	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&ori_wake_wake_up_ths,1)!=HAL_OK)
	{
		Error_Handler();
	}


	while(memtx==0);



	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&ori_wake_ctrl8_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}


	while(memtx==0);



	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&ori_wake_wake_up_dur,1)!=HAL_OK)
	{
		Error_Handler();
	}


	while(memtx==0);



	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&ori_wake_md1_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}


	while(memtx==0);





	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nWakeup Function Initialized to Previous STATE......\r\n\r\nExit from Device Orientation Function......\r\n",strlen("\r\nWakeup Function Initialized to Previous STATE......\r\n\r\nExit from Device Orientation Function......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	for(uint8_t i=0;i<3;i++)
	{
	HAL_Delay(300);
	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	}



}
