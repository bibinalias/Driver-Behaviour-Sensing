/*
 * wakeup_init.c
 *
 *  Created on: 25-Aug-2020
 *      Author: Bibin
 */
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "stm32f0xx_hal_def.h"
#include "wakeup_init.h"
#include "status.h"

extern volatile uint8_t memtx,memrx,uarttx_req;
extern enum Wakeup_Device_Mode device_mode,wakeup_mode;
extern uint8_t who_am_i;
static uint8_t dev_wake_ctrl1_xl,dev_wake_tap_cfg,dev_wake_wake_up_ths,dev_wake_ctrl8_xl,dev_wake_wake_up_dur,dev_wake_md1_cfg;
extern float mg_per_LSB;
extern uint16_t odr_xl_value;

void USER_Wakeup_Init(void)
{
	uint8_t hexcode;



	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&dev_wake_ctrl1_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=dev_wake_ctrl1_xl&~0xfc;
	hexcode|=0x60;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Turn on the accelerometer
		// ODR_XL = 416 Hz, FS_XL = 2g
	{
		Error_Handler();
	}
	else
	{
		odr_xl_value=416;
		mg_per_LSB = 0.061;
	}

	while(memtx==0);
	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&dev_wake_tap_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(who_am_i==0x69)
	{



		hexcode=dev_wake_tap_cfg&~0x11;
		hexcode|=0x11;//latch enabled,Apply HPF filter
	}



	if(who_am_i==0x6A)
	{


		hexcode=dev_wake_tap_cfg&~0x91;
		hexcode|=0x91;// Interrupt Enabled ; Apply HP filter; latch mode Enabled;
	}

	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
	{
		Error_Handler();
	}
	while(memtx==0);



	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&dev_wake_wake_up_ths,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=dev_wake_wake_up_ths&~0x3f;
	hexcode|=0x01;
	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Set wake-up threshold
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&dev_wake_ctrl8_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(who_am_i==0x69)
	{
		hexcode=dev_wake_ctrl8_xl&~0x60;

		hexcode|=0x40; //HPF cutoff odr/9
	}
	if(who_am_i==0x6a)
	{
		hexcode=dev_wake_ctrl8_xl&~0x68;

		hexcode|=0x40;//HPF cutoff odr/9;ODR/2 low pass filtered sent to composite filter
	}

	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
			{
		Error_Handler();
			}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&dev_wake_wake_up_dur,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=dev_wake_wake_up_dur&~0x60;
	hexcode|=0X40;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// ODR*2 duration for wake up
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&dev_wake_md1_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=dev_wake_md1_cfg&~0x20;
	hexcode|=0x20;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Wake-up interrupt driven to INT1 pin
	{
		Error_Handler();
	}


	while(memtx==0);


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\n\r\nDevice is Now in Sleep Mode\r\nDevice Wake up Function Initiated......\r\n\r\n",strlen("\r\n\r\nDevice is Now in Sleep Mode\r\nDevice Wake up Function Initiated......\r\n\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);
	wakeup_mode=SLEEP;
	device_mode=SLEEP;


}
