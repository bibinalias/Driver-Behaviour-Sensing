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
#include "forward_axes_sense_wakeup_init.h"
#include "status.h"

extern volatile uint8_t memtx,memrx,uarttx_req;
extern enum Wakeup_Device_Mode device_mode,wakeup_mode;
extern enum Function_Exicution forward_axes_wakeup_init_function_status;

extern uint8_t who_am_i;
static uint8_t fwd_axes_ctrl1_xl,fwd_axes_tap_cfg,fwd_axes_wake_up_ths,fwd_axes_ctrl8_xl,fwd_axes_wake_up_dur,fwd_axes_md1_cfg;

void USER_forward_axes_sense_Wakeup_Init(void)
{
	uint8_t hexcode;
	extern float mg_per_LSB;
	extern uint16_t odr_xl_value;

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&fwd_axes_ctrl1_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=fwd_axes_ctrl1_xl&~0xfc;
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
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&fwd_axes_tap_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(who_am_i==0x69)
	{



		hexcode=fwd_axes_tap_cfg&~0x11;
		hexcode|=0x11;//latch enabled,Apply HPF filter
	}



	if(who_am_i==0x6A)
	{


		hexcode=fwd_axes_tap_cfg&~0x91;
		hexcode|=0x91;// Interrupt Enabled ; Apply HP filter; latch mode Enabled;
	}

	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
	{
		Error_Handler();
	}
	while(memtx==0);



	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&fwd_axes_wake_up_ths,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=fwd_axes_wake_up_ths&~0x3f;
	hexcode|=0x03;
	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Set wake-up threshold as 93 mg
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&fwd_axes_ctrl8_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(who_am_i==0x69)
	{
		hexcode=fwd_axes_ctrl8_xl&~0x60;

		hexcode|=0x40; //HPF cutoff odr/9
	}
	if(who_am_i==0x6a)
	{
		hexcode=fwd_axes_ctrl8_xl&~0x68;

		hexcode|=0x40;//HPF cutoff odr/9;ODR/2 low pass filtered sent to composite filter
	}

	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
			{
		Error_Handler();
			}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&fwd_axes_wake_up_dur,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=fwd_axes_wake_up_dur&~0x60;
	hexcode|=0x60;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Maximum (ODR*3) duration for wake up
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&fwd_axes_md1_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=fwd_axes_md1_cfg&~0x20;
	hexcode|=0x20;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Wake-up interrupt driven to INT1 pin
	{
		Error_Handler();
	}


	while(memtx==0);


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,"\r\nDevice Forward axes Sensing Function Started......\r\n",strlen("\r\nDevice Forward axes Sensing Function Started......\r\n"))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	forward_axes_wakeup_init_function_status=EXICUTED;

	wakeup_mode=FORWARD_AXIS;
	device_mode=FORWARD_AXIS;


}
