/*
 * acceleration_wakeup_init.c
 *
 *  Created on: 11-Sep-2020
 *      Author: Bibin
 */


#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
#include "stm32f0xx_hal_def.h"
#include "acceleration_wakeup_init.h"
#include "status.h"

extern volatile uint8_t memtx,memrx,uarttx_req;
extern enum Wakeup_Device_Mode device_mode,wakeup_mode;
extern enum Function_Exicution acceleration_wakeup_init_function_status;
extern char out_string[200];
extern uint8_t who_am_i;
extern float mg_per_LSB;
extern uint16_t odr_xl_value,harsh_accel_brake_function_wakeup_threshold;
static uint8_t accel_ctrl1_xl,accel_tap_cfg,accel_wake_up_ths,accel_ctrl8_xl,accel_wake_up_dur,accel_md1_cfg;

void USER_Acceleration_Wakeup_Init(void)
{
	uint8_t hexcode;


	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&accel_ctrl1_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=accel_ctrl1_xl&~0xfc;
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
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&accel_tap_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(who_am_i==0x69)
	{



		hexcode=accel_tap_cfg&~0x11;
		hexcode|=0x11;//latch enabled,Apply HPF filter
	}



	if(who_am_i==0x6A)
	{


		hexcode=accel_tap_cfg&~0x91;
		hexcode|=0x91;// Interrupt Enabled ; Apply HP filter; latch mode Enabled;
	}

	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
	{
		Error_Handler();
	}
	while(memtx==0);



	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&accel_wake_up_ths,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=accel_wake_up_ths&~0x3f;


hexcode|=USER_Harsh_Acceleration_Brake_Function_Wakeup_Threshold_Optimize();

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Set wake-up threshold < harsh_accel_brake_function_wakeup_threshold mg
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&accel_ctrl8_xl,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	if(who_am_i==0x69)
	{
		hexcode=accel_ctrl8_xl&~0x60;

		hexcode|=0x40; //HPF cutoff odr/9
	}
	if(who_am_i==0x6a)
	{
		hexcode=accel_ctrl8_xl&~0x68;

		hexcode|=0x40;//HPF cutoff odr/9;ODR/2 low pass filtered sent to composite filter
	}

	memtx=0;
	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL8_XL,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)
			{
		Error_Handler();
			}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&accel_wake_up_dur,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=accel_wake_up_dur&~0x60;
	hexcode|=0x60;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Maximum (ODR*3) duration for wake up
	{
		Error_Handler();
	}


	while(memtx==0);

	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&accel_md1_cfg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);

	hexcode=accel_md1_cfg&~0x20;
	hexcode|=0x20;

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&hexcode,1)!=HAL_OK)// Wake-up interrupt driven to INT1 pin
	{
		Error_Handler();
	}


	while(memtx==0);


	sprintf(out_string,"\r\nDevice Acceleration Sensing Function Started......\r\n");
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	acceleration_wakeup_init_function_status=EXICUTED;

	wakeup_mode=ACCELERATION;
	device_mode=ACCELERATION;


}
