/*
 * imu_init.c
 *
 *  Created on: 26-Aug-2020
 *      Author: Bibin
 */


#include "imu_init.h"
#include "lsm6ds3_l.h"
#include "lsm6ds3_l.h"
#include "i2c.h"

void USER_IMU_Init(void)
{
	extern uint8_t who_am_i;
	extern volatile uint8_t memrx;


	while(HAL_I2C_IsDeviceReady(&hi2c2,LSM6DS3,100,1000)!=HAL_OK);//check is Device Ready

	memrx=0;

	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WHO_AM_I,I2C_MEMADD_SIZE_8BIT,&who_am_i,1)!=HAL_OK)//read device's WHO_AM_I Register
	{
		Error_Handler();
	}

	while(memrx==0);
}
