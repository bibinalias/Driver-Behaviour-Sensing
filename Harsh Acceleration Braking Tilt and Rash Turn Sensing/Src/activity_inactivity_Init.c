#include "activity_inactivity_init.h"
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "stm32f0xx_hal_def.h"



void USER_activity_inactivity_init(void)
{
	extern volatile uint8_t who_am_i,memrx,memtx;
	extern float mg_per_LSB;
	extern uint16_t odr_xl_value;



	uint8_t hex38=0x38,hex60=0x60,hex00=0,hex20=0x20,hex01=0x01,hexe1=0Xe1,hex41=0x41,hex09=0x09;

	while(HAL_I2C_IsDeviceReady(&hi2c2,LSM6DS3,100,1000)!=HAL_OK);//check is Device Ready

	memrx=0;

	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WHO_AM_I,I2C_MEMADD_SIZE_8BIT,&who_am_i,1)!=HAL_OK)//read device's WHO_AM_I Register
	{
		Error_Handler();
	}

	while(memrx==0);


	if(who_am_i==0x69)
		{

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL9_XL,I2C_MEMADD_SIZE_8BIT,&hex38,1)!=HAL_OK)// Acc X, Y, Z axes enabled
	{
		Error_Handler();
	}


	while(memtx==0);

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&hex01,1)!=HAL_OK)// Apply slope filter; latch mode Enabled
	{
		Error_Handler();
	}


	while(memtx==0);

	memtx=0;

		if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&hex41,1)!=HAL_OK)// Set wake-up threshold
		{
			Error_Handler();
		}


		while(memtx==0);


		}


	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,CTRL1_XL,I2C_MEMADD_SIZE_8BIT,&hex60,1)!=HAL_OK)// Turn on the accelerometer
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




	if(who_am_i==0x6A)
	{
		memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,TAP_CFG,I2C_MEMADD_SIZE_8BIT,&hexe1,1)!=HAL_OK)// Interrupt Enabled ; Apply slope filter; latch mode Enabled; Inactivity configuration: acc to 12.5 LP, gyro to Power-Down
	{
		Error_Handler();
	}
	while(memtx==0);

	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_THS,I2C_MEMADD_SIZE_8BIT,&hex01,1)!=HAL_OK)// Set wake-up threshold
	{
		Error_Handler();
	}


	while(memtx==0);



	}




	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,WAKE_UP_DUR,I2C_MEMADD_SIZE_8BIT,&hex09,1)!=HAL_OK)// No duration for wake up;sleep duration is 6.15s
	{
		Error_Handler();
	}


	while(memtx==0);



	memtx=0;

	if(HAL_I2C_Mem_Write_DMA(&hi2c2,LSM6DS3,MD1_CFG,I2C_MEMADD_SIZE_8BIT,&hex20,1)!=HAL_OK)// Wake-up interrupt driven to INT1 pin
	{
		Error_Handler();
	}


	while(memtx==0);



}
