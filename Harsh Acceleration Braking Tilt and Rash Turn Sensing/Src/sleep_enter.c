/*
 * sleep_enter.c
 *
 *  Created on: 07-Sep-2020
 *      Author: Bibin
 */
#include "status.h"
#include "stm32f0xx_hal_def.h"
#include "usart.h"
#include "lsm6ds3_l.h"
#include "i2c.h"


	extern enum Referance_Tilt_function current_referance_tilt_function;
	extern  float total_error_xl_mg[3];
	extern  int8_t total_error_int_xl[3];
	extern volatile uint8_t uarttx_req,memrx;
	extern enum Wakeup_Device_Mode device_mode;
	extern enum AXES forward_axes;
	extern enum  Sign sign_of_acceleration;
	extern enum Function_Exicution forward_axes_wakeup_init_function_status,acceleration_wakeup_init_function_status;

void USER_Sleep_Enter(void)
{

	uint8_t wake_up_src;

	memset(total_error_int_xl,0,sizeof(total_error_int_xl));
	memset(total_error_xl_mg,0,sizeof(total_error_xl_mg));



	sign_of_acceleration=SIGN_NOT_SET;
	device_mode=SLEEP;
	forward_axes=AXES_NOT_SET;
	forward_axes_wakeup_init_function_status=NOT_EXICUTED;
	acceleration_wakeup_init_function_status=NOT_EXICUTED;

	if(current_referance_tilt_function==GRAVITY_AXES_PLUS_FORWARD_AXES)
	{
		current_referance_tilt_function=ONLY_GRAVITY_AXES;
	}


	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);


	USER_Wakeup_Init();

	HAL_GPIO_WritePin(ST_LED_GPIO_Port,ST_LED_Pin,GPIO_PIN_RESET);



}
