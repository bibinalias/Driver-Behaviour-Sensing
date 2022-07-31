/*
 * activity_sense.c
 *
 *  Created on: 19-Aug-2020
 *      Author: Bibin
 */
#include "i2c.h"
#include "usart.h"
#include "lsm6ds3_l.h"

uint8_t USER_Activity_sense(void)
{

	extern volatile uint8_t  uarttx_req,sleep_dur_device,memrx;
	extern uint32_t last_tick_value;
	uint8_t wake_up_src,activity_status;

	if(last_tick_value+(sleep_dur_device*1000)	<=	HAL_GetTick())
	{
		HAL_GPIO_WritePin(ST_LED_GPIO_Port,ST_LED_Pin,GPIO_PIN_RESET);
		activity_status=0;

	}


	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);



	if(  (wake_up_src&0x08) == 0x08)
	{

		last_tick_value=HAL_GetTick();
		activity_status=1;


		HAL_GPIO_WritePin(ST_LED_GPIO_Port,ST_LED_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		if((wake_up_src&0x01) == 0x01)
		{

			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nZ axis has triggered the wake-up event\r\n",strlen("\r\nZ axis has triggered the wake-up event\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);



		}
		if((wake_up_src&0x02) == 0x02)
		{
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nY axis has triggered the wake-up event\r\n",strlen("\r\nY axis has triggered the wake-up event\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);

		}
		if((wake_up_src&0x04) == 0x04)
		{
			uarttx_req=1;
			if(HAL_UART_Transmit_DMA(&huart3,"\r\nX axis has triggered the wake-up event\r\n",strlen("\r\nX axis has triggered the wake-up event\r\n"))!=HAL_OK)
			{
				Error_Handler();
			}
			while(uarttx_req==1);

		}



		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

	}
	return activity_status;
}
