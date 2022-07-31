/*
 * wakeup_detection.c
 *
 *  Created on: 08-Sep-2020
 *      Author: Bibin
 */
#include "wakeup_detection.h"
#include "i2c.h"
#include "lsm6ds3_l.h"
#include "status.h"
#include "usart.h"

extern char out_string[200];
extern volatile uint8_t  uarttx_req,memrx;

enum Wakeup_Device_Mode USER_Wakeup_Detection(void)
{
	uint8_t wake_up_src;



	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,WAKE_UP_SRC,I2C_MEMADD_SIZE_8BIT,&wake_up_src,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);
	if(  (wake_up_src&0x08) == 0x08)
		{
		HAL_GPIO_WritePin(ST_LED_GPIO_Port,ST_LED_Pin,GPIO_PIN_SET);

		sprintf(out_string,"\r\nDevice is Now in Forward Axes Mode\r\n");


		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
		uarttx_req=1;
		if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
		{
			Error_Handler();
		}
		while(uarttx_req==1);
		HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);


		return FORWARD_AXIS;

		}
	return SLEEP;

}

