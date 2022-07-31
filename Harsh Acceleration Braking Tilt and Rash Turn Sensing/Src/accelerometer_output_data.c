#include "accelerometer_output_data.h"
#include "status.h"
#include "lsm6ds3_l.h"
#include "i2c.h"
#include "usart.h"
	extern volatile uint8_t memrx,uarttx_req;
	extern float mg_per_LSB;
	extern char out_string[200];

void USER_Accelerometer_Output_Data(void)
{

	int16_t array_out_xl[3];
	uint8_t status_reg;
	struct out_xl out_xl_current;
	struct out_xl_mg out_xl_current_mg;




wait:	memrx=0;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,STATUS_REG,I2C_MEMADD_SIZE_8BIT,&status_reg,1)!=HAL_OK)
	{
		Error_Handler();
	}

	while(memrx==0);



	if(status_reg&1 ==1)
	{

		memrx=0;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2,LSM6DS3,OUTX_L_XL,I2C_MEMADD_SIZE_8BIT,array_out_xl,6)!=HAL_OK)
		{
			Error_Handler();
		}

		while(memrx==0);
	}
	else
	{
		goto wait;
	}


	out_xl_current.x	=	array_out_xl[X];
	out_xl_current.y	=	array_out_xl[Y];
	out_xl_current.z	=	array_out_xl[Z];


	out_xl_current_mg.x	 =	out_xl_current.x * mg_per_LSB  ;
	out_xl_current_mg.y	 =	out_xl_current.y * mg_per_LSB  ;
	out_xl_current_mg.z	 =	out_xl_current.z * mg_per_LSB  ;





	sprintf(out_string," %d , %d , %d \r\n",(int)out_xl_current_mg.x,(int)out_xl_current_mg.y,(int)out_xl_current_mg.z);


	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_SET);
	uarttx_req=1;
	if(HAL_UART_Transmit_DMA(&huart3,out_string,strlen(out_string))!=HAL_OK)
	{
		Error_Handler();
	}
	while(uarttx_req==1);
	HAL_GPIO_WritePin(D_CON_RS485_GPIO_Port,D_CON_RS485_Pin,GPIO_PIN_RESET);

}

