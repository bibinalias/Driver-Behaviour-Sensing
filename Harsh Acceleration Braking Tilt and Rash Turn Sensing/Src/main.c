/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "status.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



uint32_t last_tick_value=0;
volatile uint8_t memtx=0,memrx=0,uarttx_req=0,uartrx_req=1,instruction_length=0, i=0,output_enable=1,tilt_req=1,sleep_dur=5,sleep_dur_device=5;

float total_error_xl_mg[3]={0};
int8_t total_error_int_xl[3]={0};

enum  Sign sign_of_acceleration=SIGN_NOT_SET,sign_of_gravity=SIGN_NOT_SET,sign_of_side_axes=SIGN_NOT_SET;
enum AXES gravity_axes=AXES_NOT_SET,forward_axes=AXES_NOT_SET,side_axes=AXES_NOT_SET;
enum Function_Exicution forward_axes_wakeup_init_function_status=NOT_EXICUTED,acceleration_wakeup_init_function_status=NOT_EXICUTED;
enum Wakeup_Device_Mode device_mode,wakeup_mode;
enum Referance_Tilt_function current_referance_tilt_function=NO_TILT_FUNCTION;
enum Acceleration_Brake harsh_accel_brake_function_wakeup_mode=HARSH_ACCELERATION;
uint16_t harsh_accel_brake_function_wakeup_threshold_optimized;
uint8_t memrx_last=0,memtx_last=0,who_am_i=0,tilt_threshold=180,rtt=125;
volatile char instruction_final[25]={0},instruction_raw[25]={0},c;
char out_string[200]={0};
float mg_per_LSB,mdps_per_LSB,rtt_optimized;

double referance_tilt_angle[3]={0},referance_tilt_angle_Optimized[3]={0};
uint16_t odr_xl_value,hat=1500,hbt=1500,harsh_accel_brake_function_wakeup_threshold=1500;
int16_t xl_axes_ref[3]={0},g_axes_ref[3]={0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	enum Acceleration_Brake acceleration_brake_status;
	enum Rash_Turn rash_turn_status;
	enum Tilt tilt_status;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

	//USER_activity_inactivity_init();
	USER_IMU_Init();
	USER_Wakeup_Init();
	USER_Referance_Tilt_Calculation();
	USER_Forward_axes_and_Tilt_Accelerometer_Init();
	USER_Rash_Turn_Gyroscope_Init();

	if(HAL_TIM_Base_Start_IT(&htim6)!=HAL_OK)
	{
		Error_Handler();
	}

	USER_UART_DMA_start();

	rtt_optimized=rtt;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		USER_Instruction_Sense();


		if (output_enable)
		{

			output_enable = 0;



			if(device_mode==SLEEP)
			{
				device_mode=USER_Wakeup_Detection();
			}



			if(gravity_axes	!=	AXES_NOT_SET	&&	forward_axes_wakeup_init_function_status==NOT_EXICUTED	&&	device_mode==FORWARD_AXIS)
			{
				USER_forward_axes_sense_Wakeup_Init();
			}

			if(device_mode==FORWARD_AXIS)
			{
				tilt_status	=	USER_Tilt_Calculation();
				rash_turn_status=USER_Rash_Turn_Detection();
				USER_Forward_axes_Find();
			}


			if(forward_axes!= AXES_NOT_SET	&&	sign_of_acceleration!=SIGN_NOT_SET &&	acceleration_wakeup_init_function_status==NOT_EXICUTED	&&	device_mode==ACCELERATION)
			{
				USER_Referance_Tilt_Calculation();
				USER_Acceleration_Wakeup_Init();
			}


			if(device_mode==ACCELERATION)
			{
				tilt_status	=	USER_Tilt_Calculation();
				rash_turn_status=USER_Rash_Turn_Detection();
				acceleration_brake_status=USER_Harsh_Acceleration_Harsh_brake_Find();
			}

			USER_Instruction_CHECK();


		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	memtx=1;
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

	memrx=1;


}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	while(1)
	{
		HAL_GPIO_TogglePin(ST_LED_GPIO_Port,ST_LED_Pin);
		HAL_Delay(200);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

	uarttx_req=0;

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartrx_req=0;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM6)
	{
		//HAL_GPIO_TogglePin(ST_LED_GPIO_Port,ST_LED_Pin);
		memset(instruction_final,0,25);
		instruction_length=i>strlen(instruction_raw)?i:strlen(instruction_raw);
		strncpy(instruction_final,strupr(instruction_raw),instruction_length);
		tilt_req=1;
		memset(instruction_raw,0,25);
		i=0;
		output_enable=1;


	}


}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
		HAL_GPIO_TogglePin(ST_LED_GPIO_Port,ST_LED_Pin);
		HAL_Delay(200);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
