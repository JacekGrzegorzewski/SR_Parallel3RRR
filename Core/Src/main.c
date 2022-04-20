/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "./motor_const.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RxBuf_SIZE 64
#define MainBuf_SIZE 128

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

int i = 0;


motorInfo test;



volatile int flag_htim2_done=0;
volatile int flag_command_recieved = 0;
uint16_t size_recieved=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

int _write( int file,unsigned char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
	return len;
}





//jednostki sie nie zgadzaja w rownaniu roznicowym
//acc_lim ma jednostke czasu, a powinien jednoski wgle nie miec
//jak naprawic bog jeden wie

//dobra walic, i tak liczy dynamicznie to jak dojdzie do maksa to policzy decel
void reset_motor(motorInfo *motor, long total_steps, unsigned accel, unsigned decel, unsigned max )
{

	HAL_TIM_OC_Stop(&htim2,TIM_CHANNEL_1);

	max = max <= MIN_VEL? MIN_VEL:max;
	motor->max_speed = (max >= MAX_VEL) ? MAX_VEL : max;
	motor->dir = total_steps>0 ? 1 : -1;
	motor->total_steps=total_steps>0?total_steps:-total_steps;
	motor->auto_reload=52500;// length of current pulse in timer ticks
	motor->rest=0;
motor->state=ACCEL;

	if(accel>1000)//przyspieszenie ogranczone do 1rad/s^2
		accel=1000;
	if(accel == 0)
		accel=1;

	if(decel>5000)//przyspieszenie ogranczone do 1rad/s^2
			decel=5000;
		if(decel == 0)
			decel=1;

	motor->step_position=0;



	motor->peak_velocity=(motor->total_steps*decel)/(accel+decel);

	motor->acceleration=accel;

	motor->deceleration=decel;

	motor->max_speed_ARR = CLK_FRQ*ALPHA*1000/motor->max_speed;

	motor->steps=0;




}




//problem wynika z duego bledu dzielenia staloprzecinkowego
//trzeba dodac poprawke i ewentualna blokade predkosci
/*
unsigned long calculate_auto_reload(motorInfo *motor)
{

	unsigned tmp=0;
	if(motor->steps < motor->total_steps)
	{
		motor->step_position+=motor->dir;
	}
	else
	{
		HAL_TIM_OC_Stop(&htim2,TIM_CHANNEL_1);
		flag_htim2_done=1;

		return 52500;
	}

	if(motor->steps <= motor->peak_velocity)
	{
		//motor->auto_reload = CLK_FRQ/(MIN_FREQ +((motor->steps)*motor->acceleration)/(ALPHA*100));

		tmp=motor->rest;
		motor->rest=(motor->auto_reload+tmp)%(motor->steps+(unsigned long)(MIN_VEL/motor->acceleration));
		motor->auto_reload -= (motor->auto_reload + tmp)/(motor->steps+(unsigned long)(MIN_VEL/motor->acceleration));


	}
	if((motor->steps > motor->acc_lim) && (motor->steps < motor->decel_start))
	{
		motor->auto_reload = motor->max_speed_ARR;
		motor->rest=0;
	}
	if(motor->steps >=motor->decel_start)
	{
		tmp=motor->rest;
		motor->rest = (motor->auto_reload+tmp)%(motor->total_steps - motor->steps + (unsigned long)(MIN_VEL/motor->deceleration));
		motor->auto_reload += (motor->auto_reload+tmp)/(motor->total_steps - motor->steps + (unsigned long)(MIN_VEL/motor->deceleration));

	}

	if(motor->auto_reload>52500)
		motor->auto_reload=52500;

	else if(motor->auto_reload < 3000)
		motor->auto_reload=3000;


	motor->steps++;
	return (motor->auto_reload);


}
*/

unsigned long calculate_auto_reload(motorInfo *motor)
{

	unsigned tmp=0;
	if(motor->steps < motor->total_steps)
	{
		motor->step_position+=motor->dir;
	}
	else
		motor->state=STOP;

	switch (motor->state)
	{
	case STOP:
		HAL_TIM_OC_Stop(&htim2,TIM_CHANNEL_1);
		flag_htim2_done=1;
		return 52500;
		break;
	case ACCEL:
		tmp=motor->rest;
		motor->rest =(motor->auto_reload + tmp)%(motor->steps + (unsigned long)(MIN_VEL/motor->acceleration));
		motor->auto_reload -= (motor->auto_reload + tmp)/(motor->steps + (unsigned long)(MIN_VEL/motor->acceleration));

		if(motor->auto_reload >= motor->max_speed_ARR)
		{
			motor->state=RUN;
			motor->rest = 0;
			motor->auto_reload=motor->max_speed_ARR;
			motor->acc_lim=motor->steps;
			motor->decel_start=motor->total_steps - motor->acc_lim*motor->acceleration/motor->deceleration;
		}
		if(motor->steps >= motor->peak_velocity)
			motor->state=DECEL;
		break;
	case RUN:
		if(motor->steps >= motor->decel_start);
			motor->state=DECEL;
		break;
	case DECEL:
		tmp = motor->rest;
		motor->rest = (motor->auto_reload+tmp)%(motor->total_steps - motor->steps + (unsigned long)(MIN_VEL/motor->deceleration));
		motor->auto_reload += (motor->auto_reload+tmp)/(motor->total_steps - motor->steps + (unsigned long)(MIN_VEL/motor->deceleration));

		break;

	}
	if(motor->auto_reload>52500)
		motor->auto_reload=52500;

	else if(motor->auto_reload < 3000)
		motor->auto_reload=3000;


	motor->steps++;
	return (motor->auto_reload);


}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	uint8_t Rx_data[256];


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//stepp_array = (uint16_t *)calloc(1,sizeof(uint16_t));
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */





	reset_motor(&test,6400,10,1,27000);
	//zmiana skali, 100 = 1, predkosc bez zmian


	  TIM2->ARR=52500;
	  HAL_TIM_Base_Start_IT(&htim2);
	  HAL_TIM_Base_Start_IT(&htim3);
	  HAL_TIM_Base_Start_IT(&htim4);

	  HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);


flag_htim2_done=flag_command_recieved=0;

  unsigned timer_val,accel,max_speed,steps,decel;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flag_htim2_done)
	  {
		  timer_val = __HAL_TIM_GET_COUNTER(&htim3)-timer_val;
		  printf("Done, Steps: %d Time: %u (10000 - 1s)\r\n",test.steps,timer_val);
		  flag_htim2_done=0;
	  }

	  if(flag_command_recieved)
	  {
		  memcpy(MainBuf,RxBuf,size_recieved);
		  		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
		  		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);

		  		printf("Recieved: %s \r\n",MainBuf);
		  		sscanf(MainBuf,"%d %u %u %u",&steps, &accel, &decel, &max_speed);
		  		//accel, decel in 0.01rad/s^2
		  		//max_speed in 0.01rad/s

		  		reset_motor(&test,steps,accel,decel,max_speed);



		  		  TIM2->ARR=52500;



		  		timer_val = __HAL_TIM_GET_COUNTER(&htim3);
		  		flag_command_recieved=0;
		  		HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 84-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEPPER_DIR_3_Pin|STEPPER_DIR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STEPPER_DIR_1_Pin|MS3_Pin|MS2_Pin|MS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEPPER_DIR_3_Pin STEPPER_DIR_2_Pin */
  GPIO_InitStruct.Pin = STEPPER_DIR_3_Pin|STEPPER_DIR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STEPPER_DIR_1_Pin MS3_Pin MS2_Pin MS1_Pin */
  GPIO_InitStruct.Pin = STEPPER_DIR_1_Pin|MS3_Pin|MS2_Pin|MS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : POSITIONED_1_Pin POSITIONED_2_Pin POSITIONED_3_Pin */
  GPIO_InitStruct.Pin = POSITIONED_1_Pin|POSITIONED_2_Pin|POSITIONED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if(huart->Instance == USART2)
	{
		flag_command_recieved=1;
		size_recieved=Size;
	}

}

/*
unsigned int calculate_perod(motorInfo *motor)
{

	if(motor->steps < motor->total_steps)
			{
				motor->steps++;
				motor->step_position++;
			}
	else
	{}

				if(motor->accel_steps == 0)
				{
					motor->n++;
					tmp = motor->rest;
					motor->d -=2*motor->d/(4*motor->n+1);

					if(motor->d <= motor->min_interval)
					{
						motor->d=motor->min_interval;
						motor->accel_steps = motor->steps;
					}
					if(motor->steps >= motor->total_steps/2)
						motor->accel_steps = motor->steps;

				}
				else if (motor->steps >= motor->total_steps - motor->accel_steps)
				{
					motor->n--;
					motor->d *= (4*motor->n+1)/(4*motor->n+1-2);
					tmp = motor->rest;


				}

				return motor->d;
			}
}

*/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2 )
	{

		i++;
		if(i==63999)
			{i++;
			i--;}

		if(test.steps<=test.total_steps)
			TIM2->ARR=calculate_auto_reload(&test);


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
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

