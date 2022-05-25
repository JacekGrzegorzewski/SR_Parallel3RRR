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
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */



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
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

int _write( int file,unsigned char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
	return len;
}





float correction(unsigned long steps, unsigned long accel)
{

	if(accel ==0)
		return 0;
	if(steps ==0)
		return 0;
	return pow(50,0.5)*((-0.1956)*pow(steps,-0.4198)+(0.1559*0.9885))/sqrt(accel);
}


float estimate_ttc(motorInfo *motor)
{
	float tmp;
	if(motor->accel_stop <= motor->peak_velocity)
		{
		motor->acc_time= motor->max_speed/(double)motor->acceleration;
			tmp = (motor->max_speed)*((double)1/motor->acceleration +(double)1/motor->deceleration)-correction(motor->accel_stop,motor->acceleration) - correction(motor->total_steps-motor->decel_start,motor->deceleration);
			tmp += (motor->decel_start-motor->accel_stop)*(double)ALPHA*1000/motor->max_speed;
		}
	else
		{
			/*tmp = (double)sqrt(2000*ALPHA*motor->total_steps);
			tmp *=1.0/sqrt(motor->acceleration) + 1.0/sqrt(motor->deceleration);
			*/
			motor->acc_time=sqrt(2000*ALPHA*motor->peak_velocity/motor->acceleration);
			tmp=sqrt(2000*ALPHA*motor->peak_velocity/motor->acceleration)-correction(motor->peak_velocity,motor->acceleration);

			tmp+=sqrt(2000*ALPHA*(motor->total_steps-motor->peak_velocity)/motor->deceleration)-correction(motor->total_steps-motor->peak_velocity,motor->deceleration);
			motor->dec_time=sqrt(2000*ALPHA*(motor->total_steps-motor->peak_velocity)/motor->deceleration);



		}


	//motor->time_before_correction =tmp;
	return tmp;
}


unsigned long acc_phase_time(unsigned long steps, unsigned long time) //
{
	unsigned long time = pow((sqrt(2*ALPHA*steps)-pow(20.0,-0.5)*((-0.1956)*pow(steps,-0.4198)+(0.1559*0.9885)))/(double)time,2);

}

unsigned long desired_acceleration(unsigned long steps, unsigned long time)
{
	//jeżeli czas jest za krótki by wykonać ruch z maksymalną prędkością, zostanie wydłużony do tego czasu, i odwrotnie jezeli czas jest za dlugi


		if(ALPHA*steps/MAX_VEL > time)
			time = 0.95*(ALPHA*steps/MAX_VEL);//mnożymy przez 0.95 by zapewnić płynniejsze przyspieszanie niż gwałtowny skok z 0 do max

		if(ALPHA*steps/MIN_VEL < time)
			time = 1.05*(ALPHA*steps/MIN_VEL);//jezeli czas jest za długi by ruch w nim wykonac z prędkością minimalną


		//przypadek bez ruchu jednostajnego



}

void init_movement(motorInfo *motor, long total_steps, unsigned long accel, unsigned long decel, unsigned max)
{


		max = max <= MIN_VEL? MIN_VEL:max;
		motor->max_speed = (max >= MAX_VEL) ? MAX_VEL : max;
		motor->dir = total_steps>0 ? 1 : -1;

		HAL_GPIO_WritePin(motor->GPIOX, motor->GPIO_Label, motor->dir > 0);


		motor->total_steps=total_steps>0?total_steps:-total_steps;
		motor->rest=0;
		motor->state=ACCEL;

		motor->auto_reload=0.676*CLK_FRQ*sqrt(2000*ALPHA/accel)/(motor->timer->Instance->PSC+1);// length of current pulse in timer ticks
		motor->max_speed_ARR = CLK_FRQ*ALPHA*1000/motor->max_speed/(motor->timer->Instance->PSC+1);
		motor->accel_stop = motor->max_speed*motor->max_speed/(2000*ALPHA*accel);

		if(!motor->accel_stop)
			motor->accel_stop =1 ;


		motor->peak_velocity=(motor->total_steps*(unsigned long long)decel)/(accel+decel);

		if(!motor->peak_velocity)
			motor->peak_velocity =1;

		if(motor->accel_stop <= motor->peak_velocity)
			motor->decel_start= (motor->total_steps-(motor->accel_stop*accel)/decel);
		else
			motor->decel_start = motor->peak_velocity;


		motor->movement_done=0;
		motor->acceleration=accel;
		motor->deceleration=decel;
		motor->steps=0;

		motor->time_to_complete=estimate_ttc(motor);

		motor->T1=motor->T2=motor->T3=0;

		motor->timer->Instance->ARR=motor->auto_reload;
		//generujemy update resetując rejestry
		//bez tego nie działa, 4 godziny życia za mną :(
		motor->timer->Instance->EGR |= TIM_EGR_UG;


  		motor->T1 = __HAL_TIM_GET_COUNTER(&htim5);
		HAL_TIM_OC_Start(motor->timer,TIM_CHANNEL_1);
	    HAL_TIM_Base_Start_IT(motor->timer);




}

void init_movement_time(motorInfo *motor,long total_steps, unsigned long time)
{




	unsigned long accel = total_steps*ALPHA/pow(time/1000.0,2);
	if(accel*time/(2.0*pow(1000,2))<MAX_VEL)//jeżeli nie osiągamy maksymalnej prędkości nie ma ruchu jednostajnego
		init_movement(motor,total_steps,accel,accel,MAX_VEL);
	else
		;
	//przypadek z ruchem jednostajnym





}


void reset_motor(motorInfo *motor,TIM_HandleTypeDef *timer,GPIO_TypeDef *GPIOX,uint16_t GPIO_Label)
{
	motor->timer = timer;
	motor->GPIOX = GPIOX;
	motor->GPIO_Label=GPIO_Label;
	test.movement_done=1;
	motor->step_position=0;

	HAL_GPIO_WritePin(motor->GPIOX, motor->GPIO_Label, motor->dir>0);
	HAL_TIM_OC_Stop(motor->timer,TIM_CHANNEL_1);


}





unsigned long calculate_auto_reload(motorInfo *motor)
{

	if(motor->steps < motor->total_steps)
	{
		motor->step_position+=motor->dir;
	}
	else
		motor->state=STOP;

	motor->steps++;
	unsigned long  tmp=0;

	switch (motor->state)
	{
	case STOP:

		motor->movement_done=1;
		flag_htim2_done=1;
		HAL_TIM_OC_Stop(motor->timer,TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(motor->timer);


			motor->T3 = __HAL_TIM_GET_COUNTER(&htim5) - motor->T3;

		return motor->auto_reload;
		break;
	case ACCEL:

		tmp=motor->rest;
		motor->rest =(2*motor->auto_reload + tmp)%(4*motor->steps + 1);
		motor->auto_reload -= (2*motor->auto_reload + tmp)/(4*motor->steps + 1);

		if(motor->steps>=motor->decel_start)
			{
	  		 motor->T1 = __HAL_TIM_GET_COUNTER(&htim5)-motor->T1;

	  	//	 htim5.Instance->CNT=0;
	  		 motor->T3 =  __HAL_TIM_GET_COUNTER(&htim5);

				motor->state=DECEL;
			}
		else if(motor->steps >= motor->accel_stop)
		{
			motor->T1 = __HAL_TIM_GET_COUNTER(&htim5)-motor->T1;

		//	 htim5.Instance->CNT=0;
			motor->T2 =  __HAL_TIM_GET_COUNTER(&htim5);

			motor->state = RUN;
			motor->auto_reload = motor->max_speed_ARR;
		}
		break;
	case RUN:
		if(motor->steps >= motor->decel_start)
		{
			motor->T2 =  __HAL_TIM_GET_COUNTER(&htim5)-motor->T2;
		//	htim5.Instance->CNT=0;
			motor->T3 =  __HAL_TIM_GET_COUNTER(&htim5);
			motor->state=DECEL;
				//motor->rest=0;
		}
		break;
	case DECEL:
		tmp=motor->rest;
		motor->rest =(long)((2*motor->auto_reload + motor->rest))%(4*((long)(motor->steps-motor->total_steps)) + 1);
		motor->auto_reload -= (long)((2*motor->auto_reload + tmp))/(4*((long)(motor->steps-motor->total_steps)) + 1);
		break;

	}



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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */





	  reset_motor(&test,&htim2,STEPPER_DIR_1_GPIO_Port, STEPPER_DIR_1_Pin);


	  HAL_TIM_Base_Start_IT(&htim3);
	  HAL_TIM_Base_Start_IT(&htim4);
	  HAL_TIM_Base_Start_IT(&htim5);




  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);


flag_htim2_done=flag_command_recieved=0;

float tmp;
  unsigned timer_val,accel,max_speed,steps,decel;
  unsigned steps_1,steps_2,time;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flag_htim2_done)
	  {
		  timer_val = __HAL_TIM_GET_COUNTER(&htim5)-timer_val;

		  printf("Done in %f, expected: %f corrected: %f MotorPos: %d\r\n",timer_val/1000000.0,test.acc_time+test.dec_time,test.time_to_complete, test.step_position);
		  printf("Time breakdown --- T1: %f   T2: %f   T3: %f\r\n", (double)test.T1/1000000, (double)test.T2/1000000, (double)test.T3/1000000);
		  tmp = ((double)test.T1/1000000+(double)test.T2/1000000+(double)test.T3/1000000);
		  printf("T_approx - T_exact : %fs , Error as percentege of actual time %f \n",test.time_to_complete-tmp,(test.time_to_complete*100.0/tmp-100.0));
		  tmp = ((double)test.T1/1000000);
		  printf("acc time estimate: %fs ,Delta %fs, error as percentage: %f",test.acc_time,(test.acc_time-tmp), test.acc_time*100.0/tmp-100.0);
		  printf("\r\n\r\n---------\r\n\r\n");
		  flag_htim2_done=0;

	  }

	  if(flag_command_recieved)
	  {
			memcpy(MainBuf, RxBuf, size_recieved);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

			printf("Recieved: %s \r\n", MainBuf);

			sscanf(MainBuf, "%d %u %u %u", &steps, &accel, &decel, &max_speed);
			//accel, decel in 1m rad/s^2
			//max_speed in 1m rad/s
			init_movement(&test, steps, accel, decel, max_speed);

			//htim5.Instance->CNT	=0;
			timer_val = __HAL_TIM_GET_COUNTER(&htim5);
			flag_command_recieved = 0;


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
  htim2.Init.Prescaler = 400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400-1;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  HAL_GPIO_WritePin(STEPPER_DIR_1_GPIO_Port, STEPPER_DIR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEPPER_DIR_3_Pin STEPPER_DIR_2_Pin */
  GPIO_InitStruct.Pin = STEPPER_DIR_3_Pin|STEPPER_DIR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STEPPER_DIR_1_Pin */
  GPIO_InitStruct.Pin = STEPPER_DIR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEPPER_DIR_1_GPIO_Port, &GPIO_InitStruct);

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



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == test.timer)
	{
		if(!test.movement_done)
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

