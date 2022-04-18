/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEPPER_PULSE_1_Pin GPIO_PIN_5
#define STEPPER_PULSE_1_GPIO_Port GPIOA
#define STEPPER_PULSE_2_Pin GPIO_PIN_6
#define STEPPER_PULSE_2_GPIO_Port GPIOA
#define STEPPER_DIR_3_Pin GPIO_PIN_14
#define STEPPER_DIR_3_GPIO_Port GPIOB
#define STEPPER_DIR_2_Pin GPIO_PIN_15
#define STEPPER_DIR_2_GPIO_Port GPIOB
#define STEPPER_DIR_1_Pin GPIO_PIN_6
#define STEPPER_DIR_1_GPIO_Port GPIOC
#define MS3_Pin GPIO_PIN_7
#define MS3_GPIO_Port GPIOC
#define MS2_Pin GPIO_PIN_8
#define MS2_GPIO_Port GPIOC
#define MS1_Pin GPIO_PIN_9
#define MS1_GPIO_Port GPIOC
#define POSITIONED_1_Pin GPIO_PIN_8
#define POSITIONED_1_GPIO_Port GPIOA
#define POSITIONED_2_Pin GPIO_PIN_9
#define POSITIONED_2_GPIO_Port GPIOA
#define POSITIONED_3_Pin GPIO_PIN_10
#define POSITIONED_3_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define STEPPER_PULSE_3_Pin GPIO_PIN_6
#define STEPPER_PULSE_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
