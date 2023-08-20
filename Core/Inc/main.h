/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define USL1_Pin GPIO_PIN_0
#define USL1_GPIO_Port GPIOC
#define USL4_Pin GPIO_PIN_15
#define USL4_GPIO_Port GPIOE
#define E_Button_Pin GPIO_PIN_10
#define E_Button_GPIO_Port GPIOB
#define E_Button_EXTI_IRQn EXTI15_10_IRQn
#define USL3_Pin GPIO_PIN_15
#define USL3_GPIO_Port GPIOD
#define M2_Pin GPIO_PIN_6
#define M2_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_7
#define M1_GPIO_Port GPIOC
#define M3_Pin GPIO_PIN_8
#define M3_GPIO_Port GPIOC
#define M4_Pin GPIO_PIN_9
#define M4_GPIO_Port GPIOC
#define USL2_Pin GPIO_PIN_12
#define USL2_GPIO_Port GPIOC
#define M2_D_Pin GPIO_PIN_0
#define M2_D_GPIO_Port GPIOD
#define M1_D_Pin GPIO_PIN_1
#define M1_D_GPIO_Port GPIOD
#define M3_D_Pin GPIO_PIN_5
#define M3_D_GPIO_Port GPIOD
#define M4_D_Pin GPIO_PIN_6
#define M4_D_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
