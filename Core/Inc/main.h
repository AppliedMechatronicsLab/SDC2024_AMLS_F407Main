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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOC
#define USL4_Pin GPIO_PIN_15
#define USL4_GPIO_Port GPIOE
#define MOSI_Pin GPIO_PIN_12
#define MOSI_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_13
#define MISO_GPIO_Port GPIOB
#define SS_Pin GPIO_PIN_14
#define SS_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_15
#define SCK_GPIO_Port GPIOB
#define USL3_Pin GPIO_PIN_15
#define USL3_GPIO_Port GPIOD
#define M2_Pin GPIO_PIN_6
#define M2_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_7
#define M1_GPIO_Port GPIOC
#define M3_H_Pin GPIO_PIN_8
#define M3_H_GPIO_Port GPIOC
#define IN_2_Pin GPIO_PIN_8
#define IN_2_GPIO_Port GPIOA
#define IN_4_Pin GPIO_PIN_11
#define IN_4_GPIO_Port GPIOA
#define USL2_Pin GPIO_PIN_12
#define USL2_GPIO_Port GPIOC
#define DIR_M2_Pin GPIO_PIN_0
#define DIR_M2_GPIO_Port GPIOD
#define DIR_M1_Pin GPIO_PIN_1
#define DIR_M1_GPIO_Port GPIOD
#define IN_3_Pin GPIO_PIN_4
#define IN_3_GPIO_Port GPIOD
#define M3_EN_Pin GPIO_PIN_6
#define M3_EN_GPIO_Port GPIOD
#define IN_1_Pin GPIO_PIN_7
#define IN_1_GPIO_Port GPIOD
#define Step_In_2_Pin GPIO_PIN_4
#define Step_In_2_GPIO_Port GPIOB
#define Step_In_1_Pin GPIO_PIN_5
#define Step_In_1_GPIO_Port GPIOB
#define BUTTON_8_Pin GPIO_PIN_8
#define BUTTON_8_GPIO_Port GPIOB
#define BUTTON_9_Pin GPIO_PIN_9
#define BUTTON_9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
