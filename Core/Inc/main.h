/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BTN6_Pin GPIO_PIN_14
#define BTN6_GPIO_Port GPIOC
#define BTN6_EXTI_IRQn EXTI15_10_IRQn
#define BTN4_Pin GPIO_PIN_15
#define BTN4_GPIO_Port GPIOC
#define BTN4_EXTI_IRQn EXTI15_10_IRQn
#define SR502_2_Pin GPIO_PIN_2
#define SR502_2_GPIO_Port GPIOA
#define SR502_2_EXTI_IRQn EXTI2_IRQn
#define LSS_Pin GPIO_PIN_3
#define LSS_GPIO_Port GPIOA
#define LSS_EXTI_IRQn EXTI3_IRQn
#define SR501_Pin GPIO_PIN_0
#define SR501_GPIO_Port GPIOB
#define SR501_EXTI_IRQn EXTI0_IRQn
#define LSS2_Pin GPIO_PIN_1
#define LSS2_GPIO_Port GPIOB
#define LSS2_EXTI_IRQn EXTI1_IRQn
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define LIGHT_Pin GPIO_PIN_10
#define LIGHT_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define BTN5_Pin GPIO_PIN_12
#define BTN5_GPIO_Port GPIOA
#define BTN5_EXTI_IRQn EXTI15_10_IRQn
#define LIGHT2_Pin GPIO_PIN_3
#define LIGHT2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOB
#define BTN1_EXTI_IRQn EXTI4_IRQn
#define BTN2_Pin GPIO_PIN_5
#define BTN2_GPIO_Port GPIOB
#define BTN2_EXTI_IRQn EXTI9_5_IRQn
#define BTN3_Pin GPIO_PIN_8
#define BTN3_GPIO_Port GPIOB
#define BTN3_EXTI_IRQn EXTI9_5_IRQn
#define DHT_Pin GPIO_PIN_9
#define DHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
