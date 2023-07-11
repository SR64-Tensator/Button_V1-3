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
#include "stm32l0xx_hal.h"

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

extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim22;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Dev_ID2_Pin GPIO_PIN_13
#define Dev_ID2_GPIO_Port GPIOC
#define Dev_ID3_Pin GPIO_PIN_14
#define Dev_ID3_GPIO_Port GPIOC
#define Dev_ID4_Pin GPIO_PIN_15
#define Dev_ID4_GPIO_Port GPIOC
#define Dev_ID5_Pin GPIO_PIN_0
#define Dev_ID5_GPIO_Port GPIOH
#define Dev_ID6_Pin GPIO_PIN_1
#define Dev_ID6_GPIO_Port GPIOH
#define Dev_ID7_Pin GPIO_PIN_0
#define Dev_ID7_GPIO_Port GPIOA
#define Key_IN4_Pin GPIO_PIN_1
#define Key_IN4_GPIO_Port GPIOA
#define Key_IN4_EXTI_IRQn EXTI0_1_IRQn
#define Key_IN2_Pin GPIO_PIN_2
#define Key_IN2_GPIO_Port GPIOA
#define Key_IN2_EXTI_IRQn EXTI2_3_IRQn
#define nRF_CSN_Pin GPIO_PIN_3
#define nRF_CSN_GPIO_Port GPIOA
#define nRF_IRQ_Pin GPIO_PIN_4
#define nRF_IRQ_GPIO_Port GPIOA
#define nRF_IRQ_EXTI_IRQn EXTI4_15_IRQn
#define nRF_SCK_Pin GPIO_PIN_5
#define nRF_SCK_GPIO_Port GPIOA
#define nRF_MISO_Pin GPIO_PIN_6
#define nRF_MISO_GPIO_Port GPIOA
#define nRF_MOSI_Pin GPIO_PIN_7
#define nRF_MOSI_GPIO_Port GPIOA
#define nRF_CE_Pin GPIO_PIN_2
#define nRF_CE_GPIO_Port GPIOB
#define Key_IN1_Pin GPIO_PIN_12
#define Key_IN1_GPIO_Port GPIOB
#define Key_IN1_EXTI_IRQn EXTI4_15_IRQn
#define Key_IN3_Pin GPIO_PIN_13
#define Key_IN3_GPIO_Port GPIOB
#define Key_IN3_EXTI_IRQn EXTI4_15_IRQn
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_3
#define LED_RED_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_4
#define LED_BLUE_GPIO_Port GPIOB
#define Dev_ID0_Pin GPIO_PIN_8
#define Dev_ID0_GPIO_Port GPIOB
#define Dev_ID1_Pin GPIO_PIN_9
#define Dev_ID1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define Debug
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
