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
#define FND_RCLK_Pin GPIO_PIN_2
#define FND_RCLK_GPIO_Port GPIOC
#define FND_DIO_Pin GPIO_PIN_3
#define FND_DIO_GPIO_Port GPIOC
#define HX711_DATA_Pin GPIO_PIN_2
#define HX711_DATA_GPIO_Port GPIOA
#define HX711_CLK_Pin GPIO_PIN_3
#define HX711_CLK_GPIO_Port GPIOA
#define FND_SCLK_Pin GPIO_PIN_10
#define FND_SCLK_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_7
#define IN4_GPIO_Port GPIOD
#define IN3_Pin GPIO_PIN_5
#define IN3_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_7
#define IN2_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_9
#define IN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
