/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include "delay.h"
#include "arm_math.h"
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
#define AD9833_SCLK_Pin GPIO_PIN_5
#define AD9833_SCLK_GPIO_Port GPIOA
#define AD9833_CS1_Pin GPIO_PIN_6
#define AD9833_CS1_GPIO_Port GPIOA
#define AD9833_MOSI_Pin GPIO_PIN_7
#define AD9833_MOSI_GPIO_Port GPIOA
#define AD9833_CS2_Pin GPIO_PIN_4
#define AD9833_CS2_GPIO_Port GPIOC
#define LCDCS_Pin GPIO_PIN_8
#define LCDCS_GPIO_Port GPIOE
#define LCDRST_Pin GPIO_PIN_9
#define LCDRST_GPIO_Port GPIOE
#define LCDDC_Pin GPIO_PIN_10
#define LCDDC_GPIO_Port GPIOE
#define LCDLED_Pin GPIO_PIN_11
#define LCDLED_GPIO_Port GPIOE
#define LEDR_Pin GPIO_PIN_8
#define LEDR_GPIO_Port GPIOD
#define LEDG_Pin GPIO_PIN_9
#define LEDG_GPIO_Port GPIOD
#define LEDB_Pin GPIO_PIN_10
#define LEDB_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
