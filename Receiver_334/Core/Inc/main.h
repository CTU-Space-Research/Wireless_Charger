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
#include "stm32f3xx_hal.h"

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
#define BTN_Pin GPIO_PIN_13
#define BTN_GPIO_Port GPIOC
#define ADC1_IN_U_Pin GPIO_PIN_1
#define ADC1_IN_U_GPIO_Port GPIOA
#define ADC1_IN_temp_Pin GPIO_PIN_2
#define ADC1_IN_temp_GPIO_Port GPIOA
#define ADC2_IN_I_Pin GPIO_PIN_4
#define ADC2_IN_I_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOB
#define OUTPUT_EN_Pin GPIO_PIN_13
#define OUTPUT_EN_GPIO_Port GPIOB
#define PING_IMP_Pin GPIO_PIN_14
#define PING_IMP_GPIO_Port GPIOB
#define ASK_IN_Pin GPIO_PIN_8
#define ASK_IN_GPIO_Port GPIOA
#define FSK_DEMOD_IN_Pin GPIO_PIN_6
#define FSK_DEMOD_IN_GPIO_Port GPIOB
#define RS485_SWITCH_Pin GPIO_PIN_9
#define RS485_SWITCH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
