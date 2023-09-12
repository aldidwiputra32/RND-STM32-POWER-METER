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
#include "stm32f0xx_hal.h"

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
#define MODBUS_En_Pin GPIO_PIN_1
#define MODBUS_En_GPIO_Port GPIOA
#define CS1_DRIVER_Pin GPIO_PIN_4
#define CS1_DRIVER_GPIO_Port GPIOA
#define CS2_DRIVER_Pin GPIO_PIN_0
#define CS2_DRIVER_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_8
#define CS_GPIO_Port GPIOA
#define SPI_INT_Pin GPIO_PIN_9
#define SPI_INT_GPIO_Port GPIOA
#define BTN_Enter_Pin GPIO_PIN_15
#define BTN_Enter_GPIO_Port GPIOA
#define BTN_Enter_EXTI_IRQn EXTI4_15_IRQn
#define BTN_Set_Pin GPIO_PIN_3
#define BTN_Set_GPIO_Port GPIOB
#define BTN_Set_EXTI_IRQn EXTI2_3_IRQn
#define BTN_Up_Pin GPIO_PIN_4
#define BTN_Up_GPIO_Port GPIOB
#define BTN_Up_EXTI_IRQn EXTI4_15_IRQn
#define BACKLIGHT_En_Pin GPIO_PIN_5
#define BACKLIGHT_En_GPIO_Port GPIOB
#define BTN_Next_Pin GPIO_PIN_6
#define BTN_Next_GPIO_Port GPIOB
#define BTN_Next_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
