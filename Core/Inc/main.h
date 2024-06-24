/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define DEBOUNCE 14
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ROW5_Pin GPIO_PIN_0
#define ROW5_GPIO_Port GPIOC
#define COL4_Pin GPIO_PIN_1
#define COL4_GPIO_Port GPIOC
#define COL5_Pin GPIO_PIN_2
#define COL5_GPIO_Port GPIOC
#define COL6_Pin GPIO_PIN_3
#define COL6_GPIO_Port GPIOC
#define COL1_Pin GPIO_PIN_0
#define COL1_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_1
#define COL2_GPIO_Port GPIOA
#define COL3_Pin GPIO_PIN_2
#define COL3_GPIO_Port GPIOA
#define ROW1_Pin GPIO_PIN_3
#define ROW1_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_4
#define ROW2_GPIO_Port GPIOA
#define ROW4_Pin GPIO_PIN_5
#define ROW4_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_6
#define ROW3_GPIO_Port GPIOA
#define COL7_Pin GPIO_PIN_12
#define COL7_GPIO_Port GPIOB
#define ROW6_Pin GPIO_PIN_13
#define ROW6_GPIO_Port GPIOB
#define ROW7_Pin GPIO_PIN_14
#define ROW7_GPIO_Port GPIOB
#define ROW8_Pin GPIO_PIN_15
#define ROW8_GPIO_Port GPIOB
#define ROW9_Pin GPIO_PIN_6
#define ROW9_GPIO_Port GPIOC
#define COL9_Pin GPIO_PIN_7
#define COL9_GPIO_Port GPIOC
#define COL8_Pin GPIO_PIN_8
#define COL8_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
