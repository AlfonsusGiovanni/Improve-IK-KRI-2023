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
#define LIMIT_F_Pin GPIO_PIN_2
#define LIMIT_F_GPIO_Port GPIOE
#define SERVO_6B_Pin GPIO_PIN_6
#define SERVO_6B_GPIO_Port GPIOE
#define SERVO_5A_Pin GPIO_PIN_0
#define SERVO_5A_GPIO_Port GPIOA
#define SERVO_5C_Pin GPIO_PIN_1
#define SERVO_5C_GPIO_Port GPIOA
#define SERVO_6C_Pin GPIO_PIN_2
#define SERVO_6C_GPIO_Port GPIOA
#define SERVO_5B_Pin GPIO_PIN_3
#define SERVO_5B_GPIO_Port GPIOA
#define SERVO_6A_Pin GPIO_PIN_6
#define SERVO_6A_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOA
#define SERVO_4B_Pin GPIO_PIN_0
#define SERVO_4B_GPIO_Port GPIOB
#define TOGGLE_Pin GPIO_PIN_8
#define TOGGLE_GPIO_Port GPIOE
#define SERVO_4C_Pin GPIO_PIN_13
#define SERVO_4C_GPIO_Port GPIOE
#define SEROK_Pin GPIO_PIN_14
#define SEROK_GPIO_Port GPIOB
#define CAPIT_SEROK_Pin GPIO_PIN_15
#define CAPIT_SEROK_GPIO_Port GPIOB
#define CAPIT_1_Pin GPIO_PIN_12
#define CAPIT_1_GPIO_Port GPIOD
#define CAPIT_2_Pin GPIO_PIN_13
#define CAPIT_2_GPIO_Port GPIOD
#define CAPIT_3_Pin GPIO_PIN_14
#define CAPIT_3_GPIO_Port GPIOD
#define TX_KOM_Pin GPIO_PIN_6
#define TX_KOM_GPIO_Port GPIOC
#define RX_KOM_Pin GPIO_PIN_7
#define RX_KOM_GPIO_Port GPIOC
#define SERVO_4A_Pin GPIO_PIN_8
#define SERVO_4A_GPIO_Port GPIOC
#define SERVO_3C_Pin GPIO_PIN_8
#define SERVO_3C_GPIO_Port GPIOA
#define SERVO_3B_Pin GPIO_PIN_9
#define SERVO_3B_GPIO_Port GPIOA
#define SERVO_3A_Pin GPIO_PIN_11
#define SERVO_3A_GPIO_Port GPIOA
#define SERVO_2C_Pin GPIO_PIN_15
#define SERVO_2C_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_3
#define M2_GPIO_Port GPIOD
#define M1_Pin GPIO_PIN_4
#define M1_GPIO_Port GPIOD
#define SERVO_2B_Pin GPIO_PIN_3
#define SERVO_2B_GPIO_Port GPIOB
#define SERVO_2A_Pin GPIO_PIN_4
#define SERVO_2A_GPIO_Port GPIOB
#define SERVO_1C_Pin GPIO_PIN_5
#define SERVO_1C_GPIO_Port GPIOB
#define GYRO_SCL_Pin GPIO_PIN_6
#define GYRO_SCL_GPIO_Port GPIOB
#define GYRO_SDA_Pin GPIO_PIN_7
#define GYRO_SDA_GPIO_Port GPIOB
#define SERVO_1B_Pin GPIO_PIN_8
#define SERVO_1B_GPIO_Port GPIOB
#define SERVO_1A_Pin GPIO_PIN_9
#define SERVO_1A_GPIO_Port GPIOB
#define LIMIT_B_Pin GPIO_PIN_0
#define LIMIT_B_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
