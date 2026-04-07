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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "struct_typedef.h"
#include "pid.h"
//#include "encoder.h"
//#include "CRSF.h"
#include "math.h"
#include "motor.h"
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
#define PWM_M1_1_Pin GPIO_PIN_15

#define PWM_M1_1_GPIO_Port GPIOA

#define PWM_M1_2_Pin GPIO_PIN_3

#define PWM_M1_2_GPIO_Port GPIOB

#define PWM_M3_1_Pin GPIO_PIN_4

#define PWM_M3_1_GPIO_Port GPIOB

#define PWM_M3_2_Pin GPIO_PIN_5

#define PWM_M3_2_GPIO_Port GPIOB

/* M2 用 TIM2_CH3/CH4；原 PA2/PA3 与 USART2 冲突，故 CH3 改 PC6(AF2)，CH4 仍用 PA3 */
#define PWM_M2_1_Pin GPIO_PIN_6

#define PWM_M2_1_GPIO_Port GPIOC

#define PWM_M2_2_Pin GPIO_PIN_3

#define PWM_M2_2_GPIO_Port GPIOA

#define PWM_M4_1_Pin GPIO_PIN_0

#define PWM_M4_1_GPIO_Port GPIOB

#define PWM_M4_2_Pin GPIO_PIN_1

#define PWM_M4_2_GPIO_Port GPIOB


/* USER CODE BEGIN Private defines */
extern int motor_L_set;          /* 左电机设定（预留） */
extern int motor_R_set;          /* 右电机设定（预留） */
#define PI		3.14
float fastCos(float angle) ;
float fastSin(float angle) ;
float Angle_Convert_Radians(float Ang);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
