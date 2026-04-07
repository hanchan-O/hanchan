/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "motor_hw_config.h"
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN Private defines */

/*
 * MT6701 四路 TIM1 输入捕获引脚（STM32G031 数据手册 AF 表）：
 * - TIM1_CH3/CH4 仅能在 PA10/PA11（等）上，PA8/PA9 是 TIM1_CH1/CH2 的复用，不能与 PA0/PA1 同时接四路。
 * - 若 PCB 将前两路接在 PA8/PA9，请将 MT6701_TIM1_CH12_USE_PA8_PA9 置 1，且不要再使用 PA0/PA1 接 CH1/CH2。
 * - 后两路固定为 CH3=PA10、CH4=PA11（与“另两路用 PA8/9”的常见需求兼容：四路为 PA8,PA9,PA10,PA11）。
 */
#ifndef MT6701_TIM1_CH12_USE_PA8_PA9
#define MT6701_TIM1_CH12_USE_PA8_PA9 0
#endif

#if MT6701_TIM1_CH12_USE_PA8_PA9
#define MT6701_PIN_CH1 GPIO_PIN_8
#define MT6701_PIN_CH2 GPIO_PIN_9
#else
#define MT6701_PIN_CH1 GPIO_PIN_0
#define MT6701_PIN_CH2 GPIO_PIN_1
#endif
#define MT6701_PIN_CH3 GPIO_PIN_10
#define MT6701_PIN_CH4 GPIO_PIN_11

#if MOTOR_HW_USE_TIM1_IC_CH34
#define MT6701_GPIO_PIN_MASK (MT6701_PIN_CH1 | MT6701_PIN_CH2 | MT6701_PIN_CH3 | MT6701_PIN_CH4)
#else
#define MT6701_GPIO_PIN_MASK (MT6701_PIN_CH1 | MT6701_PIN_CH2)
#endif

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM14_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
