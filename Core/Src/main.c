/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  
/**
************************************************************************************************
* @brief    独立驱动、姿态调节与参数控制
* @param    None
* @return   None
* @author   hanchan		2025.12.28
************************************************************************************************
**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "elrs.h"
#include "AS5600_PWM.h"
#include "motor.h"
#include "flight_control.h"
#include <stdint.h>
#include <stdbool.h>

// 调试模式开关 - 注释掉此行可禁用所有调试功能，节省内存
//#define DEBUG_MODE

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 左右电机的基准点（中位角度//调小翅膀向上）
// 使用const存储在Flash中，节省RAM
const int16_t motor_front_L_midpoint = 2034;
const int16_t motor_front_R_midpoint = 1844;
const int16_t motor_back_L_midpoint  = 1886;
const int16_t motor_back_R_midpoint  = 1996;

// 调试变量 - 用于观察电机行为（仅在DEBUG_MODE定义时启用）
#ifdef DEBUG_MODE
volatile int16_t debug_err[4] = {0};     // 各电机目标角度
volatile int16_t debug_current[4] = {0};    // 各电机当前角度
volatile int16_t debug_pwm[4] = {0};        // 各电机PWM输出
#endif

// 飞行控制相关结构体和函数已迁移到 flight_control.h / flight_control.c

// 电机控制辅助函数
void motor_disable(void)  // 翅膀失能
{
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
}

void motor_stop(void)  // 翅膀暂停
{
    TIM2->CCR1 = 19999;
    TIM2->CCR2 = 19999;
    TIM2->CCR3 = 19999;
    TIM2->CCR4 = 19999;
    TIM3->CCR1 = 19999;
    TIM3->CCR2 = 19999;
    TIM3->CCR3 = 19999;
    TIM3->CCR4 = 19999;
}

void motor_test(void)  // 调试前翅膀水平打开，运行后翅膀向下摆动，即为电机方向正确
{
    Set_Pwm(3000, 3000, 3000, 3000);
    HAL_Delay(100);
    Set_Pwm(0, 0, 0, 0);
    while (1);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// MIN宏定义 - 返回两个值中的较小值
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 {

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	HAL_Delay(1000);		//等待上电完成
	MX_USART1_UART_Init();	//开启接收机串口
	ELRS_Init();			//接收机初始化
	Chassis_PID_Init(); 	//电机PID初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // 步骤1：ADC采集滤波（每次循环都执行，确保数据最新）
    StarAndGetResult();

    if (elrs_data.Switch == 1)
    {

        if (elrs_data.Mode == 2)
        {
            // ==================== Mode2: 优化扑动飞行模式 ====================
            static uint8_t last_mode = 255;
            if (last_mode != 2) {
                last_mode = 2;
                heading_hold.hold_enabled = 0;  // 重置航向保持
            }

            // 步骤1: 姿态估计（每次循环都执行）
            Estimate_Attitude();

            // 步骤2: 设置PID参数（统一高响应）
            motor_1_pid.Kp = 32;
            motor_2_pid.Kp = 32;
            motor_3_pid.Kp = 32;
            motor_4_pid.Kp = 32;

            // 步骤3: 计算基准位置（含微调偏移）
            const int16_t motor_front_L_ready = motor_front_L_midpoint + elrs_data.midpoint + elrs_data.midpoint_1;
            const int16_t motor_front_R_ready = motor_front_R_midpoint + elrs_data.midpoint + elrs_data.midpoint_1;
            const int16_t motor_back_L_ready  = motor_back_L_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1;
            const int16_t motor_back_R_ready  = motor_back_R_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1;

            // 步骤4: 动态转向计算
            TurnControl_t turn_ctrl;
            Calculate_Dynamic_Turn(elrs_data.Yaw, &turn_ctrl);

            // 步骤5: 扑动频率控制
            thr = 12;  // 可改为 elrs_data.Throttle 实现变速
            uint32_t step_ms = Calculate_Flap_Step_Time(thr);

            uint32_t tick = HAL_GetTick();

            if ((int32_t)(tick - sm_next_tick) >= 0)
            {
                // 到时间：执行扑动步进
                Execute_Flap_Step(&turn_ctrl,
                                  motor_front_L_ready,
                                  motor_front_R_ready,
                                  motor_back_L_ready,
                                  motor_back_R_ready);

                // 步骤6: 电机同步补偿（减小4电机差异）
                Motor_Sync_Compensate();

                // 步骤7: 姿态稳定补偿（自动平衡）
                Attitude_Stabilize();

                // 步骤8: 航向保持（摇杆回中时自动直线飞行）
                if (abs16_fast(elrs_data.Yaw) < 50) {
                    // Yaw摇杆在中位附近，启用航向保持
                    Heading_Hold_Update(1);
                    // 应用航向保持修正（叠加到Yaw输入）
                    int16_t hold_comp = (heading_hold.heading_error * 2) / 10;
                    // 重新计算带保持补偿的转向
                    if (abs16_fast(hold_comp) > 5) {
                        Calculate_Dynamic_Turn(elrs_data.Yaw - hold_comp, &turn_ctrl);
                    }
                } else {
                    // Yaw摇杆偏离中位，禁用航向保持
                    Heading_Hold_Update(0);
                }

                // 安排下一次步进
                sm_next_tick += step_ms;

                // 记录调试数据
                #ifdef DEBUG_MODE
                debug_err[0] = Wings_Data.Wings_motor[0].Target_Angle - Wings_Data.Wings_motor[0].Corrective_Angle;
                debug_err[1] = Wings_Data.Wings_motor[1].Target_Angle - Wings_Data.Wings_motor[1].Corrective_Angle;
                debug_err[2] = Wings_Data.Wings_motor[2].Target_Angle - Wings_Data.Wings_motor[2].Corrective_Angle;
                debug_err[3] = Wings_Data.Wings_motor[3].Target_Angle - Wings_Data.Wings_motor[3].Corrective_Angle;
                #endif
            }

            Motor_PID_Control();
        }
        else if (elrs_data.Mode == 1)
		{
			static uint8_t last_mode = 255;
			if (last_mode != 1) last_mode = 1;
			
			motor_1_pid.Kp = 16;  // 电机0(右前) - 增加Kp以加快响应
			motor_2_pid.Kp = 16;  // 电机1(左后)
			motor_3_pid.Kp = 16;  // 电机2(左前)
			motor_4_pid.Kp = 20;  // 电机3(右后) - 增加Kp以加快响应
			// 基准角计算（带转向差动）
			// 注意：编码器值减小=翅膀上移（升力减小），编码器值增大=翅膀下移（升力增大）
			// 左转（Yaw<0）：右翼下移（升力增大），左翼上移（升力减小）
			// 右转（Yaw>0）：左翼下移（升力增大），右翼上移（升力减小）
			const int16_t front_targetL = motor_front_L_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 + 3*elrs_data.Yaw;
			const int16_t front_targetR = motor_front_R_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 - 3*elrs_data.Yaw;
			const int16_t back_targetL  = motor_back_L_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 + 3*elrs_data.Yaw;
			const int16_t back_targetR  = motor_back_R_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 - 3*elrs_data.Yaw;

			// 从当前值缓慢移动到目标值
			static uint32_t last_smooth_time = 0;
			const uint32_t SMOOTH_INTERVAL = 5; // 平滑间隔 5ms
			const int16_t MAX_STEP = 60;        // 最大步进值
			
			if ((int32_t)(now - last_smooth_time) >= SMOOTH_INTERVAL)
			{
				// 电机0（右前）→ 使用 front_targetR（右前目标）
				int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
				if (current0 < front_targetR) {
					current0 += MIN(MAX_STEP, front_targetR - current0);
				} else if (current0 > front_targetR) {
					current0 -= MIN(MAX_STEP, current0 - front_targetR);
				}
				Wings_Data.Wings_motor[0].Target_Angle = current0;
				
				// 电机2（左前）→ 使用 front_targetL（左前目标）
				int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
				if (current2 < front_targetL) {
					current2 += MIN(MAX_STEP, front_targetL - current2);
				} else if (current2 > front_targetL) {
					current2 -= MIN(MAX_STEP, current2 - front_targetL);
				}
				Wings_Data.Wings_motor[2].Target_Angle = current2;
				
				// 电机1（左后）→ 使用 back_targetL（左后目标）
				int16_t current1 = Wings_Data.Wings_motor[1].Corrective_Angle;
				if (current1 < back_targetL) {
					current1 += MIN(MAX_STEP, back_targetL - current1);
				} else if (current1 > back_targetL) {
					current1 -= MIN(MAX_STEP, current1 - back_targetL);
				}
				Wings_Data.Wings_motor[1].Target_Angle = current1;
				
				// 电机3（右后）→ 使用 back_targetR（右后目标）
				int16_t current3 = Wings_Data.Wings_motor[3].Corrective_Angle;
				if (current3 < back_targetR) {
					current3 += MIN(MAX_STEP, back_targetR - current3);
				} else if (current3 > back_targetR) {
					current3 -= MIN(MAX_STEP, current3 - back_targetR);
				}
				Wings_Data.Wings_motor[3].Target_Angle = current3;
				
				// 记录调试数据 - Mode1平翅模式误差（目标-当前）
				#ifdef DEBUG_MODE
				debug_err[0] = front_targetR - Wings_Data.Wings_motor[0].Corrective_Angle;  // 右前误差
				debug_err[1] = back_targetL - Wings_Data.Wings_motor[1].Corrective_Angle;   // 左后误差
				debug_err[2] = front_targetL - Wings_Data.Wings_motor[2].Corrective_Angle;  // 左前误差
				debug_err[3] = back_targetR - Wings_Data.Wings_motor[3].Corrective_Angle;   // 右后误差
				#endif
				
				last_smooth_time = now;
			}
		
			Motor_PID_Control();
			reset_flap_state();
	
		}
        else
		{
			// Mode0并翅模式 - 梯形速度规划改进版
			motor_1_pid.Kp = 16;   // 电机0(右前) - 增加Kp以加快响应
			motor_2_pid.Kp = 18;   // 电机1(左后)
			motor_3_pid.Kp = 16;   // 电机2(左前)
			motor_4_pid.Kp = 20;   // 电机3(右后) - 增加Kp以加快响应
			
			// 目标角度 - 统一为1024（竖直并翅）
			// 注意：数组索引与电机位置的对应关系
			// Wings_motor[0] = 右前, Wings_motor[1] = 左后
			// Wings_motor[2] = 左前, Wings_motor[3] = 右后
			const int16_t target_motor0 = 1024;  // 右前
			const int16_t target_motor1 = 1024;  // 左后
			const int16_t target_motor2 = 1024;  // 左前
			const int16_t target_motor3 = 1024;  // 右后
			
			// 梯形速度规划参数
			const uint32_t SMOOTH_INTERVAL = 5;   // 5ms更新周期
			const int16_t MAX_VELOCITY = 60;       // 最大速度（编码器值/周期）
			const int16_t ACCEL_LIMIT = 8;         // 加速度限制（速度变化量/周期）
			const int16_t HARD_DEADZONE = 10;       // 硬死区（直接停止）
			const int16_t SOFT_DEADZONE = 40;      // 软死区（开始减速）
			
			// 静态变量保持状态
			static uint32_t last_smooth_time_mode0 = 0;
			static int16_t current_vel[4] = {0, 0, 0, 0};  // 当前速度
			static uint8_t last_mode = 255;  // 上次模式，用于检测模式切换
			
			// 检测是否刚进入Mode0，如果是则清零速度
			if (last_mode != 0) {
				current_vel[0] = 0;
				current_vel[1] = 0;
				current_vel[2] = 0;
				current_vel[3] = 0;
				last_mode = 0;
			}

			if ((int32_t)(now - last_smooth_time_mode0) >= SMOOTH_INTERVAL)
			{
				// 计算四个电机的误差
				int16_t err[4];
				err[0] = target_motor0 - Wings_Data.Wings_motor[0].Corrective_Angle;  // 右前
				err[1] = target_motor1 - Wings_Data.Wings_motor[1].Corrective_Angle;  // 左后
				err[2] = target_motor2 - Wings_Data.Wings_motor[2].Corrective_Angle;  // 左前
				err[3] = target_motor3 - Wings_Data.Wings_motor[3].Corrective_Angle;  // 右后
				
				// 目标值数组
				int16_t targets[4] = {target_motor0, target_motor1, target_motor2, target_motor3};
				
				// 遍历四个电机，应用梯形速度规划
				for (uint8_t i = 0; i < 4; i++)
				{
					int16_t error = err[i];
					int16_t abs_err = abs16_fast(error);
					
					// 硬死区：误差很小，直接到达目标，速度归零
					if (abs_err <= HARD_DEADZONE)
					{
						Wings_Data.Wings_motor[i].Target_Angle = targets[i];
						current_vel[i] = 0;
						continue;
					}
					
					// 计算期望速度（带软死区减速）
					int16_t desired_vel;
					if (abs_err < SOFT_DEADZONE)
					{
						// 软死区内：二次曲线减速，平滑接近目标
						// 公式：vel = error * (abs_err / SOFT_DEADZONE)
						// 这样速度随误差减小而二次减小
						desired_vel = (int16_t)((int32_t)error * abs_err / SOFT_DEADZONE);
					}
					else
					{
						// 软死区外：全速前进，但限制最大速度
						desired_vel = error;
					}
					
					// 限制最大速度
					if (desired_vel > MAX_VELOCITY) desired_vel = MAX_VELOCITY;
					if (desired_vel < -MAX_VELOCITY) desired_vel = -MAX_VELOCITY;
					
					// 梯形速度规划：限制加速度（平滑启动/停止）
					int16_t vel_diff = desired_vel - current_vel[i];
					if (vel_diff > ACCEL_LIMIT) vel_diff = ACCEL_LIMIT;
					if (vel_diff < -ACCEL_LIMIT) vel_diff = -ACCEL_LIMIT;
					
					// 更新当前速度
					current_vel[i] += vel_diff;
					
					// 计算新的目标角度
					Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle + current_vel[i];
				}
				
				// 记录调试数据 - 使用err误差值更直观
				#ifdef DEBUG_MODE
				debug_err[0] = err[0];
				debug_err[1] = err[1];
				debug_err[2] = err[2];
				debug_err[3] = err[3];
				#endif
				
				last_smooth_time_mode0 = now;
			}
			
			Motor_PID_Control();

			reset_flap_state();
	
		}

    }
    else
    {
        motor_disable();
        reset_flap_state();
    }
}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)  //串口接收中断回调函数
{
	if(huart == &huart1)   //判断串口1中断	
	{
		ELRS_UARTE_RxCallback(Size);

	}
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
