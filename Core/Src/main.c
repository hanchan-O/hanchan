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
#define DEBUG_MODE

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 左右电机的基准点（中位角度//调小翅膀向上）
// 使用const存储在Flash中，节省RAM
const int16_t motor_front_L_midpoint = 2054;
const int16_t motor_front_R_midpoint = 2030;
const int16_t motor_back_L_midpoint  = 2060;
const int16_t motor_back_R_midpoint  = 2000;

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
            // ==================== Mode2: 完整版（带姿态控制） ====================
            static uint8_t last_mode = 255;
            static uint32_t attitude_tick = 0;  // 姿态估计分频计时
            if (last_mode != 2) {
                last_mode = 2;
                reset_flap_state();  // 重置扑动状态
                // 【V6.1差异化】切换到Mode 2时，恢复扑动最优参数
                // 从motor.h读取：Kp=18, Ki=0.08, Kd=3.0
                for (int i = 0; i < 4; i++) {
                    pid_type_def* pid_list[4] = {&motor_1_pid, &motor_2_pid, &motor_3_pid, &motor_4_pid};
                    pid_list[i]->Kp = motor_pid_params[i][0];  // Kp=18
                    pid_list[i]->Ki = motor_pid_params[i][1];  // Ki=0.08
                    pid_list[i]->Kd = motor_pid_params[i][2];  // Kd=3.0
                }
            }

            // 【V6.1差异化PID策略】
            // Mode 2是高速扑动模式，使用motor.h中的最优参数(Kp=18,Ki=0.08,Kd=3.0)
            // 无需在此处覆盖，PD+前馈控制已针对7.4V锂电池和扑动优化

            // 计算基准位置
            const int16_t motor_front_L_ready = motor_front_L_midpoint + elrs_data.midpoint;
            const int16_t motor_front_R_ready = motor_front_R_midpoint + elrs_data.midpoint;
            const int16_t motor_back_L_ready  = motor_back_L_midpoint  + elrs_data.midpoint;
            const int16_t motor_back_R_ready  = motor_back_R_midpoint  + elrs_data.midpoint;

            // 动态转向计算
            TurnControl_t turn_ctrl;
            Calculate_Dynamic_Turn(elrs_data.Yaw, &turn_ctrl);

            // 扑动频率：使用Throttle通道(5-15)映射到频率
            thr = (uint8_t)elrs_data.Throttle;  // 5-15
            if (thr < 5) thr = 5;
            if (thr > 15) thr = 15;
            uint32_t step_ms = Calculate_Flap_Step_Time(thr);

            uint32_t current_tick = HAL_GetTick();

            if ((int32_t)(current_tick - sm_next_tick) >= 0)
            {
                // 执行扑动步进
                Execute_Flap_Step(elrs_data.Yaw, &turn_ctrl,
                                  motor_front_L_ready,
                                  motor_front_R_ready,
                                  motor_back_L_ready,
                                  motor_back_R_ready);

                // 步骤2：姿态估计与电机同步补偿（每20ms执行一次，50Hz）
                if ((int32_t)(current_tick - attitude_tick) >= 20) {
                    attitude_tick = current_tick;
                    
                    // 姿态估计 - 计算各电机位置和差异
                    Estimate_Attitude();
                    
                    // 电机同步补偿 - 使4电机保持同步
                    Motor_Sync_Compensate();
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
			if (last_mode != 1) {
				last_mode = 1;
				// 【V6.1差异化】Mode 1切换时，应用保守PID参数（防止静态定位抖动）
				// Kp=10: 降低响应速度，提高稳定性
				// Ki=0.05: 很小的积分项，消除稳态误差但不会积分饱和
				// Kd=2.0: 适度微分，抑制超调
				for (int i = 0; i < 4; i++) {
					pid_type_def* pid_list[4] = {&motor_1_pid, &motor_2_pid, &motor_3_pid, &motor_4_pid};
					pid_list[i]->Kp = 10.0f;
					pid_list[i]->Ki = 0.05f;
					pid_list[i]->Kd = 2.0f;
				}
			}

			// 基准角计算（带转向差动）
			// 注意：编码器值减小=翅膀上移（升力减小），编码器值增大=翅膀下移（升力增大）
			// 左转（Yaw<0）：右翼下移（升力增大），左翼上移（升力减小）
			// 右转（Yaw>0）：左翼下移（升力增大），右翼上移（升力减小）
			const int16_t front_targetL = motor_front_L_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 + 2*elrs_data.Yaw;
			const int16_t front_targetR = motor_front_R_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 - 2*elrs_data.Yaw;
			const int16_t back_targetL  = motor_back_L_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 + 2*elrs_data.Yaw;
			const int16_t back_targetR  = motor_back_R_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 - 2*elrs_data.Yaw;

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
			static uint8_t last_mode_0 = 255;
			// Mode0并翅模式 - 梯形速度规划改进版
			if (last_mode_0 != 0) {
				last_mode_0 = 0;
				// 【V6.1差异化】Mode 0切换时，应用保守PID参数（防止静态定位抖动）
				// 并翅模式是静态定位，不需要快速响应，稳定性优先
				for (int i = 0; i < 4; i++) {
					pid_type_def* pid_list[4] = {&motor_1_pid, &motor_2_pid, &motor_3_pid, &motor_4_pid};
					pid_list[i]->Kp = 10.0f;
					pid_list[i]->Ki = 0.05f;
					pid_list[i]->Kd = 2.0f;
				}
			}

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
