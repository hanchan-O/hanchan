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

// 左右电机的基准点（中位角）//调小翅膀向上
// 使用const存储在Flash中，节省RAM
const int16_t motor_front_L_midpoint = 1924;
const int16_t motor_front_R_midpoint = 2000;
const int16_t motor_back_L_midpoint  = 2000;
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
#if MOTOR_HW_USE_PWM_TIM2_CH12
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
#endif
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
#if MOTOR_HW_USE_PWM_TIM3_CH34
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
#endif
}

void motor_stop(void)  // 翅膀暂停
{
#if MOTOR_HW_USE_PWM_TIM2_CH12
    TIM2->CCR1 = 19999;
    TIM2->CCR2 = 19999;
#endif
    TIM2->CCR3 = 19999;
    TIM2->CCR4 = 19999;
    TIM3->CCR1 = 19999;
    TIM3->CCR2 = 19999;
#if MOTOR_HW_USE_PWM_TIM3_CH34
    TIM3->CCR3 = 19999;
    TIM3->CCR4 = 19999;
#endif
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
  /* USART1：PB6=TX、PB7=RX（AF0_USART1），波特率 420000，RX DMA=DMA1_Ch1（见 usart.c） */
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  /* ADC 已停用（MT6701 改 TIM1 输入捕获），不再调用 MX_ADC1_Init；adc.c 可保留空实现备 Cube 同步 */
  /* USER CODE BEGIN 2 */
	/* TIM2 四路 PWM -> M1/M2；引脚 AF2（见 tim.c HAL_TIM_MspPostInit TIM2）
	 * CH1=PA15(M1_1)  CH2=PB3(M1_2)  CH3=PC6(M2_1)  CH4=PA3(M2_2) */
	HAL_TIM_Base_Start(&htim2);
#if MOTOR_HW_USE_PWM_TIM2_CH12
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* PA15 M1 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); /* PB3  */
#endif
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); /* PC6 M2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); /* PA3  */

	/* TIM3：M3(PB4/PB5)；M4(PB0/PB1) 仅在四桥模式启用 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); /* PB4 M3_1 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); /* PB5 M3_2 */
#if MOTOR_HW_USE_PWM_TIM3_CH34
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); /* PB0 M4_1 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); /* PB1 M4_2 */
#endif

	/* TIM14：仅内部时基+中断，无 PWM 引脚输出（原 TIM1 时基迁至 TIM14，见 tim.c） */
	HAL_TIM_Base_Start_IT(&htim14);

	/* TIM1 四路输入捕获 -> MT6701 PWM；引脚 AF1，具体见 tim.h 宏 MT6701_TIM1_CH12_USE_PA8_PA9
	 * =0：CH1=PA0 CH2=PA1 CH3=PA10 CH4=PA11
	 * =1：CH1=PA8 CH2=PA9 CH3=PA10 CH4=PA11 */
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
#if MOTOR_HW_USE_TIM1_IC_CH34
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
#endif

	/* 上电后延时，等待电源与外设稳定 */
	HAL_Delay(1000);
	/* 再次 USART1：引脚仍为 PB6/PB7，供 ELRS 接收机串口（与上面 MX_USART1_UART_Init 硬件相同） */
	MX_USART1_UART_Init();
	ELRS_Init();			/* ELRS 接收机协议与状态初始化 */
	Chassis_PID_Init();		/* 底盘电机 PID 参数与状态初始化 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // 四路 MT6701 PWM：TIM1 IC 输入捕获
    StarAndGetResult();

    /* Failsafe：检测 ELRS/CRSF 遥控链路（USART1）是否超时。
     * last_signal_time 仅在 elrs.c 收到有效 RC 通道帧时刷新；与 MT6701 磁编码器/TIM1 无关。 */
    if ((int32_t)(now - last_signal_time) > SIGNAL_TIMEOUT_MS)
    {
        /* 超过 SIGNAL_TIMEOUT_MS（500ms）未收到遥控帧：视为遥控器链路丢失 */
        motor_disable();
        reset_flap_state();
        continue;  /* 跳过本次循环，不执行后续飞控逻辑 */
    }

    /* elrs_data 来自 CRSF：channels[4]=Switch，channels[5]=Mode（见 elrs.c 解析）
     * Switch==0：遥控关控，走下方 else 分支失能电机
     * Switch==1 且 Mode：0=并翅  1=平翅  2=扑动（带姿态） */
    if (elrs_data.Switch == 1)
    {

        if (elrs_data.Mode == 2)
        {
            /* ---------- Mode2：扑动模式（完整版，带姿态估计与四电机同步） ----------
             * 用 Throttle 映射扑动频率，Yaw 参与转向；sm_next_tick 控制扑动步进时刻；
             * 每圈扑动内调用 Execute_Flap_Step；另以 50Hz 做 Estimate_Attitude + Motor_Sync_Compensate */
            static uint8_t last_mode = 255;
            static uint32_t attitude_tick = 0;  /* 上次做姿态估计的 tick，用于 20ms 分频 */
            if (last_mode != 2) {
                last_mode = 2;
                reset_flap_state();  /* 切入扑动时清空扑动状态机，避免残留相位 */
                Motor_Flap_Slew_Reset(); /* 目标软跟随初值对齐当前角度，避免切入瞬间猛拉 */
            }

            /* 四路电机 PID 的 Kp（可按电池电压与抖动情况微调） */
            motor_1_pid.Kp = 15;  /* 电机索引0 右前 */
            motor_2_pid.Kp = 15;  /* 电机索引1 左后 */
            motor_3_pid.Kp = 12;   /* 电机索引2 左前 */
            motor_4_pid.Kp = 12;   /* 电机索引3 右后 */

            /* 在各自中位基础上叠加 elrs 映射的 midpoint，得到四翼“就绪”目标基准 */
            const int16_t motor_front_L_ready = motor_front_L_midpoint + elrs_data.midpoint;
            const int16_t motor_front_R_ready = motor_front_R_midpoint + elrs_data.midpoint;
            const int16_t motor_back_L_ready  = motor_back_L_midpoint  + elrs_data.midpoint;
            const int16_t motor_back_R_ready  = motor_back_R_midpoint  + elrs_data.midpoint;

            /* 油门 FLAP_THR_MAP_MIN~MAX → 扑动频率 FLAP_FREQ_MIN_cHz~MAX_cHz（见 flight_control.h） */
            thr = (uint8_t)elrs_data.Throttle;
            if (thr < FLAP_THR_MAP_MIN) thr = FLAP_THR_MAP_MIN;
            if (thr > FLAP_THR_MAP_MAX) thr = FLAP_THR_MAP_MAX;

            /* 由 Yaw、油门计算转向与扑动幅度相关量，供 Execute_Flap_Step 使用 */
            TurnControl_t turn_ctrl;
            Calculate_Dynamic_Turn(elrs_data.Yaw, thr, &turn_ctrl);

            uint32_t step_ms = Calculate_Flap_Step_Time(thr);

            uint32_t current_tick = HAL_GetTick();

            if ((int32_t)(current_tick - sm_next_tick) >= 0)
            {
                /* 按当前遥控量推进一拍扑动（四电机目标角由扑动状态机更新） */
                Execute_Flap_Step(elrs_data.Yaw, &turn_ctrl,
                                  motor_front_L_ready,
                                  motor_front_R_ready,
                                  motor_back_L_ready,
                                  motor_back_R_ready);

                /* 与扑动节拍独立：每 20ms 根据编码器反馈估计姿态，并微调四电机目标使相位对齐 */
                if ((int32_t)(current_tick - attitude_tick) >= 20) {
                    attitude_tick = current_tick;
                    
                    Estimate_Attitude();
                    
                    Motor_Sync_Compensate();
                }

                sm_next_tick += step_ms;  /* 下一拍扑动时刻 */

                #ifdef DEBUG_MODE
                /* 各电机：目标角 - 当前校正角（跟踪误差），供在线观察 */
                debug_err[0] = Wings_Data.Wings_motor[0].Target_Angle - Wings_Data.Wings_motor[0].Corrective_Angle;
                debug_err[1] = Wings_Data.Wings_motor[1].Target_Angle - Wings_Data.Wings_motor[1].Corrective_Angle;
                debug_err[2] = Wings_Data.Wings_motor[2].Target_Angle - Wings_Data.Wings_motor[2].Corrective_Angle;
                debug_err[3] = Wings_Data.Wings_motor[3].Target_Angle - Wings_Data.Wings_motor[3].Corrective_Angle;
                #endif
            }

            Motor_PID_Control_Flap();  /* 扑动：先对指令目标做限速跟随，再 PID 输出 PWM */
        }
        else if (elrs_data.Mode == 1)
		{
			/* ---------- Mode1：平翅模式（四翼共面偏转，带 Yaw 差动，无扑动状态机） ----------
			 * 目标角由 front/back + midpoint + Yaw 组合算出；每 5ms 对 Target 做限幅爬升（MAX_STEP） */
			static uint8_t last_mode = 255;
			if (last_mode != 1) last_mode = 1;
			
			/* 平翅用较低 Kp，避免高压下振荡（与 Mode2 数值可分开调） */
			motor_1_pid.Kp = 8;   // 电机 0(右前)（原 12）
			motor_2_pid.Kp = 8;   // 电机 1(左后)（原 12）
			motor_3_pid.Kp = 8;   // 电机 2(左前)（原 12）
			motor_4_pid.Kp = 8;   // 电机 3(右后)（原 12）
			/* 四路目标角 = 该机翼中位 + Pitch 映射 midpoint + |Yaw| 映射 midpoint_1 ± 偏航差动(2*Yaw)
			 * motor_*_midpoint：各电机标定中位（左前/右前/左后/右后 各不同）
			 * midpoint：前后微调，四路同加（elrs Pitch 映射）
			 * midpoint_1：转弯越大附加越多（elrs 由 |Yaw| 映射，见 elrs.c）
			 * ±2*Yaw：左半翼 +2*Yaw、右半翼 -2*Yaw，形成左右差动；系数 2 为转向增益
			 * 编码器：值减小≈翼上抬升力减小，值增大≈翼下压升力增大 */
			const int16_t front_targetL = motor_front_L_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 + 2*elrs_data.Yaw;  /* 左前 */
			const int16_t front_targetR = motor_front_R_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 - 2*elrs_data.Yaw;  /* 右前 */
			const int16_t back_targetL  = motor_back_L_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 + 2*elrs_data.Yaw;  /* 左后 */
			const int16_t back_targetR  = motor_back_R_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 - 2*elrs_data.Yaw;  /* 右后 */

			// 从当前值缓慢移动到目标值
			static uint32_t last_smooth_time = 0;
			const uint32_t SMOOTH_INTERVAL = 5; // 平滑间隔 5ms
			const int16_t MAX_STEP = 60;        // 最大步进值
			
			if ((int32_t)(now - last_smooth_time) >= SMOOTH_INTERVAL)
			{
				// 电机0（右前）-> 使用 front_targetR（右前目标）
				int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
				if (current0 < front_targetR) {
					current0 += MIN(MAX_STEP, front_targetR - current0);
				} else if (current0 > front_targetR) {
					current0 -= MIN(MAX_STEP, current0 - front_targetR);
				}
				Wings_Data.Wings_motor[0].Target_Angle = current0;
				
				// 电机2（左前）-> 使用 front_targetL（左前目标）
				int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
				if (current2 < front_targetL) {
					current2 += MIN(MAX_STEP, front_targetL - current2);
				} else if (current2 > front_targetL) {
					current2 -= MIN(MAX_STEP, current2 - front_targetL);
				}
				Wings_Data.Wings_motor[2].Target_Angle = current2;
				
				// 电机1（左后）-> 使用 back_targetL（左后目标）
				int16_t current1 = Wings_Data.Wings_motor[1].Corrective_Angle;
				if (current1 < back_targetL) {
					current1 += MIN(MAX_STEP, back_targetL - current1);
				} else if (current1 > back_targetL) {
					current1 -= MIN(MAX_STEP, current1 - back_targetL);
				}
				Wings_Data.Wings_motor[1].Target_Angle = current1;
				
				// 电机3（右后）-> 使用 back_targetR（右后目标）
				int16_t current3 = Wings_Data.Wings_motor[3].Corrective_Angle;
				if (current3 < back_targetR) {
					current3 += MIN(MAX_STEP, back_targetR - current3);
				} else if (current3 > back_targetR) {
					current3 -= MIN(MAX_STEP, current3 - back_targetR);
				}
				Wings_Data.Wings_motor[3].Target_Angle = current3;
				
				// 记录调试数据 - Mode1 平翅模式误差（目标-当前）
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
			/* ---------- Mode0：并翅模式（四翼收拢至竖直附近，目标角恒为 1024） ----------
			 * 用梯形速度规划：先算位置误差，再限制期望速度、加速度，使运动平滑无冲击 */
			motor_1_pid.Kp = 10;   /* 并翅可适当提高 Kp 以加快到位 */
			motor_2_pid.Kp = 10;   // 电机1(左后)
			motor_3_pid.Kp = 10;   // 电机2(左前)
			motor_4_pid.Kp = 10;   // 电机3(右后) - 增加Kp以加快响应
			
			// 目标角度 - 统一为 1024（竖直并翅）
			// 注意：数组索引与电机位置的对应关系
			// Wings_motor[0] = 右前, Wings_motor[1] = 左后
			// Wings_motor[2] = 左前, Wings_motor[3] = 右后
			const int16_t target_motor0 = 1024;  // 右前
			const int16_t target_motor1 = 1024;  // 左后
			const int16_t target_motor2 = 1024;  // 左前
			const int16_t target_motor3 = 1024;  // 右后
			
			// 梯形速度规划参数
			const uint32_t SMOOTH_INTERVAL = 5;   // 5ms 更新周期
			const int16_t MAX_VELOCITY = 60;       // 最大速度（编码器值/周期）
			const int16_t ACCEL_LIMIT = 8;         // 加速度限制（速度变化量/周期）
			const int16_t HARD_DEADZONE = 10;       // 硬死区（直接停止）
			const int16_t SOFT_DEADZONE = 40;      // 软死区（开始减速）
			
			// 静态变量保持状态
			static uint32_t last_smooth_time_mode0 = 0;
			static int16_t current_vel[4] = {0, 0, 0, 0};  // 当前速度
			static uint8_t last_mode = 255;  // 上次模式，用于检测模式切换
			
			// 检测是否刚进入 Mode0，若是则清零速度
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
					
					// 梯形速度规划：限制加速度（平滑启停）
					int16_t vel_diff = desired_vel - current_vel[i];
					if (vel_diff > ACCEL_LIMIT) vel_diff = ACCEL_LIMIT;
					if (vel_diff < -ACCEL_LIMIT) vel_diff = -ACCEL_LIMIT;
					
					// 更新当前速度
					current_vel[i] += vel_diff;
					
					// 计算新的目标角度
					Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle + current_vel[i];
				}
				
				// 记录调试数据 - 使用 err 误差值更直观
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
        /* Switch==0：遥控判定为关闭，不执行上述 Mode，直接关 PWM 并清扑动状态 */
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14)
  {
    /* TIM14 周期中断（原 TIM1 时基相关，见 tim.c） */
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)  // 串口接收中断回调
{
	if(huart == &huart1)   // 判断串口1
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
