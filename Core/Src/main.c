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

// ═══════════════════════════════════════════════════════════════════════
//  左右电机的基准点（中位角度）- Mode1平翅模式的"水平位置"
// ═══════════════════════════════════════════════════════════════════════
//
// 【物理含义】
//   当遥控器归中(Yaw=0, midpoint=0, midpoint_1=0)时，
//   翅膀应该呈现"水平"状态对应的编码器角度值。
//
// 【如何调整】⭐⭐⭐ 关键操作指南
//
//   ┌─────────────────────────────────────────────────────────────────┐
//   │  调大数值 → 翅膀向下偏（编码器值增大 = 角度向下）            │
//   │  调小数值 → 翅膀向上偏（编码器值减小 = 角度向上）            │
//   └─────────────────────────────────────────────────────────────────┘
//
//   例：motor_front_L_midpoint 当前值 = 1968
//       - 改成 1900 → 左前翅膀 向上翘 (减68)
//       - 改成 2040 → 左前翅膀 向下低 (+72)
//
// 【校准步骤】
//   1. 切换到 Mode1（平翅模式）
//   2. 遥控器完全归中（Yaw=0, midpoint=0, midpoint_1=0）
//   3. 观察翅膀是否水平
//   4. 不水平？→ 调整下面的数值 → 重新烧录测试
//
// 【V6.3.4校准记录】M3(左前)
//   测试条件：Mode=1, Yaw=-39, midpoint=0, midpoint_1=1 → 物理水平
//   此时M3 Corrective_Angle = 2069
//   反推：当Yaw=0,midpoint=0,midpoint_1=0 时 → 目标应为2069
//
// 【V6.3.5微调】归中测试后发现向下偏(M3实际=2092，目标=2069)
//   差值 = +23，说明基准值偏大，需减小约20-25
//
// ⚠️ 双电机版本：只保留 M1(右前) 和 M3(左前) 的中点值
//
const int16_t motor_front_L_midpoint = 2000;// M3(左前) 中点值
                                                // ⚠️ 调小→上翘 | 调大→下低
const int16_t motor_front_R_midpoint = 2048;   // M1(右前) 中点值
                                                // ⚠️ 调小→上翘 | 调大→下低

// ==================== 调试变量（双电机版本）====================
//
// ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
// ┃  调试数组定义表（双电机版本）                                              ┃
// ┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
// ┃  数组名        │ 索引 │ 物理电机 │ Wings_motor索引 │       说明          ┃
// ┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
// ┃  debug_err     │ [0]  │ M1(右前) │      [0]        │ M1目标-当前角度误差  ┃
// ┃  debug_err     │ [1]  │ M3(左前) │      [2]        │ M3目标-当前角度误差  ┃
// ┃  debug_current │ [0]  │ M1(右前) │      [0]        │ M1当前角度          ┃
// ┃  debug_current │ [1]  │ M3(左前) │      [2]        │ M3当前角度          ┃
// ┃  debug_pwm     │ [0]  │ M1(右前) │      [0]        │ M1的PWM输出值       ┃
// ┃  debug_pwm     │ [1]  │ M3(左前) │      [2]        │ M3的PWM输出值       ┃
// ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
//
// ⚠️ 注意：虽然Wings_motor[1]和[3]被保留用于兼容性，但实际只使用[0]和[2]

#ifdef DEBUG_MODE
// ========== STM-Studio 调试变量（独立命名，避免数组索引问题）==========
// M3(左前)电机监控变量
volatile int16_t dbg_M3_angle = 1001;      // M3 当前角度（编码器值）
volatile int16_t dbg_M3_target = 1002;     // M3 目标角度
volatile int16_t dbg_M3_err = 1003;        // M3 跟踪误差
volatile int16_t dbg_M3_pwm = 1004;        // M3 PWM 输出

// M1(右前)电机监控变量（可选，用于对比）
volatile int16_t dbg_M1_angle = 2001;      // M1 当前角度
volatile int16_t dbg_M1_err = 2002;        // M1 跟踪误差
volatile int16_t dbg_M1_pwm = 2003;        // M1 PWM 输出

// 系统状态变量
volatile uint8_t dbg_mode = 255;           // 当前模式 (0/1/2)
volatile uint8_t dbg_thr = 0;              // 当前油门值
volatile uint8_t dbg_sm_idx = 0;           // 波形表索引
volatile uint16_t dbg_flap_count = 0;      // 扑动周期计数

// 内部数组（不直接用于 STM-Studio）
volatile int16_t debug_err[2] = {0};
volatile int16_t debug_current[2] = {0};
volatile int16_t debug_pwm[2] = {0};

// ADC延迟测试专用变量（阶段3使用）
volatile uint16_t dbg_adc_raw_M3 = 0;       // M3 原始ADC值（滤波前）
volatile uint16_t dbg_adc_filtered_M3 = 0;  // M3 滤波后ADC值
volatile uint16_t dbg_adc_raw_M1 = 0;       // M1 原始ADC值（滤波前）
volatile uint16_t dbg_adc_filtered_M1 = 0;  // M1 滤波后ADC值

// 波形生成验证专用变量（新增）
volatile int16_t dbg_wave_value = 0;        // 当前读取的BIOMIMETIC_WAVE值
volatile int16_t dbg_q15_result = 0;        // q15_mul计算结果
volatile int8_t  dbg_current_idx = 0;       // 当前使用的sm_idx（递增前的值）
#endif

// 飞行控制相关结构体和函数已迁移到 flight_control.h / flight_control.c

// 电机控制辅助函数
void motor_disable(void)  // 翅膀失能（双电机版本：停止所有PWM输出）
{
    // ⚠️ 双电机版本：虽然只使用 TIM2 CH1/2 和 TIM3 CH1/2，
    //   但为了安全起见，仍清除所有8个通道的输出
    TIM2->CCR1 = 0;   // M1 正转
    TIM2->CCR2 = 0;   // M1 反转
    TIM2->CCR3 = 0;   // ~~M2~~ 已删除
    TIM2->CCR4 = 0;   // ~~M2~~ 已删除
    TIM3->CCR1 = 0;   // M3 正转
    TIM3->CCR2 = 0;   // M3 反转
    TIM3->CCR3 = 0;   // ~~M4~~ 已删除
    TIM3->CCR4 = 0;   // ~~M4~~ 已删除
}

void motor_stop(void)  // 翅膀暂停（双电机版本：设置制动状态）
{
    // ⚠️ 双电机版本：同上，设置所有通道为制动值
    TIM2->CCR1 = 19999;
    TIM2->CCR2 = 19999;
    TIM2->CCR3 = 19999;  // ~~M2~~ 已删除
    TIM2->CCR4 = 19999;  // ~~M2~~ 已删除
    TIM3->CCR1 = 19999;
    TIM3->CCR2 = 19999;
    TIM3->CCR3 = 19999;  // ~~M4~~ 已删除
    TIM3->CCR4 = 19999;  // ~~M4~~ 已删除
}

void motor_test(void)  // V6.2.1 单电机测试 - 只驱动M3(PB4+PB5)
{
    // Set_Pwm(m1, m3)
    //          ↑   ↑
    //        M1  M3 (Set_Pwm参数顺序 - 双电机版本)
    Set_Pwm(0, 3000);     // 只给M3通电
    HAL_Delay(200);             // 转200ms
    Set_Pwm(0, 0);       // 停止
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
  unsigned short Test_NUM=0;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	HAL_Delay(1000);		//等待上电完成
	MX_USART2_UART_Init();	//开启接收机串口
	ELRS_Init();			//接收机初始化
	Chassis_PID_Init(); 	//电机PID初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    // ⚠️ V6.2.1 单电机测试 - 已测完 ✅
    // motor_test();

  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // 步骤1：ADC采集滤波（每次循环都执行，确保数据最新）
    StarAndGetResult();

    // 步骤1.5：更新调试变量（每周期刷新）
    #ifdef DEBUG_MODE
    debug_current[0] = Wings_Data.Wings_motor[0].Corrective_Angle;
    debug_current[1] = Wings_Data.Wings_motor[2].Corrective_Angle;
    
    // STM-Studio 友好变量同步
    dbg_M1_angle = debug_current[0];
    dbg_M3_angle = debug_current[1];
    dbg_M3_target = Wings_Data.Wings_motor[2].Target_Angle;
    dbg_thr = thr;
    dbg_sm_idx = sm_idx;
    
    // ADC调试数据同步
    extern uint16_t AD_Value[];           // 原始ADC数组（在AS5600_PWM.c定义）
    extern float filtered_ad[];            // 滤波后数组（在AS5600_PWM.c定义）
    dbg_adc_raw_M3 = AD_Value[0];         // M3原始ADC值（PA0通道）
    dbg_adc_filtered_M3 = (uint16_t)filtered_ad[0];  // M3滤波后值
    dbg_adc_raw_M1 = AD_Value[1];         // M1原始ADC值（PA1通道）
    dbg_adc_filtered_M1 = (uint16_t)filtered_ad[1];  // M1滤波后值
    #endif

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
                // V6.5修复：模式切换时清除PID状态，防止积分项累积导致单向旋转
                PID_clear(&motor_1_pid);  // 清除M1 PID状态
                PID_clear(&motor_3_pid);  // 清除M3 PID状态
            }

            // V6.4优化：自适应PID策略
            // 根据当前扑动频率动态调整Kp/Ki/Kd参数
            // 低频(≤3Hz)：激进参数，快速响应
            // 中频(3-5Hz)：平衡参数，兼顾响应与稳定
            // 高频(>5Hz)：保守参数，优先稳定

            // 计算基准位置（双电机版本）
            const int16_t motor_front_L_ready = motor_front_L_midpoint + elrs_data.midpoint;
            const int16_t motor_front_R_ready = motor_front_R_midpoint + elrs_data.midpoint;

            // 动态转向计算
            TurnControl_t turn_ctrl;
            Calculate_Dynamic_Turn(elrs_data.Yaw, &turn_ctrl);

            // V6.9最终：优化频率范围，thr=6~10
            // throttle=6 → 2.4Hz, throttle=8 → 3.2Hz, throttle=10 → 4.0Hz
            thr = (uint8_t)elrs_data.Throttle;
            if (thr < 6) thr = 6;   // V6.9：最低6，约2.4Hz（确保升力）
            if (thr > 10) thr = 10;  // V6.9：最高10，约4.0Hz（配合MIN_STEP_MS=10ms）

            uint32_t current_tick = HAL_GetTick();

            if ((int32_t)(current_tick - sm_next_tick) >= 0)
            {
                // V6.7修复：先用当前sm_idx计算step_ms（用于判断下拍还是上挥）
                uint32_t step_ms = Calculate_Flap_Step_Time(thr);
                
                // 执行扑动步进（双电机版本）
                // 参数：yaw输入, 转向控制结构体, M3(左前)基准位置, M1(右前)基准位置
                Execute_Flap_Step(elrs_data.Yaw, &turn_ctrl,
                                  motor_front_L_ready,  // M3(左前)中点+微调
                                  motor_front_R_ready); // M1(右前)中点+微调
                
                // 【关键修复】Execute_Flap_Step已经让sm_idx递增了
                // 现在用新的sm_idx重新计算step_ms，用于下一次步进的时间安排
                // 这样下拍(快)和上挥(慢)才能真正有不同的步进时间！
                step_ms = Calculate_Flap_Step_Time(thr);
                
                // V6.7修复：正确计算扑动频率
                // 下拍：14步（索引0-13），上挥：11步（索引14-24）
                // 总周期 = 下拍时间 + 上挥时间
                uint32_t step_ms_down = Calculate_Biomimetic_Step_Time(thr, 1);
                uint32_t step_ms_up = Calculate_Biomimetic_Step_Time(thr, 0);
                uint32_t total_period_ms = step_ms_down * 14 + step_ms_up * 11;
                float current_freq = 1000.0f / total_period_ms;
                
                // 自适应PID参数调整
                Adapt_PID_For_Frequency(current_freq, &motor_1_pid, &motor_3_pid);

                // 步骤2：姿态估计与电机同步补偿（每20ms执行一次，50Hz）
                if ((int32_t)(current_tick - attitude_tick) >= 20) {
                    attitude_tick = current_tick;
                    
                    // 姿态估计 - 计算各电机位置和差异
                    Estimate_Attitude();
                    
                    // 电机同步补偿 - 使2电机保持同步（双电机版本）
                    Motor_Sync_Compensate();
                }

                // 安排下一次步进（用修复后的step_ms）
                sm_next_tick += step_ms;

                // 记录调试数据
                #ifdef DEBUG_MODE
                debug_err[0] = Wings_Data.Wings_motor[0].Target_Angle - Wings_Data.Wings_motor[0].Corrective_Angle;
                debug_err[1] = Wings_Data.Wings_motor[2].Target_Angle - Wings_Data.Wings_motor[2].Corrective_Angle;
                
                // STM-Studio 友好变量同步
                dbg_M1_err = debug_err[0];
                dbg_M3_err = debug_err[1];
                dbg_M3_target = Wings_Data.Wings_motor[2].Target_Angle;
                dbg_mode = 2;
                #endif
            }

            Motor_PID_Control();
        }
        else if (elrs_data.Mode == 1)
		{
			static uint8_t last_mode_1 = 255;
			// Mode1平翅模式 - 梯形速度规划改进版（参考Mode0逻辑）
			if (last_mode_1 != 1) {
				last_mode_1 = 1;
				// V6.5修复：模式切换时清除PID状态
				PID_clear(&motor_1_pid);
				PID_clear(&motor_3_pid);
			}
			
			// V6.10修复：每次循环强制设置Mode1的PID参数，防止被Mode2的动态PID覆盖
			// 从Mode2切换时，确保立即恢复Mode1的高Kp值
			if (motor_1_pid.Kp != 9.0f || motor_3_pid.Kp != 9.0f) {
				motor_1_pid.Kp = 9.0f;
				motor_1_pid.Ki = 0.01f;
				motor_1_pid.Kd = 7.5f;
				motor_3_pid.Kp = 9.0f;
				motor_3_pid.Ki = 0.01f;
				motor_3_pid.Kd = 7.5f;
			}

			// 基准角计算（双电机版本 - 带转向差动）
            // 注意：编码器值减小=翅膀上移（升力减小），编码器值增大=翅膀下移（升力增大）
            // 左转（Yaw<0）：右翼下移（升力增大），左翼上移（升力减小）
            // 右转（Yaw>0）：左翼下移（升力增大），右翼上移（升力减小）
            const int16_t target_motor0 = motor_front_R_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 - 2*elrs_data.Yaw;  // 右前
            const int16_t target_motor2 = motor_front_L_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 + 2*elrs_data.Yaw;  // 左前

			// 梯形速度规划参数（V6.4.6防超调）
			const uint32_t SMOOTH_INTERVAL = 5;   // 5ms更新周期
			const int16_t MAX_VELOCITY = 45;      // ⭐ 降速：50→45（减少冲出距离）
			const int16_t ACCEL_LIMIT = 6;        // ⭐ 柔和加速：7→6

			// ⭐【V6.4.6】死区参数：提前减速 + 保持稳定迟滞
			const int16_t HARD_DEADZONE_ENTER = 40;   // 保持40
			const int16_t HARD_DEADZONE_EXIT = 60;    // 保持60，带宽=20
			const int16_t SOFT_DEADZONE = 150;        // ⭐ 提前减速：130→150
			
			// 静态变量保持状态（双电机版本）
			static uint32_t last_smooth_time_mode1 = 0;
			static int16_t current_vel_mode1[2] = {0, 0};     // [0]=M1速度, [1]=M3速度
			static uint8_t last_mode_mode1 = 255;             // 上次模式，用于检测模式切换
			static uint8_t in_hard_deadzone_mode1[2] = {0, 0}; // [0]=M1死区状态, [1]=M3死区状态
			
			// 索引映射宏：Wings_motor索引(0/2) → 数组索引(0/1)
			#define MOTOR_IDX_TO_ARRAY_IDX_MODE1(motor_idx) ((motor_idx) == 2 ? 1 : 0)
			
			// 检测是否刚进入Mode1，如果是则清零速度和死区状态
			if (last_mode_mode1 != 1) {
				current_vel_mode1[0] = 0;  // M1
				current_vel_mode1[1] = 0;  // M3
				in_hard_deadzone_mode1[0] = 0;  // M1
				in_hard_deadzone_mode1[1] = 0;  // M3
				last_mode_mode1 = 1;
			}

			if ((int32_t)(now - last_smooth_time_mode1) >= SMOOTH_INTERVAL)
			{
				// 计算误差（双电机版本）
				int16_t err[4];
				err[0] = target_motor0 - Wings_Data.Wings_motor[0].Corrective_Angle;  // 右前M1
				err[2] = target_motor2 - Wings_Data.Wings_motor[2].Corrective_Angle;  // 左前M3
				
				// 目标值数组（双电机版本）
				int16_t targets[4] = {target_motor0, 0, target_motor2, 0};
				
				// 只处理M1(索引0)和M3(索引2)两个电机
				for (uint8_t i = 0; i < 4; i += 2)  // 步长为2，只处理索引0和2
				{
					uint8_t arr_idx = MOTOR_IDX_TO_ARRAY_IDX_MODE1(i);  // 索引映射：0→0, 2→1
					int16_t error = err[i];
					int16_t abs_err = abs16_fast(error);
					
					// ⭐【V6.3.4修复】带迟滞的硬死区判断（与Mode0一致）
					// ⚠️ 关键：死区内必须让 Target_Angle = Corrective_Angle（当前位置）
					if (!in_hard_deadzone_mode1[arr_idx] && abs_err <= HARD_DEADZONE_ENTER)
					{
						in_hard_deadzone_mode1[arr_idx] = 1;
						Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle;  // ⭐ 目标=实际位置
						current_vel_mode1[arr_idx] = 0;

						// 清零PID积分项（双重保险）
						if (i == 0) {
							PID_clear(&motor_1_pid);  // M1
						} else if (i == 2) {
							PID_clear(&motor_3_pid);  // M3
						}

						continue;
					}
					else if (in_hard_deadzone_mode1[arr_idx] && abs_err > HARD_DEADZONE_EXIT)
					{
						in_hard_deadzone_mode1[arr_idx] = 0;
					}
					else if (in_hard_deadzone_mode1[arr_idx])
					{
						Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle;  // ⭐ 保持目标=实际位置
						current_vel_mode1[arr_idx] = 0;

						// 持续清零PID，确保死区内输出为0
						if (i == 0) {
							PID_clear(&motor_1_pid);
						} else if (i == 2) {
							PID_clear(&motor_3_pid);
						}

						continue;
					}
					
					// 软死区减速
					int16_t desired_vel;
					if (abs_err < SOFT_DEADZONE)
					{
						desired_vel = (int16_t)((int32_t)error * abs_err / SOFT_DEADZONE);
					}
					else
					{
						desired_vel = error;
					}
					
					// 限制最大速度
					if (desired_vel > MAX_VELOCITY) desired_vel = MAX_VELOCITY;
					if (desired_vel < -MAX_VELOCITY) desired_vel = -MAX_VELOCITY;
					
					// 梯形速度规划：限制加速度
					int16_t vel_diff = desired_vel - current_vel_mode1[arr_idx];
					if (vel_diff > ACCEL_LIMIT) vel_diff = ACCEL_LIMIT;
					if (vel_diff < -ACCEL_LIMIT) vel_diff = -ACCEL_LIMIT;
					
					current_vel_mode1[arr_idx] += vel_diff;
					
					Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle + current_vel_mode1[arr_idx];
				}
				
				// 记录调试数据（双电机版本）
				#ifdef DEBUG_MODE
				debug_err[0] = err[0];  // M1(右前)误差
				debug_err[1] = err[2];  // M3(左前)误差
				#endif
				
				last_smooth_time_mode1 = now;
			}
		
			Motor_PID_Control();
			reset_flap_state();

		}
        else if (elrs_data.Mode == 4)
        {
            static uint8_t last_mode_4 = 255;
            static int16_t current_speed[2] = {0, 0};

            if (last_mode_4 != 4) {
                last_mode_4 = 4;
                current_speed[0] = 0;
                current_speed[1] = 0;
                PID_clear(&motor_1_pid);
                PID_clear(&motor_3_pid);
            }

            // ==================== Mode4 电机持续旋转控制 ====================
            //
            // 【速度计算】
            // target_speed = Throttle × 600
            // Throttle 范围: 5~15 → target_speed: 3000~9000
            //
            // 【速度限制】
            // 最低转速: 3000 (约25%占空比)
            // 最高转速: 12000 (约100%占空比)
            //
            // 【转向控制】
            // Yaw < -20: 电机正转（两翼同向）
            // Yaw > +20: 电机反转（两翼反向）
            // -20 ≤ Yaw ≤ 20: 电机逐渐减速停止
            //
            // 【注意】Mode4 是开环速度控制，不经过 PID 位置闭环
            // ================================================================

            int16_t target_speed = (int16_t)elrs_data.Throttle * 600;
            if (target_speed < 3000) target_speed = 3000;
            if (target_speed > 12000) target_speed = 12000;

            if (elrs_data.Yaw < -20) {
                current_speed[0] = target_speed;
                current_speed[1] = target_speed;
            } else if (elrs_data.Yaw > 20) {
                current_speed[0] = -target_speed;
                current_speed[1] = -target_speed;
            } else {
                if (current_speed[0] < 0) current_speed[0] += 200;
                else if (current_speed[0] > 0) current_speed[0] -= 200;
                if (abs(current_speed[0]) < 100) current_speed[0] = 0;

                if (current_speed[1] < 0) current_speed[1] += 200;
                else if (current_speed[1] > 0) current_speed[1] -= 200;
                if (abs(current_speed[1]) < 100) current_speed[1] = 0;
            }

            int16_t m1_pwm = -current_speed[0];
            int16_t m3_pwm = current_speed[1];
            Set_Pwm(m1_pwm, m3_pwm);

            #ifdef DEBUG_MODE
            dbg_mode = 4;
            dbg_M1_pwm = m1_pwm;
            dbg_M3_pwm = m3_pwm;
            #endif
        }
        else
		{
			static uint8_t last_mode_0 = 255;
			// Mode0并翅模式 - 梯形速度规划改进版
			if (last_mode_0 != 0) {
				last_mode_0 = 0;
				Test_NUM=0;
				// V6.5修复：模式切换时清除PID状态
				PID_clear(&motor_1_pid);
				PID_clear(&motor_3_pid);
			}
			
			// V6.10修复：每次循环强制设置Mode0的PID参数，防止被Mode2的动态PID覆盖
			if (motor_1_pid.Kp != 10.0f || motor_3_pid.Kp != 10.0f) {
				motor_1_pid.Kp = 10.0f;
				motor_1_pid.Ki = 0.02f;
				motor_1_pid.Kd = 5.0f;
				motor_3_pid.Kp = 10.0f;
				motor_3_pid.Ki = 0.02f;
				motor_3_pid.Kd = 5.0f;
			}
//         motor_3_pid.Kp=motor_1_pid.Kp;
//			motor_3_pid.Ki=motor_1_pid.Ki;
//			motor_3_pid.Kd=motor_1_pid.Kd;
			// 目标角度 - 统一为1024（竖直并翅）
			// 注意：数组索引与电机位置的对应关系（双电机版本）
			// Wings_motor[0] = 右前
			// Wings_motor[2] = 左前
			const int16_t target_motor0 = 1024;  // 右前
			const int16_t target_motor2 = 1024;  // 左前
			
			// 梯形速度规划参数（V6.4.5回归基线 + 适度提速）
			const uint32_t SMOOTH_INTERVAL = 5;   // 5ms更新周期
			const int16_t MAX_VELOCITY = 55;      // 基线50→55 (+10%)
			const int16_t ACCEL_LIMIT = 8;        // 基线7→8 (+14%)

			// ⭐【V6.4.5】死区参数：回归V6.3.x稳定值
			const int16_t HARD_DEADZONE_ENTER = 40;   // 回归40
			const int16_t HARD_DEADZONE_EXIT = 60;    // 回归60，带宽=20
			const int16_t SOFT_DEADZONE = 110;        // 基线120→110 (适度提速)
			
			// 静态变量保持状态（双电机版本优化：数组大小从[4]减至[2]）
			static uint32_t last_smooth_time_mode0 = 0;
			static int16_t current_vel[2] = {0, 0};           // [0]=M1速度, [1]=M3速度
			static uint8_t last_mode = 255;  // 上次模式，用于检测模式切换
			static uint8_t in_hard_deadzone[2] = {0, 0};     // [0]=M1死区状态, [1]=M3死区状态
			
			// 索引映射宏：Wings_motor索引(0/2) → 数组索引(0/1)
			#define MOTOR_IDX_TO_ARRAY_IDX(motor_idx) ((motor_idx) == 2 ? 1 : 0)
			
			// 检测是否刚进入Mode0，如果是则清零速度和死区状态
			if (last_mode != 0) {
				current_vel[0] = 0;  // M1
				current_vel[1] = 0;  // M3
				in_hard_deadzone[0] = 0;  // M1
				in_hard_deadzone[1] = 0;  // M3
				last_mode = 0;
			}

			if ((int32_t)(now - last_smooth_time_mode0) >= SMOOTH_INTERVAL)
			{
				 Test_NUM++;
				// 计算误差（双电机版本）
					int16_t err[4];
					err[0] = target_motor0 - Wings_Data.Wings_motor[0].Corrective_Angle;  // 右前M1
					err[2] = target_motor2 - Wings_Data.Wings_motor[2].Corrective_Angle;  // 左前M3
					
					// 目标值数组（双电机版本）
					int16_t targets[4] = {target_motor0, 0, target_motor2, 0};
					
					// 只处理M1(索引0)和M3(索引2)两个电机
					for (uint8_t i = 0; i < 4; i += 2)  // 步长为2，只处理索引0和2
					{
						uint8_t arr_idx = MOTOR_IDX_TO_ARRAY_IDX(i);  // 索引映射：0→0, 2→1
						int16_t error = err[i];
						int16_t abs_err = abs16_fast(error);
						
						// ⭐【V6.3.4修复】带迟滞的硬死区判断
						// ⚠️ 关键：死区内必须让 Target_Angle = Corrective_Angle（当前位置）
						//   而不是 targets[i]（目标值）！否则PID误差≠0，PWM仍会输出！
						if (!in_hard_deadzone[arr_idx] && abs_err <= HARD_DEADZONE_ENTER)
						{
							in_hard_deadzone[arr_idx] = 1;
							Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle;  // ⭐ 目标=实际位置，误差=0
							current_vel[arr_idx] = 0;

							// 清零PID积分项（双重保险）
							if (i == 0) {
								PID_clear(&motor_1_pid);  // M1
							} else if (i == 2) {
								PID_clear(&motor_3_pid);  // M3
							}

							continue;
						}
						else if (in_hard_deadzone[arr_idx] && abs_err > HARD_DEADZONE_EXIT)
						{
							in_hard_deadzone[arr_idx] = 0;
						}
						else if (in_hard_deadzone[arr_idx])
						{
							Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle;  // ⭐ 保持目标=实际位置
							current_vel[arr_idx] = 0;

							// 持续清零PID，确保死区内输出为0
							if (i == 0) {
								PID_clear(&motor_1_pid);
							} else if (i == 2) {
								PID_clear(&motor_3_pid);
							}

							continue;
						}
						
						int16_t desired_vel;
						if (abs_err < SOFT_DEADZONE)
						{
							desired_vel = (int16_t)((int32_t)error * abs_err / SOFT_DEADZONE);
						}
						else
						{
							desired_vel = error;
						}
						
						if (desired_vel > MAX_VELOCITY) desired_vel = MAX_VELOCITY;
						if (desired_vel < -MAX_VELOCITY) desired_vel = -MAX_VELOCITY;
						
						int16_t vel_diff = desired_vel - current_vel[arr_idx];
						if (vel_diff > ACCEL_LIMIT) vel_diff = ACCEL_LIMIT;
						if (vel_diff < -ACCEL_LIMIT) vel_diff = -ACCEL_LIMIT;
						
						current_vel[arr_idx] += vel_diff;
						
						Wings_Data.Wings_motor[i].Target_Angle = Wings_Data.Wings_motor[i].Corrective_Angle + current_vel[arr_idx];
					}
					
					// 记录调试数据（双电机版本）
					#ifdef DEBUG_MODE
					debug_err[0] = err[0];  // M1(右前)误差
					debug_err[1] = err[2];  // M3(左前)误差
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
	if(huart == &huart2)   //判断串口2中断	
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
