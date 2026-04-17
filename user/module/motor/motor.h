#ifndef __MOTOR_H
#define __MOTOR_H


#include "pid.h"
#include "main.h"
#include "AS5600_PWM.h"
#include "stm32g0xx_hal.h"



typedef struct {
    struct {
        uint16_t Corrective_Angle;   // 校正角度（经过中位校准和滤波处理的角度）- 必须
        int16_t Target_Angle;       // 目标角度（PID控制的目标位置）- 必须
        int16_t Target_Speed;       // 目标转速（PID输出）- 必须
        // 以下字段如需使用可取消注释：
        // int16_t Magnet_Flag;     // 磁铁状态标志（0:丢失, 1:正常, 2:弱）
        // int16_t Raw_Angle;       // 原始角度值（AS5600原始读取值0-4095）
        // int16_t Speed;           // 实际电机转速
        // int32_t Position;        // 电机累计位置（多圈运动）
    } Wings_motor[4];   // 翅膀电机数组：[0]右前, [2]左前（双电机版本只使用这两个）
    
} WINGS_DATA;

extern WINGS_DATA Wings_Data;
extern pid_type_def motor_1_pid, motor_3_pid;
// ====== 【双电机版本】扑动专用PID参数 ======
// ⭐【V6.3优化】针对7.4V锂电池和齿轮间隙重新调参
//
// 供电电压影响分析：
//   - 7.4V锂电池（满电8.4V，标称7.4V，截止6.0V）
//   - 相比5V供电，PWM驱动力矩增强约48%
//   - 需要相应减小PID增益，避免过激响应
//
// 齿轮间隙问题：
//   - 间隙约30-50编码器值
//   - 间隙内"空转"时编码器不变，但PID持续输出
//   - 突然"咬合"时产生冲击 → 抖动
//   - 解决：减小Kp避免过激，增大Kd增加阻尼
//
// V6.3参数调整（相比V6.0）：
//   - Kp: 18→12 (-33%)：避免间隙内过激响应
//   - Ki: 0.08→0.03 (-62%)：避免间隙内积分累积
//   - Kd: 3.0→5.0 (+67%)：增加阻尼，抑制振荡
//
// 调参指南（V6.3版）：
//   跟踪慢/跟不上 → Kp+2 (最大不超过18，否则间隙抖动)
//   有稳态偏差 → Ki+0.01 (最大0.08，否则积分饱和)
//   换向时过冲 → Kd+1 (最大8.0，否则响应变慢)
//   仍有抖动 → Kp-2 或 增大死区阈值
//
static const float motor_pid_params[2][3] = {
    // {Kp,   Ki,    Kd}
    {12.0f, 0.03f, 5.0f},     // 电机0(右前) - V6.3优化版
    {12.0f, 0.03f, 5.0f}      // 电机2(左前) - V6.3优化版
};

// 兼容旧代码的宏定义（双电机版本）
#define MOTOR_1_SPEED_PID_KP motor_pid_params[0][0]
#define MOTOR_1_SPEED_PID_KI motor_pid_params[0][1]
#define MOTOR_1_SPEED_PID_KD motor_pid_params[0][2]
#define MOTOR_3_SPEED_PID_KP motor_pid_params[1][0]
#define MOTOR_3_SPEED_PID_KI motor_pid_params[1][1]
#define MOTOR_3_SPEED_PID_KD motor_pid_params[1][2]

// 1	2
// 
// 4	3





/*-------------Motor_PWM_M1--------------*/
#define PWM_M1_1 	  TIM2->CCR1	 //PWM_M1
#define PWM_M1_2 	  TIM2->CCR2	 //PWM_M1
/*------------------------------------*/

/*-------------Motor_PWM_M2--------------*/
#define PWM_M2_1 	  TIM2->CCR3	 //PWM_M2
#define PWM_M2_2 	  TIM2->CCR4	 //PWM_M2

/*------------------------------------*/

/*-------------Motor_PWM_M3--------------*/
#define PWM_M3_1 	  TIM3->CCR1	 //PWM_M3
#define PWM_M3_2 	  TIM3->CCR2	 //PWM_M3

/*------------------------------------*/

/*-------------Motor_PWM_M4--------------*/
#define PWM_M4_1 	  TIM3->CCR3	 //PWM_M4
#define PWM_M4_2 	  TIM3->CCR4	 //PWM_M4

/*------------------------------------*/

extern void Motor_PID_Control(void);
extern void Chassis_PID_Init(void);
extern void Set_Pwm(int16_t motor1_out, int16_t motor3_out);
extern uint16_t myabs(int16_t a);
extern void Motor_ECD_Control(void);

// 外部变量声明（供调试使用）
extern volatile int motor_1_set_pwm;
extern volatile int motor_3_set_pwm;

// 分支消除的 int16 绝对值（两补码）
static inline uint16_t abs16_fast(int16_t x) {
    int16_t m = x >> 15;            // x<0 → -1；x>=0 → 0
    return (uint16_t)((x ^ m) - m); // 等价于 abs(x)
}
#endif
