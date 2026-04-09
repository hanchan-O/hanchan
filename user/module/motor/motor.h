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
    } Wings_motor[4];   // 翅膀电机数组：[0]右前, [1]左后, [2]左前, [3]右后
    // 注意：索引1是左后，不是左前！与main.c中的电机布局定义保持一致
    
} WINGS_DATA;

extern WINGS_DATA Wings_Data;
extern pid_type_def motor_1_pid,motor_2_pid,motor_3_pid,motor_4_pid;
// ====== 【V6.1优化】扑动专用PID参数 ======
// 基于仿生扑动特性调优，相比V6.0的纯P控制有显著提升：
// - Kp统一为18: 消除前后电机响应不一致导致的俯仰不稳
// - Ki=0.08: 极小积分项，消除齿轮间隙造成的稳态误差
// - Kd=3.0: 抑制极限位置换向超调，保护塑料齿轮箱
//
// 调参指南：
//   跟踪慢/跟不上 → Kp+2 (最大不超过25，否则6Hz会振荡)
//   有稳态偏差(追不到目标) → Ki+0.02 (最大0.15，否则积分饱和)
//   换向时过冲/撞击声 → Kd+1 (最大8.0，否则响应变慢)
static const float motor_pid_params[4][3] = {
    // {Kp,   Ki,    Kd}
    {18.0f, 0.08f, 3.0f},     // 电机0(右前)
    {18.0f, 0.08f, 3.0f},     // 电机1(左后)
    {18.0f, 0.08f, 3.0f},     // 电机2(左前)
    {18.0f, 0.08f, 3.0f}      // 电机3(右后)
};

// 兼容旧代码的宏定义（后续可删除）
#define MOTOR_1_SPEED_PID_KP motor_pid_params[0][0]
#define MOTOR_1_SPEED_PID_KI motor_pid_params[0][1]
#define MOTOR_1_SPEED_PID_KD motor_pid_params[0][2]
#define MOTOR_2_SPEED_PID_KP motor_pid_params[1][0]
#define MOTOR_2_SPEED_PID_KI motor_pid_params[1][1]
#define MOTOR_2_SPEED_PID_KD motor_pid_params[1][2]
#define MOTOR_3_SPEED_PID_KP motor_pid_params[2][0]
#define MOTOR_3_SPEED_PID_KI motor_pid_params[2][1]
#define MOTOR_3_SPEED_PID_KD motor_pid_params[2][2]
#define MOTOR_4_SPEED_PID_KP motor_pid_params[3][0]
#define MOTOR_4_SPEED_PID_KI motor_pid_params[3][1]
#define MOTOR_4_SPEED_PID_KD motor_pid_params[3][2]

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
extern void Set_Pwm(int16_t motor1_out, int16_t motor2_out, int16_t motor3_out, int16_t motor4_out);
extern uint16_t myabs(int16_t a);
extern void Motor_ECD_Control(void);

// 分支消除的 int16 绝对值（两补码）
static inline uint16_t abs16_fast(int16_t x) {
    int16_t m = x >> 15;            // x<0 → -1；x>=0 → 0
    return (uint16_t)((x ^ m) - m); // 等价于 abs(x)
}
#endif
