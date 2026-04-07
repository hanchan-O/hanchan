#ifndef __MOTOR_H
#define __MOTOR_H

#include "motor_hw_config.h"

#include "pid.h"
#include "main.h"
#include "AS5600_PWM.h"
#include "stm32g0xx_hal.h"



typedef struct {
    struct {
        uint16_t Corrective_Angle;   /* 编码器校准角度 */
        int16_t Target_Angle;        /* 目标 PID 输入 */
        int16_t Target_Speed;        /* 目标 PID 输出PWM 值 */
        /* 以下为预留字段 */
        /* int16_t Magnet_Flag;     磁铁状态0 无 1 有 2 强 */
        /* int16_t Raw_Angle;       原始 0-4095 */
        /* int16_t Speed;           原始速度 */
        /* int32_t Position;        累计位置 */
    } Wings_motor[4];   /* [0]..[3] 对应电机序号 */

} WINGS_DATA;

extern WINGS_DATA Wings_Data;
extern pid_type_def motor_1_pid,motor_2_pid,motor_3_pid,motor_4_pid;
/* PID 参数使用static const定义便于调试 */
static const float motor_pid_params[4][3] = {
    /* {Kp, Ki, Kd} */
    {20.0f, 0.0f, 0.0f},  /* 电机 0 */
    {15.0f, 0.0f, 0.0f},  /* 电机 1 */
    {20.0f, 0.0f, 0.0f},  /* 电机 2 */
    {15.0f, 0.0f, 0.0f}   /* 电机 3 */
};

/* 电机 PWM 参数配置宏定义 */
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

/* ???? PWM ???????? */


/*-------------Motor_PWM_M1--------------*/
#define PWM_M1_1 	  TIM2->CCR1	 /* PWM_M1 */
#define PWM_M1_2 	  TIM2->CCR2	 /* PWM_M1 */
/*------------------------------------*/

/*-------------Motor_PWM_M2--------------*/
#define PWM_M2_1 	  TIM2->CCR3	 /* PWM_M2 */
#define PWM_M2_2 	  TIM2->CCR4	 /* PWM_M2 */

/*------------------------------------*/

/*-------------Motor_PWM_M3--------------*/
#define PWM_M3_1 	  TIM3->CCR1	 /* PWM_M3 */
#define PWM_M3_2 	  TIM3->CCR2	 /* PWM_M3 */

/*------------------------------------*/

/*-------------Motor_PWM_M4--------------*/
#define PWM_M4_1 	  TIM3->CCR3	 /* PWM_M4 */
#define PWM_M4_2 	  TIM3->CCR4	 /* PWM_M4 */

/*------------------------------------*/

extern void Motor_PID_Control(void);
/* 扑翼模式：Target_Angle 直接控制并输出修正后的 PID */
extern void Motor_PID_Control_Flap(void);
/* 扑翼模式：Target_Angle 加上 slew 限幅后再输出 PID */
extern void Motor_Flap_Slew_Reset(void);
extern void Chassis_PID_Init(void);
extern void Set_Pwm(int16_t motor1_out, int16_t motor2_out, int16_t motor3_out, int16_t motor4_out);
extern uint16_t myabs(int16_t a);
extern void Motor_ECD_Control(void);

/* int16 快速绝对值函数 */
static inline uint16_t abs16_fast(int16_t x) {
    int16_t m = x >> 15;            /* x<0 时返回 -1，否则返回 0 */
    return (uint16_t)((x ^ m) - m); /* 计算 abs(x) */
}
#endif
