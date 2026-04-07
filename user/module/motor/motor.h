#ifndef __MOTOR_H
#define __MOTOR_H

#include "motor_hw_config.h"

#include "pid.h"
#include "main.h"
#include "AS5600_PWM.h"
#include "stm32g0xx_hal.h"



typedef struct {
    struct {
        uint16_t Corrective_Angle;   /* ????????? */
        int16_t Target_Angle;        /* ??? PID ??? */
        int16_t Target_Speed;        /* ??? PID ???PWM ?? */
        /* ????????? */
        /* int16_t Magnet_Flag;     ???0 ?? 1 ?? 2 ?? */
        /* int16_t Raw_Angle;       ?? 0-4095 */
        /* int16_t Speed;           ??? */
        /* int32_t Position;        ???? */
    } Wings_motor[4];   /* [0]..[3] ?????? */

} WINGS_DATA;

extern WINGS_DATA Wings_Data;
extern pid_type_def motor_1_pid,motor_2_pid,motor_3_pid,motor_4_pid;
/* PID ????static const????????? */
static const float motor_pid_params[4][3] = {
    /* {Kp, Ki, Kd} */
    {20.0f, 0.0f, 0.0f},  /* ?? 0 */
    {15.0f, 0.0f, 0.0f},  /* ?? 1 */
    {20.0f, 0.0f, 0.0f},  /* ?? 2 */
    {15.0f, 0.0f, 0.0f}   /* ?? 3 */
};

/* ??????? PID ?? */
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
/* ?????Target_Angle ??????????? PID */
extern void Motor_PID_Control_Flap(void);
/* ?????? Target_Angle ? slew ????? PID */
extern void Motor_Flap_Slew_Reset(void);
extern void Chassis_PID_Init(void);
extern void Set_Pwm(int16_t motor1_out, int16_t motor2_out, int16_t motor3_out, int16_t motor4_out);
extern uint16_t myabs(int16_t a);
extern void Motor_ECD_Control(void);

/* int16 ????? */
static inline uint16_t abs16_fast(int16_t x) {
    int16_t m = x >> 15;            /* x<0 ? -1?x>=0 ? 0 */
    return (uint16_t)((x ^ m) - m); /* ?? abs(x) */
}
#endif
