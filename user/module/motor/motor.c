#include "motor.h"
/**
************************************************************************************************
* @brief    电机 PID 角度与 PWM 控制
* @param    None
* @return   None
* @author   hanchan		2025.12.28
************************************************************************************************
**/
/* 翼型数据 */
WINGS_DATA Wings_Data;

/************ 电机 PWM 输出变量定义 *************/

int motor_1_set_pwm;
int motor_2_set_pwm;
int motor_3_set_pwm;
int motor_4_set_pwm;


/************ 电机速度 PID 参数 *************/
pid_type_def motor_1_pid,motor_2_pid,motor_3_pid,motor_4_pid;
/************** 电机 PID 输出限幅 *************/
const float motore_max_out=12000,motor_max_iout=4095; /* 输出限幅参数 */


/** 电机速度 PID 参数初始化 **/
void Chassis_PID_Init(void) 
{
	const float motor_1_speed_pid[3] = {MOTOR_1_SPEED_PID_KP, MOTOR_1_SPEED_PID_KI, MOTOR_1_SPEED_PID_KD};
	const float motor_2_speed_pid[3] = {MOTOR_2_SPEED_PID_KP, MOTOR_2_SPEED_PID_KI, MOTOR_2_SPEED_PID_KD};
	const float motor_3_speed_pid[3] = {MOTOR_3_SPEED_PID_KP, MOTOR_3_SPEED_PID_KI, MOTOR_3_SPEED_PID_KD};
	const float motor_4_speed_pid[3] = {MOTOR_4_SPEED_PID_KP, MOTOR_4_SPEED_PID_KI, MOTOR_4_SPEED_PID_KD};
	PID_init(&motor_1_pid, PID_POSITION, motor_1_speed_pid, motore_max_out, motor_max_iout);
	PID_init(&motor_2_pid, PID_POSITION, motor_2_speed_pid, motore_max_out, motor_max_iout);
	PID_init(&motor_3_pid, PID_POSITION, motor_3_speed_pid, motore_max_out, motor_max_iout);
	PID_init(&motor_4_pid, PID_POSITION, motor_4_speed_pid, motore_max_out, motor_max_iout);
}



/* ---------- 扑翼延迟限幅处理（防止突变） ---------- */
#define FLAP_SLEW_PER_MS  140  /* 最大每毫秒变化角度值 */

static int16_t flap_slew_pos[4];
static uint32_t flap_slew_last_ms;

void Motor_Flap_Slew_Reset(void)
{
	for (uint8_t i = 0; i < 4; i++)
		flap_slew_pos[i] = (int16_t)Wings_Data.Wings_motor[i].Corrective_Angle;
	flap_slew_last_ms = HAL_GetTick();
}

static void Motor_Flap_Slew_Update(void)
{
	uint32_t now = HAL_GetTick();
	uint32_t dt = now - flap_slew_last_ms;
	if (dt == 0)
		return;
	if (dt > 20U)
		dt = 20U;
	flap_slew_last_ms = now;

	int32_t max_step = (int32_t)FLAP_SLEW_PER_MS * (int32_t)dt;
	if (max_step < 1)
		max_step = 1;
	if (max_step > 8000)
		max_step = 8000;

	for (uint8_t i = 0; i < 4; i++)
	{
		int16_t cmd = Wings_Data.Wings_motor[i].Target_Angle;
		int32_t delta = (int32_t)cmd - (int32_t)flap_slew_pos[i];
		if (delta > max_step)
			flap_slew_pos[i] += (int16_t)max_step;
		else if (delta < -max_step)
			flap_slew_pos[i] -= (int16_t)max_step;
		else
			flap_slew_pos[i] = cmd;
	}
}

void Motor_PID_Control_Flap(void)
{
	Motor_Flap_Slew_Update();

	Wings_Data.Wings_motor[0].Target_Speed = motor_1_set_pwm =
	    -PID_calc(&motor_1_pid, Wings_Data.Wings_motor[0].Corrective_Angle, flap_slew_pos[0]);

	Wings_Data.Wings_motor[3].Target_Speed = motor_4_set_pwm =
	    -PID_calc(&motor_4_pid, Wings_Data.Wings_motor[3].Corrective_Angle, flap_slew_pos[3]);

	Wings_Data.Wings_motor[2].Target_Speed = motor_3_set_pwm =
	    PID_calc(&motor_3_pid, Wings_Data.Wings_motor[2].Corrective_Angle, flap_slew_pos[2]);

	Wings_Data.Wings_motor[1].Target_Speed = motor_2_set_pwm =
	    PID_calc(&motor_2_pid, Wings_Data.Wings_motor[1].Corrective_Angle, flap_slew_pos[1]);

	Set_Pwm(motor_1_set_pwm, motor_4_set_pwm, motor_3_set_pwm, motor_2_set_pwm);
}

/*************** 电机 PID 控制主函数 ****************/
void Motor_PID_Control(void)
{
	/* 上方为 PWM 输出对应 M1~M4 的 Set_Pwm */
	/* Wings_motor 序号 [0]..[3] 对应 PCB 丝印序号 */
	
	/* 通道 0 的 PID 计算 -> M1 */
	Wings_Data.Wings_motor[0].Target_Speed=motor_1_set_pwm=-PID_calc(&motor_1_pid, Wings_Data.Wings_motor[0].Corrective_Angle , Wings_Data.Wings_motor[0].Target_Angle );
	
	/* 通道 3 的 PID 计算 -> M2 */
	Wings_Data.Wings_motor[3].Target_Speed=motor_4_set_pwm=-PID_calc(&motor_4_pid, Wings_Data.Wings_motor[3].Corrective_Angle , Wings_Data.Wings_motor[3].Target_Angle );
	
	/* 通道 2 的 PID 计算 -> M3 */
	Wings_Data.Wings_motor[2].Target_Speed=motor_3_set_pwm=PID_calc(&motor_3_pid, Wings_Data.Wings_motor[2].Corrective_Angle , Wings_Data.Wings_motor[2].Target_Angle );
	
	/* 通道 1 的 PID 计算 -> M4 */
	Wings_Data.Wings_motor[1].Target_Speed=motor_2_set_pwm=PID_calc(&motor_2_pid, Wings_Data.Wings_motor[1].Corrective_Angle , Wings_Data.Wings_motor[1].Target_Angle );
	
	/* Set_Pwm( M1, M2, M3, M4 )顺序对应 motor_1, motor_4, motor_3, motor_2 */
	Set_Pwm(motor_1_set_pwm, motor_4_set_pwm, motor_3_set_pwm, motor_2_set_pwm);
}

/**************************************************************************
Function: ??? PID ?????? PWM ??
Input   : motor1_out..motor4_out ???? PID ???????
Output  : none
**************************************************************************/



void Set_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
#if !MOTOR_HW_USE_PWM_TIM2_CH12
	m1 = 0;
#endif
#if !MOTOR_HW_USE_PWM_TIM3_CH34
	m4 = 0;
#endif
   /* -------- Motor 1 -------- */
    uint16_t pwm1  = abs16_fast(m1);
    uint16_t mask1 = (uint16_t)-(m1 > 0);      /* >0 时输出 0xFFFF，<=0 时输出 0 */
    PWM_M1_2 = (uint16_t)(pwm1 & mask1);       /* 反向通道CH2=pwm */
    PWM_M1_1 = (uint16_t)(pwm1 & ~mask1);      /* 正向通道CH1=pwm */

    /* -------- Motor 2 -------- */
    uint16_t pwm2  = abs16_fast(m2);
    uint16_t mask2 = (uint16_t)-(m2 > 0);
    PWM_M2_2 = (uint16_t)(pwm2 & mask2);
    PWM_M2_1 = (uint16_t)(pwm2 & ~mask2);

    /* -------- Motor 3 -------- */
    uint16_t pwm3  = abs16_fast(m3);
    uint16_t mask3 = (uint16_t)-(m3 > 0);
    PWM_M3_2 = (uint16_t)(pwm3 & mask3);
    PWM_M3_1 = (uint16_t)(pwm3 & ~mask3);

    /* -------- Motor 4 -------- */
    uint16_t pwm4  = abs16_fast(m4);
    uint16_t mask4 = (uint16_t)-(m4 > 0);
    PWM_M4_2 = (uint16_t)(pwm4 & mask4);
    PWM_M4_1 = (uint16_t)(pwm4 & ~mask4);

}
