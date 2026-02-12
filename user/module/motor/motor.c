#include "motor.h"
/**
************************************************************************************************
* @brief    底盘运动控制算法
* @param    None
* @return   None
* @author   hanchan		2025.12.28
************************************************************************************************
**/
// 外部变量声明
WINGS_DATA Wings_Data;

/************ 运动速度变量定义 *************/

int motor_1_set_pwm;
int motor_2_set_pwm;
int motor_3_set_pwm;
int motor_4_set_pwm;


/************ PID结构体变量定义 *************/
pid_type_def motor_1_pid,motor_2_pid,motor_3_pid,motor_4_pid;
/************** 速度PID限幅定义 *************/
const float motore_max_out=12000,motor_max_iout=4095; //速度环输出限幅 积分输出限幅


/**底盘PID初始化**/
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





/*************** 底盘PID控制 ****************/
void Motor_PID_Control(void)
{
	// 根据硬件接线调整PWM输出映射：
	// 硬件接线：M1=右前, M2=右后, M3=左前, M4=左后
	// 数组索引：[0]=右前, [1]=左后, [2]=左前, [3]=右后
	
	// 电机0（右前）→ M1通道
	Wings_Data.Wings_motor[0].Target_Speed=motor_1_set_pwm=PID_calc(&motor_1_pid, Wings_Data.Wings_motor[0].Corrective_Angle , Wings_Data.Wings_motor[0].Target_Angle );
	
	// 电机3（右后）→ M2通道
	Wings_Data.Wings_motor[3].Target_Speed=motor_4_set_pwm=-PID_calc(&motor_4_pid, Wings_Data.Wings_motor[3].Corrective_Angle , Wings_Data.Wings_motor[3].Target_Angle );
	
	// 电机2（左前）→ M3通道
	Wings_Data.Wings_motor[2].Target_Speed=motor_3_set_pwm=-PID_calc(&motor_3_pid, Wings_Data.Wings_motor[2].Corrective_Angle , Wings_Data.Wings_motor[2].Target_Angle );
	
	// 电机1（左后）→ M4通道
	Wings_Data.Wings_motor[1].Target_Speed=motor_2_set_pwm=PID_calc(&motor_2_pid, Wings_Data.Wings_motor[1].Corrective_Angle , Wings_Data.Wings_motor[1].Target_Angle );
	
	// Set_Pwm参数顺序：M1, M2, M3, M4
	// 对应：右前(motor_1), 右后(motor_4), 左前(motor_3), 左后(motor_2)
	Set_Pwm(motor_1_set_pwm, motor_4_set_pwm, motor_3_set_pwm, motor_2_set_pwm);
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : motor1_out - 电机1输出值, motor2_out - 电机2输出值
          motor3_out - 电机3输出值, motor4_out - 电机4输出值
Output  : none
函数功能：赋值给PWM寄存器，控制四个转速与方向
入口参数：motor1_out - 电机1输出值（来自Wings_Data的PID计算）
          motor2_out - 电机2输出值
          motor3_out - 电机3输出值  
          motor4_out - 电机4输出值
返回  值：无
**************************************************************************/



void Set_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
   // -------- Motor 1 --------
    uint16_t pwm1  = abs16_fast(m1);
    uint16_t mask1 = (uint16_t)-(m1 > 0);      // >0 为 0xFFFF；<=0 为 0x0000
    PWM_M1_2 = (uint16_t)(pwm1 & mask1);       // 反转：CH2=pwm
    PWM_M1_1 = (uint16_t)(pwm1 & ~mask1);      // 正/停：CH1=pwm

    // -------- Motor 2 --------
    uint16_t pwm2  = abs16_fast(m2);
    uint16_t mask2 = (uint16_t)-(m2 > 0);
    PWM_M2_2 = (uint16_t)(pwm2 & mask2);
    PWM_M2_1 = (uint16_t)(pwm2 & ~mask2);

    // -------- Motor 3 --------
    uint16_t pwm3  = abs16_fast(m3);
    uint16_t mask3 = (uint16_t)-(m3 > 0);
    PWM_M3_2 = (uint16_t)(pwm3 & mask3);
    PWM_M3_1 = (uint16_t)(pwm3 & ~mask3);

    // -------- Motor 4 --------
    uint16_t pwm4  = abs16_fast(m4);
    uint16_t mask4 = (uint16_t)-(m4 > 0);
    PWM_M4_2 = (uint16_t)(pwm4 & mask4);
    PWM_M4_1 = (uint16_t)(pwm4 & ~mask4);

}
