/**
 * @file    motor.c
 * @brief   电机控制模块（PID闭环 + PWM驱动）
 * @author  hanchan
 * @date    2026-04-10
 *
 * ## 模块功能
 * 本模块负责4个电机的PID控制和PWM输出，是连接上层控制算法
 * 和底层硬件驱动的桥梁。
 *
 * ## 信号链路
 * Target_Angle（目标角度）→ PID计算 → PWM输出 → 电机转动
 *     ↑                                                      ↓
 *     └───────── Corrective_Angle（实际角度）← 磁编码器 ←──┘
 *
 * ## 关键特性
 * - 4路独立PID控制器（位置式PID）
 * - H桥PWM驱动（支持正反转）
 * - 输出限幅保护电机和驱动器
 */

#include "motor.h"

// ==================== 全局变量定义 ====================

/**
 * @brief 机翼数据结构体实例
 * 
 * 存储所有电机的状态信息：
 * - Target_Angle: 目标角度（由flight_control模块设置）
 * - Corrective_Angle: 实际角度（由AS5600_PWM模块更新）
 * - Target_Speed: 目标速度/PID输出值
 */
WINGS_DATA Wings_Data;

/**
 * @brief 各电机的PWM设定值（调试用）
 * 
 * 存储PID计算结果，可在调试时观察各电机的输出情况
 */
int motor_1_set_pwm;   // 右前电机PWM值
int motor_2_set_pwm;   // 左后电机PWM值（注意：变量名与索引不对应！）
int motor_3_set_pwm;   // 左前电机PWM值
int motor_4_set_pwm;   // 右后电机PWM值

/**
 * @brief 4个电机的PID结构体实例
 * 
 * 每个电机有独立的PID参数，可以单独调整以适应机械差异
 * - motor_1_pid: 控制电机0（右前）
 * - motor_2_pid: 控制电机1（左后）
 * - motor_3_pid: 控制电机2（左前）
 * - motor_4_pid: 控制电机3（右后）
 */
pid_type_def motor_1_pid, motor_2_pid, motor_3_pid, motor_4_pid;

// ==================== 【重要】可调参数配置区 ====================
// 🔧 调试时主要修改这里的参数！

/**
 * @brief PID输出限幅值
 * 
 * 作用：限制PID计算结果的最大绝对值，防止输出过大损坏驱动器或电机
 * 
 * 可调范围：8000 ~ 15000
 * 当前值：12000（推荐值）
 * 建议：
 *   - 电机响应不够快 → 可以尝试增大到14000-15000
 *   - 电机抖动或过冲 → 减小到10000或更低
 *   - 一般保持12000即可
 * 
 * ⚠️ 注意：这个值与定时器的PWM分辨率相关
 * 如果使用16位定时器（0-65535），12000约占18%占空比
 * 如果使用自动重载值为12000的定时器，则对应100%占空比
 */
const float motore_max_out = 12000;

/**
 * @brief 积分项限幅值
 * 
 * 作用：限制PID积分项的累积值，防止积分饱和（integral windup）
 * 当误差持续存在时，积分项会不断增大，如果不加限制，
 * 即使误差反向也需要很长时间才能消除积分影响
 * 
 * 可调范围：2048 ~ 8192
 * 当前值：4095（推荐值）
 * 建议：
 *   - 一般不需要调整
 *   - 如果出现明显的超调或振荡，可以减小到2048
 *   - 如果系统响应太慢且无超调，可以增大到6000
 */
const float motor_max_iout = 4095;

/**
************************************************************************************************
* @brief    初始化4个电机的PID控制器
*
* ## 功能
* 在系统启动时调用一次，初始化每个电机的PID参数。
* 从motor.h头文件中读取预设的Kp、Ki、Kd值，
* 并配置输出限幅和积分限幅。
*
* ## PID参数来源
* Kp/Ki/Kd的默认值定义在motor.h中的宏：
* - MOTOR_1_SPEED_PID_KP/KI/KD：电机0（右前）的PID参数
* - MOTOR_2_SPEED_PID_KP/KI/KD：电机1（左后）的PID参数
* - MOTOR_3_SPEED_PID_KP/KI/KD：电机2（左前）的PID参数
* - MOTOR_4_SPEED_PID_KP/KI/KD：电机3（右后）的PID参数
*
* ## 调用时机
* 在main.c的系统初始化阶段调用（通常在HAL_Init()之后）
************************************************************************************************
**/
void Chassis_PID_Init(void) 
{
	// 从头文件读取PID参数数组 [Kp, Ki, Kd]
	const float motor_1_speed_pid[3] = {MOTOR_1_SPEED_PID_KP, MOTOR_1_SPEED_PID_KI, MOTOR_1_SPEED_PID_KD};
	const float motor_2_speed_pid[3] = {MOTOR_2_SPEED_PID_KP, MOTOR_2_SPEED_PID_KI, MOTOR_2_SPEED_PID_KD};
	const float motor_3_speed_pid[3] = {MOTOR_3_SPEED_PID_KP, MOTOR_3_SPEED_PID_KI, MOTOR_3_SPEED_PID_KD};
	const float motor_4_speed_pid[3] = {MOTOR_4_SPEED_PID_KP, MOTOR_4_SPEED_PID_KI, MOTOR_4_SPEED_PID_KD};

	// 初始化4个PID控制器
	// 参数：PID结构体指针, PID模式(位置式), PID参数数组, 输出上限, 积分上限
	PID_init(&motor_1_pid, PID_POSITION, motor_1_speed_pid, motore_max_out, motor_max_iout);
	PID_init(&motor_2_pid, PID_POSITION, motor_2_speed_pid, motore_max_out, motor_max_iout);
	PID_init(&motor_3_pid, PID_POSITION, motor_3_speed_pid, motore_max_out, motor_max_iout);
	PID_init(&motor_4_pid, PID_POSITION, motor_4_speed_pid, motore_max_out, motor_max_iout);
}

/**
************************************************************************************************
* @brief    执行4路电机PID控制 ⭐核心函数
*
* ## 功能
* 这是本模块的主函数，由主循环高频调用（每次循环都调用）。
* 对4个电机分别执行以下操作：
* 1. 计算PID误差：error = Target - Current
* 2. 执行PID算法：output = Kp×error + Ki×∫error + Kd×Δerror
* 3. 保存输出值到全局变量（供调试观察）
* 4. 调用Set_Pwm()输出PWM信号
*
* ## 电机映射关系（重要！）
* 
* 数组索引 vs 物理位置 vs PWM通道：
* ┌─────────────────────────────────────────────────────┐
* │ 数组索引 │ 物理位置 │ PID变量名 │ PWM通道 │ 备注    │
* ├──────────┼──────────┼───────────┼─────────┼─────────┤
* │ [0]      │ 右前     │ motor_1  │ M1      │         │
* │ [1]      │ 左后     │ motor_2  │ M4      │ 取反！  │
* │ [2]      │ 左前     │ motor_3  │ M3      │         │
* │ [3]      │ 右后     │ motor_4  │ M2      │         │
* └─────────────────────────────────────────────────────┘
*
* ⚠️ 特别注意：
* - 电机1（左后）的PID输出取负号（-PID_calc），因为安装方向相反
* - Set_Pwm()的参数顺序是(M1,M2,M3,M4)，不是按数组索引顺序！
*
* ## PID公式（位置式）
* output = Kp × error(t) + Ki × Σerror(0..t) + Kd × (error(t) - error(t-1))
*
* 其中：
* - error = Target_Angle - Corrective_Angle（目标角度 - 实际角度）
* - output范围：[-motore_max_out, +motore_max_out]
* - 正输出=正转，负输出=反转
************************************************************************************************
**/
void Motor_PID_Control(void)
{
	// ====== 电机0（右前）→ M1通道 ======
	// PID_calc参数：(PID结构体, 反馈值/实际值, 设定值/目标值)
	// 返回值：PID输出（正=需要正向旋转，负=需要反向旋转）
	Wings_Data.Wings_motor[0].Target_Speed = 
	    motor_1_set_pwm = 
	        -PID_calc(&motor_1_pid, 
	                  Wings_Data.Wings_motor[0].Corrective_Angle,   // 实际角度（反馈）
	                  Wings_Data.Wings_motor[0].Target_Angle);       // 目标角度（设定）

	// ====== 电机3（右后）→ M2通道 ======
	Wings_Data.Wings_motor[3].Target_Speed = 
	    motor_4_set_pwm = 
	        PID_calc(&motor_4_pid, 
	                 Wings_Data.Wings_motor[3].Corrective_Angle, 
	                 Wings_Data.Wings_motor[3].Target_Angle);

	// ====== 电机2（左前）→ M3通道 ======
	Wings_Data.Wings_motor[2].Target_Speed = 
	    motor_3_set_pwm = 
	        PID_calc(&motor_3_pid, 
	                 Wings_Data.Wings_motor[2].Corrective_Angle, 
	                 Wings_Data.Wings_motor[2].Target_Angle);

	// ====== 电机1（左后）→ M4通道（⚠️取反！）=====
	// 这个电机的安装方向与其他相反，所以PID输出要取负
	// 原因：当其他电机"正向旋转"时，这个电机需要"反向旋转"
	//       才能达到相同的物理效果（翅膀向同一方向运动）
	Wings_Data.Wings_motor[1].Target_Speed = 
	    motor_2_set_pwm = 
	        -PID_calc(&motor_2_pid,                  // 注意前面的负号！
	                  Wings_Data.Wings_motor[1].Corrective_Angle, 
	                  Wings_Data.Wings_motor[1].Target_Angle);

	// ====== 输出PWM信号到4个通道 ======
	// Set_Pwm参数顺序：M1(右前), M2(右后), M3(左前), M4(左后)
	Set_Pwm(motor_1_set_pwm, motor_4_set_pwm, motor_3_set_pwm, motor_2_set_pwm);
}

/**
************************************************************************************************
* @brief    设置4路电机PWM输出（H桥驱动）
*
* ## 功能
* 将PID计算得到的控制量转换为实际的PWM波形输出到电机驱动器。
* 使用H桥驱动方式，每个电机需要2个PWM通道来控制方向：
* - 正转：CH1输出PWM，CH2输出0
* - 反转：CH1输出0，CH2输出PWM
* - 停止：CH1和CH2都输出0
*
* ## 驱动原理（H桥）
* 每个电机由一个H桥电路驱动，有4个开关管（Q1-Q4）：
*        VCC
*         │
*    ┌────┴────┐
*    Q1       Q2
*    │         │
*  MOTOR+  MOTOR-
*    │         │
*    Q3       Q4
*    │         │
*    └────┬────┘
*        GND
*
* 正转：Q1,Q4导通 → 电流从MOTOR+流向MOTOR-
* 反转：Q2,Q3导通 → 电流从MOTOR-流向MOTOR+
* 制动：Q1,Q2导通 或 Q3,Q4导通 → 电机短路制动
*
* ## PWM实现方式
* 本代码使用互补PWM模式：
* - mask = (input > 0) ? 0xFFFF : 0x0000  （符号掩码）
* - CH1 = pwm & ~mask  （正向通道）
* - CH2 = pwm & mask   （反向通道）
* 这样当input>0时，CH2=pwm, CH1=0（反转）
*     当input≤0时，CH1=pwm, CH2=0（正转或停止）
*
* ## 参数说明
* @param m1: 电机1（右前）的PWM值（来自motor_1_set_pwm）
* @param m2: 电机2（右后）的PWM值（来自motor_4_set_pwm）
* @param m3: 电机3（左前）的PWM值（来自motor_3_set_pwm）
* @param m4: 电机4（左后）的PWM值（来自motor_2_set_pwm）
*
* 数值含义：
* - 正数：一个方向旋转（具体方向取决于接线）
* - 负数：相反方向旋转
* - 绝对值越大，转速越快
* - 0：停止（但可能有保持力矩，取决于驱动器类型）
************************************************************************************************
**/
void Set_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	// ====== Motor 1（右前） ======
	uint16_t pwm1  = abs16_fast(m1);           // 取绝对值作为PWM占空比
	uint16_t mask1 = (uint16_t)-(m1 > 0);     // 符号判断：>0则mask=0xFFFF，否则=0x0000
	PWM_M1_2 = (uint16_t)(pwm1 & mask1);      // 反转通道：m1>0时有输出
	PWM_M1_1 = (uint16_t)(pwm1 & ~mask1);     // 正转通道：m1≤0时有输出

	// ====== Motor 2（右后） ======
	uint16_t pwm2  = abs16_fast(m2);
	uint16_t mask2 = (uint16_t)-(m2 > 0);
	PWM_M2_2 = (uint16_t)(pwm2 & mask2);
	PWM_M2_1 = (uint16_t)(pwm2 & ~mask2);

	// ====== Motor 3（左前） ======
	uint16_t pwm3  = abs16_fast(m3);
	uint16_t mask3 = (uint16_t)-(m3 > 0);
	PWM_M3_2 = (uint16_t)(pwm3 & mask3);
	PWM_M3_1 = (uint16_t)(pwm3 & ~mask3);

	// ====== Motor 4（左后） ======
	uint16_t pwm4  = abs16_fast(m4);
	uint16_t mask4 = (uint16_t)-(m4 > 0);
	PWM_M4_2 = (uint16_t)(pwm4 & mask4);
	PWM_M4_1 = (uint16_t)(pwm4 & ~mask4);
}
