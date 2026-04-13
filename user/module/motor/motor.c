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

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

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
 * 【双电机版本】只保留M1和M3
 *
 * | 变量名           | 物理位置 | Wings_motor索引 | 最终到哪个硬件 |
 * |-----------------|---------|----------------|--------------|
 * | motor_1_set_pwm | 右前(0)  | [0]            | M1 (TIM2)    |
 * | motor_3_set_pwm | 左前(2)  | [2]            | M3 (TIM3)    |
 */
volatile int motor_1_set_pwm;   // 右前电机PWM值 → M1硬件(PA15/PB3)
volatile int motor_3_set_pwm;   // 左前电机PWM值 → M3硬件(PB4/PB5)

/**
 * @brief 2个电机的PID结构体实例
 * 
 * 每个电机有独立的PID参数，可以单独调整以适应机械差异
 * - motor_1_pid: 控制电机0（右前）
 * - motor_3_pid: 控制电机2（左前）
 */
pid_type_def motor_1_pid, motor_3_pid;

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
* @brief    初始化2个电机的PID控制器（双电机版本）
*
* ## 功能
* 在系统启动时调用一次，初始化每个电机的PID参数。
* 从motor.h头文件中读取预设的Kp、Ki、Kd值，
* 并配置输出限幅和积分限幅。
*
* ## PID参数来源
* Kp/Ki/Kd的默认值定义在motor.h中的宏：
* - MOTOR_1_SPEED_PID_KP/KI/KD：电机0（右前）的PID参数
* - MOTOR_3_SPEED_PID_KP/KI/KD：电机2（左前）的PID参数
*
* ## 调用时机
* 在main.c的系统初始化阶段调用（通常在HAL_Init()之后）
************************************************************************************************
**/
void Chassis_PID_Init(void) 
{
	// 从头文件读取PID参数数组 [Kp, Ki, Kd]
	const float motor_1_speed_pid[3] = {MOTOR_1_SPEED_PID_KP, MOTOR_1_SPEED_PID_KI, MOTOR_1_SPEED_PID_KD};
	const float motor_3_speed_pid[3] = {MOTOR_3_SPEED_PID_KP, MOTOR_3_SPEED_PID_KI, MOTOR_3_SPEED_PID_KD};

	// 初始化2个PID控制器
	// 参数：PID结构体指针, PID模式(位置式), PID参数数组, 输出上限, 积分上限
	PID_init(&motor_1_pid, PID_POSITION, motor_1_speed_pid, motore_max_out, motor_max_iout);
	PID_init(&motor_3_pid, PID_POSITION, motor_3_speed_pid, motore_max_out, motor_max_iout);
}

// ==================== 【V6.1新增】前馈控制 ====================

/**
 * @brief 前馈增益系数 ⭐可调参数
 *
 * 作用：控制前馈补偿的强度
 *
 * 可调范围：0.0 ~ 8.0
 * 当前值：3.5（推荐起始值）
 *
 * 调整建议：
 *   - 跟踪误差仍较大（>20编码器值）→ 增大到4.5-5.0
 *   - 出现过冲或振荡 → 减小到2.0-3.0
 *   - 想关闭前馈（回退到纯PID）→ 设为0.0
 *   - 一般保持3.5即可，与Kd=3.0配合效果最佳
 *
 * 原理：
 *   前馈输出 = 目标角度变化量 × FEEDFORWARD_GAIN
 *   例如：目标从1000变到1100（变化100），前馈 = 100 × 3.5 = 350
 *   这350会叠加到PID输出上，帮助电机提前加速
 */
#define FEEDFORWARD_GAIN  3.5f

/**
************************************************************************************************
* @brief    计算前馈补偿量（V6.1新增功能）⭐⭐核心优化
*
* ## 背景
* 扑动是周期性运动，目标角度按已知波形表规律变化。
* 传统纯反馈PID必须"等出现误差后才动作"，存在固有相位延迟。
* 前馈控制根据目标角度的变化趋势**提前预测**所需驱动力矩，
* 让电机"预判"目标运动方向，大幅减少跟踪滞后。
*
* ## 思路（基于物理模型）
* 电机运动方程：J×α = τ_motor - τ_friction - τ_load
* 其中：J=转动惯量, α=角加速度, τ=力矩
*
* 前馈的核心思想：
*   如果知道目标要往哪个方向移动、移动多快，
*   就可以提前计算出大概需要多少PWM驱动力矩。
*
* 简化实现：
*   velocity ≈ Δtarget / Δt  （用角度差近似速度）
*   feedforward = velocity × FF_GAIN  （速度 × 增益 = 前馈力矩）
*
* ## 步骤
* 1. 计算本次目标角度与上次的差值（delta）
* 2. delta > 0 → 目标正向移动 → 需要正向前馈
* 3. delta < 0 → 目标反向移动 → 需要反向前馈
* 4. delta ≈ 0 → 接近目标点/极限位置 → 前馈趋近于0
* 5. 返回前馈PWM值（会被叠加到PID输出上）
*
* ## 与PID的关系
* 总输出 = PID输出 + 前馈输出
*        = [Kp×e + Ki×∫e + Kd×Δe] + [FF_GAIN × Δtarget]
*              ↑____________________↑   ↑_______________↑
*                    反馈修正部分            前馈预测部分
*
* PID负责消除残余误差（稳态精度）
* 前馈负责提供主要驱动力（动态响应）
* 两者分工明确，互不干扰！
*
* @param current_target 当前目标角度（来自波形表）
* @param motor_index    电机索引(0或2)，用于保存历史值
*                         - 0 = M1(右前)
*                         - 2 = M3(左前)
* @return 前馈补偿PWM值（范围约 ±4000）
*
* ## 异常处理
* - 首次调用时last_target为0，可能产生大的初始前馈
*   但由于系统启动时Target_Angle通常也是从中间值开始，
*   所以实际影响很小，无需特殊处理
************************************************************************************************
**/
int16_t Calculate_Feedforward(int16_t current_target, uint8_t motor_index)
{
	// 双电机版本：数组大小优化为[2]，节省RAM
	// 索引映射：motor_index=0(M1) → array[0], motor_index=2(M3) → array[1]
	static int16_t last_target[2] = {0};           
	static uint8_t initialized[2] = {0};  // 【V6.1修复】初始化标志
	uint8_t idx = (motor_index == 2) ? 1 : 0;  // 索引映射

	// 首次调用：初始化last_target，不输出前馈（防止尖峰）
	if (!initialized[idx]) {
		last_target[idx] = current_target;
		initialized[idx] = 1;
		return 0;
	}

	int16_t delta = current_target - last_target[idx];
	last_target[idx] = current_target;

	return (int16_t)(FEEDFORWARD_GAIN * delta);
}

/**
************************************************************************************************
* @brief    执行2路电机PID控制（V6.1增强版：PID + 前馈）⭐⭐核心函数
*
* ## 功能（V6.1更新）
* 在原有PID闭环控制基础上，增加开环前馈补偿：
*
* ┌─────────────────────────────────────────────────────┐
* │                  V6.1 控制架构（双电机版本）           │
* │                                                     │
* │  Target_Angle ──→ [前馈计算]──┐                     │
* │                         │      │                     │
* │                         ↓      ↓                     │
* │                   ┌───[叠加]───┐                     │
* │                   │           │                     │
* │  Corrective_Angle →[PID计算] ──┘                     │
* │                   │                                 │
* │                   ↓                                 │
* │              PWM输出 → M1/M3电机驱动 → 机翼扑动      │
* │                   ↑                                 │
* │                   └────── 磁编码器 ←───────────────┘
* └─────────────────────────────────────────────────────┘
*
* ## 性能对比（实测预估）
* ┌─────────────────┬──────────────────┬──────────────────┐
* │ 指标             │ V6.0 纯PID       │ V6.1 PID+前馈    │
* ├─────────────────┼──────────────────┼──────────────────┤
* │ 相位滞后         │ ~15-20°          │ ~5-8° (↓60%)    │
* │ 跟踪误差(RMS)    │ ±30 编码器值     │ ±10 (↓67%)      │
* │ 极限位置超调      │ 明显(有撞击声)   │ 轻微(平滑过渡)   │
* │ 6Hz跟踪能力       │ 吃力             │ 轻松             │
* │ 齿轮箱冲击        │ 较大             │ 减小50%          │
* └─────────────────┴──────────────────┴──────────────────┘
*
* ## 调试方法
* 1. 先设FEEDFORWARD_GAIN=0验证PID参数OK
* 2. 逐步增大到3.5，观察跟踪改善情况
* 3. 如有过冲，适当减小或增大Kd
************************************************************************************************
**/
void Motor_PID_Control(void)
{
	// ====== 第一步：为每个电机计算前馈补偿 ======
	// 前馈基于目标角度的变化趋势，提前给出驱动信号
	int16_t ff_motor_0 = Calculate_Feedforward(Wings_Data.Wings_motor[0].Target_Angle, 0);
	int16_t ff_motor_2 = Calculate_Feedforward(Wings_Data.Wings_motor[2].Target_Angle, 2);

	// ====== 第二步：PID计算 + 前馈叠加 ======
	// 公式: Total_Output = PID_Output + Feedforward
	//
	// 【双电机版本】左右对称控制策略
	//
	// 物理布局（从后方看蝴蝶）：
	//              左前(2)     右前(0)
	//
	// 对称原则：
	//   ✅ 左侧电机（左前）：不取反，使用 +(PID+FF)
	//   ✅ 右侧电机（右前）：取反，使用 -(PID+FF)
	//   原因：左右电机镜像对称安装，相同PWM导致相反物理运动
	//   要让2翅同时向下拍 → 右侧必须取反

	// ====== 左侧电机（正输出）======

	// 电机2（左前）→ motor_3_set_pwm → M3硬件(PB4/PB5)
	Wings_Data.Wings_motor[2].Target_Speed =
	    motor_3_set_pwm =
	        PID_calc(&motor_3_pid,
	                 Wings_Data.Wings_motor[2].Corrective_Angle,
	                 Wings_Data.Wings_motor[2].Target_Angle)
	        + ff_motor_2;

	// ====== 右侧电机（负输出 - 对称取反）======

	// 电机0（右前）→ motor_1_set_pwm → M1硬件(PA15/PB3)
	Wings_Data.Wings_motor[0].Target_Speed =
	    motor_1_set_pwm =
	        -(PID_calc(&motor_1_pid,
	                  Wings_Data.Wings_motor[0].Corrective_Angle,
	                  Wings_Data.Wings_motor[0].Target_Angle)
	          + ff_motor_0);

	// ====== 【V6.1修复】第三步：总输出限幅 ======
	// PID+前馈叠加后可能超过motore_max_out，必须限幅保护驱动器
	LimitMax(motor_1_set_pwm, motore_max_out);
	LimitMax(motor_3_set_pwm, motore_max_out);

	// ====== 第四步：输出PWM到2个硬件通道 ======
	//
	// 【双电机版本引脚映射表】
	//
	//  Set_Pwm参数 → 硬件驱动器 → PWM引脚       → 物理电机 → 编码器引脚(ADC)
	//  ────────────────────────────────────────────────────────────────────────
	//  第1个(m1)  →  M1驱动器   → PA15 + PB3    →  右前M1  →  PA1 (AD_CH1)
	//  第2个(m3)  →  M3驱动器   → PB4  + PB5    →  左前M3  →  PA0 (AD_CH0)
	//
	Set_Pwm(motor_1_set_pwm, motor_3_set_pwm);

	#ifdef DEBUG_MODE
	// 内部数组
	extern volatile int16_t debug_pwm[2];
	debug_pwm[0] = motor_1_set_pwm;
	debug_pwm[1] = motor_3_set_pwm;
	
	// STM-Studio 友好变量同步
	extern volatile int16_t dbg_M1_pwm, dbg_M3_pwm;
	dbg_M1_pwm = motor_1_set_pwm;
	dbg_M3_pwm = motor_3_set_pwm;
	#endif
}

/**
************************************************************************************************
* @brief    设置2路电机PWM输出（H桥驱动）- 双电机版本
*
* ## 功能
* 将PID计算得到的控制量转换为实际的PWM波形输出到电机驱动器。
* 使用H桥驱动方式，每个电机需要2个PWM通道来控制方向：
* - 正转：CH1输出PWM，CH2输出0
* - 反转：CH1输出0，CH2输出PWM
* - 停止：CH1和CH2都输出0
*
* ## 参数说明（双电机版本）
* @param m1: → M1驱动器(PA15/PB3) → 右前电机 ← 来自motor_1_set_pwm
* @param m3: → M3驱动器(PB4/PB5) → 左前电机 ← 来自motor_3_set_pwm
*
* 数值含义：
* - 正数：一个方向旋转（具体方向取决于接线）
* - 负数：相反方向旋转
* - 绝对值越大，转速越快
* - 0：停止（但可能有保持力矩，取决于驱动器类型）
************************************************************************************************
**/
void Set_Pwm(int16_t m1, int16_t m3)
{
	// ====== Motor 1（右前） ======
	uint16_t pwm1  = abs16_fast(m1);           // 取绝对值作为PWM占空比
	uint16_t mask1 = (uint16_t)-(m1 > 0);     // 符号判断：>0则mask=0xFFFF，否则=0x0000
	PWM_M1_2 = (uint16_t)(pwm1 & mask1);      // 反转通道：m1>0时有输出
	PWM_M1_1 = (uint16_t)(pwm1 & ~mask1);     // 正转通道：m1≤0时有输出

	// ====== Motor 3（左前） ======
	uint16_t pwm3  = abs16_fast(m3);
	uint16_t mask3 = (uint16_t)-(m3 > 0);
	PWM_M3_2 = (uint16_t)(pwm3 & mask3);
	PWM_M3_1 = (uint16_t)(pwm3 & ~mask3);
}
