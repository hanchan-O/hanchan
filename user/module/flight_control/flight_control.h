#ifndef __FLIGHT_CONTROL_H__
#define __FLIGHT_CONTROL_H__

#include "main.h"
#include "motor.h"
#include "elrs.h"
#include <stdint.h>
#include <stdbool.h>

// ==================== 飞行控制优化系统（仿生版V2.0） ====================

// 姿态估计结构 - 基于电机位置推算飞行姿态
typedef struct {
    int16_t roll;           // 横滚角（左右倾斜）：正值=左高右低
    int16_t pitch;          // 俯仰角（前后倾斜）：正值=前高后低
    int16_t yaw_rate;       // 偏航速率（左右振幅差）
    int16_t lift_balance;   // 升力平衡度（4电机位置方差）
    int16_t sync_error[4];  // 各电机与平均位置的同步误差
    int16_t avg_position;   // 4电机平均位置
} Attitude_Est_t;



// 动态转向参数
typedef struct {
    int16_t base_amp;       // 基础振幅
    int16_t turn_ratio;     // 转向比例（-1.0 ~ +1.0）
    int16_t phase_diff;     // 相位差（0 ~ 4）
    int16_t bias_angle;     // 整体偏置角度
} TurnControl_t;

// 外部变量声明
extern Attitude_Est_t attitude;

// ====== 仿生非对称波形表（Q15格式） ======
// 25点波形表：下拍快(40%)，上挥慢(60%)，极限位置有缓冲
extern const int16_t BIOMIMETIC_WAVE[25];

// ====== 非阻塞状态机的静态状态（外部声明） ======
extern uint8_t  sm_idx;           // 0..24，波形表索引
extern int8_t   sm_dir;          // +1 正向 / -1 反向
extern uint32_t sm_next_tick;    // 下次步进的时刻（ms）
extern uint8_t  thr;             // 扑动频率控制

// ==================== 函数声明 ====================

// 限制函数
int16_t constrain(int16_t val, int16_t min, int16_t max);


// Q15 乘法： (a * b) >> 15
static inline int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * (int32_t)b) >> 15);
}

// 重置扑动状态机（用于模式切换或开关关闭时）
void reset_flap_state(void);

/**
************************************************************************************************
* @brief    计算仿生扑动步进时间（非对称时序）
* @param    throttle: 油门值（5-15）
* @param    is_downstroke: 是否为下拍阶段
* @return   步进时间间隔（毫秒）
************************************************************************************************
**/
uint32_t Calculate_Biomimetic_Step_Time(uint8_t throttle, uint8_t is_downstroke);

/**
************************************************************************************************
* @brief    动态幅度补偿 - 校准理论幅度与实际幅度的偏差
* @param    base_amplitude: 基准振幅
* @param    current_freq: 当前扑动频率(Hz)
* @return   补偿后的振幅
************************************************************************************************
**/
int16_t Apply_Amplitude_Compensation(int16_t base_amplitude, float current_freq);

/**
************************************************************************************************
* @brief    极限位置缓冲处理 - 防止冲击损坏齿轮箱
* @param    raw_target: 原始目标角度
* @param    motor_index: 电机索引(0-3)
* @return   缓冲后的目标角度
************************************************************************************************
**/
int16_t Apply_Limit_Buffer(int16_t raw_target, uint8_t motor_index);

/**
************************************************************************************************
* @brief    姿态估计函数 - 基于4电机位置推算飞行姿态
* @param    None
* @return   None
* @说明     计算横滚、俯仰、偏航速率和升力平衡度
************************************************************************************************
**/
void Estimate_Attitude(void);

/**
************************************************************************************************
* @brief    电机同步补偿 - 使4电机保持同步（增强版）
* @param    None
* @return   None
* @说明     当电机位置差异过大时，微调目标角度使电机趋向同步
************************************************************************************************
**/
void Motor_Sync_Compensate(void);



/**
************************************************************************************************
* @brief    动态转向计算 - 实现小半径转向
* @param    yaw_input: 转向输入（-100 ~ +100，来自elrs_data.Yaw）
* @param    turn_data: 输出转向参数结构
* @return   None
* @说明     根据摇杆输入计算动态振幅、相位差和扑动中心偏置
************************************************************************************************
**/
void Calculate_Dynamic_Turn(int16_t yaw_input, TurnControl_t* turn_data);

/**
************************************************************************************************
* @brief    计算扑动步进时间间隔（兼容旧接口）
* @param    throttle: 油门值（控制频率）
* @return   步进时间间隔（毫秒）
************************************************************************************************
**/
uint32_t Calculate_Flap_Step_Time(uint8_t throttle);

/**
************************************************************************************************
* @brief    执行仿生扑动步进 - 更新电机目标角度（核心优化版）
* @param    yaw_input: 转向输入（-100 ~ +100）
* @param    turn_ctrl: 转向控制参数
* @param    motor_front_L_ready: 左前电机基准位置
* @param    motor_front_R_ready: 右前电机基准位置
* @param    motor_back_L_ready: 左后电机基准位置
* @param    motor_back_R_ready: 右后电机基准位置
* @return   None
************************************************************************************************
**/
void Execute_Flap_Step(int16_t yaw_input,
                       TurnControl_t* turn_ctrl,
                       int16_t motor_M3_ready,    // M3(左前)基准位置
                       int16_t motor_M1_ready);   // M1(右前)基准位置

/**
************************************************************************************************
* @brief    自适应PID参数调整 - 根据频率动态优化控制参数 ⭐V6.4新增
* @param    current_freq: 当前扑动频率(Hz)
* @param    pid_motor1: M1电机的PID控制器指针
* @param    pid_motor3: M3电机的PID控制器指针
* @return   None
* @说明     低频(≤3Hz)：激进参数，快速响应
*          中频(3-5Hz)：平衡参数，兼顾响应与稳定
*          高频(>5Hz)：保守参数，优先稳定
************************************************************************************************
**/
void Adapt_PID_For_Frequency(float current_freq, pid_type_def* pid_motor1, pid_type_def* pid_motor3);

/**
************************************************************************************************
* @brief    动态缓冲区大小计算 - 根据频率调整极限保护强度 ⭐V6.4新增
* @param    current_freq: 当前扑动频率(Hz)
* @return   动态缓冲区大小（编码器单位）
* @说明     低频：小缓冲（80），响应快
*          中频：标准缓冲（120），平衡
*          高频：大缓冲（160），强保护
************************************************************************************************
**/
int16_t Get_Dynamic_Buffer_Size(float current_freq);

#endif // __FLIGHT_CONTROL_H__
