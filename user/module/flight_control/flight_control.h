#ifndef __FLIGHT_CONTROL_H__
#define __FLIGHT_CONTROL_H__

#include "main.h"
#include "motor.h"
#include "elrs.h"
#include <stdint.h>
#include <stdbool.h>

// ==================== 飞行控制优化系统 ====================

// 姿态估计结构 - 基于电机位置推算飞行姿态
typedef struct {
    int16_t roll;           // 横滚角（左右倾斜）：正值=左高右低
    int16_t pitch;          // 俯仰角（前后倾斜）：正值=前高后低
    int16_t yaw_rate;       // 偏航速率（左右振幅差）
    int16_t lift_balance;   // 升力平衡度（4电机位置方差）
    int16_t sync_error[4];  // 各电机与平均位置的同步误差
    int16_t avg_position;   // 4电机平均位置
} Attitude_Est_t;

// 航向保持结构
typedef struct {
    int32_t yaw_integral;       // 偏航积分（估算航向变化）
    int16_t heading_error;      // 航向误差
    uint8_t hold_enabled;       // 保持使能标志
    int16_t last_roll;          // 上次的横滚值（用于计算偏航速率）
} Heading_Hold_t;

// 动态转向参数
typedef struct {
    int16_t base_amp;       // 基础振幅
    int16_t turn_ratio;     // 转向比例（-1.0 ~ +1.0）
    int16_t phase_diff;     // 相位差（0 ~ 4）
    int16_t bias_angle;     // 整体偏置角度
} TurnControl_t;

// 外部变量声明
extern Attitude_Est_t attitude;
extern Heading_Hold_t heading_hold;

// ====== 定点余弦表（Q15），9点：0°~180°（步长22.5°） ======
// 索引0=0°(cos=1.0), 索引4=90°(cos=0), 索引8=180°(cos=-1.0)
// 用于扑动运动生成正弦波形
extern const int16_t COS_Q15_15[9];

// ====== 非阻塞状态机的静态状态（外部声明） ======
extern uint8_t  sm_idx;           // 0..8，余弦表索引
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
* @brief    姿态估计函数 - 基于4电机位置推算飞行姿态
* @param    None
* @return   None
* @说明     计算横滚、俯仰、偏航速率和升力平衡度
************************************************************************************************
**/
void Estimate_Attitude(void);

/**
************************************************************************************************
* @brief    电机同步补偿 - 使4电机保持同步
* @param    None
* @return   None
* @说明     当电机位置差异过大时，微调目标角度使电机趋向同步
************************************************************************************************
**/
void Motor_Sync_Compensate(void);

/**
************************************************************************************************
* @brief    姿态稳定补偿 - 自动修正飞行姿态
* @param    None
* @return   None
* @说明     根据横滚和俯仰角，微调电机目标角度产生恢复力矩
************************************************************************************************
**/
void Attitude_Stabilize(void);

/**
************************************************************************************************
* @brief    航向保持更新 - 自动保持直线飞行
* @param    enable: 1=使能航向保持, 0=重置
* @return   None
* @说明     通过积分偏航速率估算航向变化，自动产生反向转向力矩
************************************************************************************************
**/
void Heading_Hold_Update(uint8_t enable);

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
* @brief    计算扑动步进时间间隔
* @param    throttle: 油门值（控制频率）
* @return   步进时间间隔（毫秒）
************************************************************************************************
**/
uint32_t Calculate_Flap_Step_Time(uint8_t throttle);

/**
************************************************************************************************
* @brief    执行扑动步进 - 更新电机目标角度
* @param    turn_ctrl: 转向控制参数
* @param    motor_front_L_ready: 左前电机基准位置
* @param    motor_front_R_ready: 右前电机基准位置
* @param    motor_back_L_ready: 左后电机基准位置
* @param    motor_back_R_ready: 右后电机基准位置
* @return   None
************************************************************************************************
**/
void Execute_Flap_Step(TurnControl_t* turn_ctrl,
                       int16_t motor_front_L_ready,
                       int16_t motor_front_R_ready,
                       int16_t motor_back_L_ready,
                       int16_t motor_back_R_ready);

#endif // __FLIGHT_CONTROL_H__
