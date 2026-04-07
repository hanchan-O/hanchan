#ifndef __FLIGHT_CONTROL_H__
#define __FLIGHT_CONTROL_H__

#include "main.h"
#include "motor.h"
#include "elrs.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * ========== 扑翼频率（厘赫兹 cHz）==========
 * 扑翼频率以 cHz 表示：频率(Hz) * 100。例如 1.68 Hz -> 168，5.05 Hz -> 505。
 * Calculate_Flap_Step_Time() 结合 FLAP_STEPS_PER_CYCLE 得到每步周期 ms。
 */
//+++260407:hww:扑翼参数与姿态/同步补偿接口
#define FLAP_FREQ_MIN_cHz   168U   /* 最小扑翼频率 *100，即 1.68 Hz */
#define FLAP_FREQ_MAX_cHz   505U   /* 最大扑翼频率 *100，即 5.05 Hz */

/* 油门映射到扑翼幅度范围；elrs Throttle 通道 */
#define FLAP_THR_MAP_MIN    5U
#define FLAP_THR_MAP_MAX    15U

/* 每扑动周期步数；Execute_Flap_Step 每调用一次 sm_idx 前进一步 */
#define FLAP_STEPS_PER_CYCLE  18U

#if FLAP_FREQ_MAX_cHz <= FLAP_FREQ_MIN_cHz
#error FLAP_FREQ_MAX_cHz must be > FLAP_FREQ_MIN_cHz
#endif
#if FLAP_THR_MAP_MAX <= FLAP_THR_MAP_MIN
#error FLAP_THR_MAP_MAX must be > FLAP_THR_MAP_MIN
#endif

/* 简易姿态估计（基于翼面角度） */
typedef struct {
    int16_t roll;           /* 横滚相关量（翼面差） */
    int16_t pitch;          /* 俯仰（当前置 0） */
    int16_t yaw_rate;       /* 偏航输入相关 */
    int16_t lift_balance;   /* 升力/对称性度量 */
    int16_t sync_error[4];  /* 各翼相对平均位置的误差 */
    int16_t avg_position;   /* 平均翼面位置 */
} Attitude_Est_t;

/* 动态转弯/扑翼幅度参数 */
typedef struct {
    int16_t base_amp;    /* 基础幅值，映射到 Target_Angle 幅度 */
    int16_t turn_ratio;  /* 转弯强度 */
    int16_t phase_diff;  /* 左右翼相位差 */
    int16_t bias_angle;  /* 偏置角 */
} TurnControl_t;

extern Attitude_Est_t attitude;

extern uint8_t  sm_idx;        /* 扑动相位索引 0..8 */
extern int8_t   sm_dir;        /* 步进方向 +1 / -1 */
extern uint32_t sm_next_tick;  /* 下一步时刻 [ms] */
extern uint8_t  thr;           /* 当前油门/映射用 */

int16_t constrain(int16_t val, int16_t min, int16_t max);

/* Q15 定点乘法（结果右移 15 位） */
static inline int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * (int32_t)b) >> 15);
}

void reset_flap_state(void);
void Estimate_Attitude(void);
void Motor_Sync_Compensate(void);
void Calculate_Dynamic_Turn(int16_t yaw_input, uint8_t throttle, TurnControl_t* turn_data);
uint32_t Calculate_Flap_Step_Time(uint8_t throttle);
void Execute_Flap_Step(int16_t yaw_input,
                       TurnControl_t* turn_ctrl,
                       int16_t motor_front_L_ready,
                       int16_t motor_front_R_ready,
                       int16_t motor_back_L_ready,
                       int16_t motor_back_R_ready);

#endif
