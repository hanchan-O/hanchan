/**
 * @file    flight_control.c
 * @brief   仿生扑动控制模块 - V6.10
 * @author  hanchan
 * @date    2026-04-15
 *
 * 功能：
 * - 对称波形生成（上下拍各50%时间）
 * - 动态频率-幅度补偿
 * - 转向控制（振幅差动+相位差+中心偏置）
 *
 * 可调参数：MIN_STEP_MS, base_amp, PID参数
 */

#define DEBUG_MODE

#include "flight_control.h"
#include "motor.h"
#include <math.h>

// ==================== 全局变量 ====================

Attitude_Est_t attitude = {0};

// 波形表（Q15格式）：25点对称，±50°对应±32767
const int16_t BIOMIMETIC_WAVE[25] = {
     32767, 27306, 21845, 16384, 10923,  5461,     0,  // 0-6: 上极限→中心
     -5461,-10923,-16384,-21845,-27306,-32767,       // 7-12: 中心→下极限
    -27306,-21845,-16384,-10923, -5461,     0,       // 13-18: 下极限→中心
      5461, 10923, 16384, 21845, 27306, 32767        // 19-24: 中心→上极限
};

// 状态机变量
uint8_t  sm_idx = 0;
int8_t   sm_dir = 1;
uint32_t sm_next_tick = 0;
uint8_t  thr = 0;

// 调试变量
volatile int16_t dbg_amp = 0;
volatile float   dbg_freq = 0.0f;
volatile int16_t dbg_wave_value = 0;
volatile int8_t  dbg_sm_idx = 0;

// ==================== 可调参数 ====================

#define DOWNSTROKE_RATIO 50     // 下拍时间占比（对称50%）
#define UPSTROKE_RATIO   50     // 上挥时间占比
#define MAX_BIOMIMETIC_FREQ 7   // 最大频率7Hz
#define MIN_STEP_MS 8           // 最小步进8ms（电机需6.4ms/8°）

// 内部变量
static volatile uint16_t actual_flap_count = 0;
static volatile float amplitude_compensation = 1.0f;

// ==================== 工具函数 ====================

int16_t constrain(int16_t val, int16_t min, int16_t max) {
    return (val < min) ? min : ((val > max) ? max : val);
}

void reset_flap_state(void) {
    sm_idx = 0;
    sm_dir = 1;
    sm_next_tick = HAL_GetTick();
    actual_flap_count = 0;
}

// ==================== 核心算法 ====================

/**
 * @brief 计算步进时间
 * @param throttle: 油门6-10
 * @param is_downstroke: 1=下拍, 0=上挥
 * @return 步进时间(ms)
 */
uint32_t Calculate_Biomimetic_Step_Time(uint8_t throttle, uint8_t is_downstroke) {
    if (throttle == 0) throttle = 5;
    
    const uint32_t BASE_PERIOD_MS = 2500U;
    uint32_t target_period_ms = BASE_PERIOD_MS / throttle;
    
    const uint32_t MIN_PERIOD_MS = 250U;  // 限制最高4Hz
    if (target_period_ms < MIN_PERIOD_MS) target_period_ms = MIN_PERIOD_MS;
    
    const uint8_t down_steps = 13;  // 索引0-12
    const uint8_t up_steps = 12;    // 索引13-24
    
    uint32_t step_ms;
    if (is_downstroke) {
        step_ms = (target_period_ms * DOWNSTROKE_RATIO / 100) / down_steps;
    } else {
        step_ms = (target_period_ms * UPSTROKE_RATIO / 100) / up_steps;
    }
    
    return (step_ms < MIN_STEP_MS) ? MIN_STEP_MS : step_ms;
}

/**
 * @brief 幅度补偿 - 高频时适当减小幅度
 */
int16_t Apply_Amplitude_Compensation(int16_t base_amplitude, float current_freq) {
    float compensation_factor;
    
    // base_amp=1024对应90°，高频小幅减小
    if (current_freq <= 2.5f) {
        compensation_factor = 1.00f;  // 90°
    } else if (current_freq <= 3.5f) {
        compensation_factor = 0.97f;  // 87°
    } else if (current_freq <= 4.5f) {
        compensation_factor = 0.94f;  // 85°
    } else {
        compensation_factor = 0.91f;  // 82°
    }
    
    return (int16_t)(base_amplitude * compensation_factor);
}

/**
 * @brief 自适应PID - 根据频率调整参数
 */
void Adapt_PID_For_Frequency(float current_freq, pid_type_def* pid_motor1, pid_type_def* pid_motor3) {
    float Kp, Ki, Kd;
    
    if (current_freq <= 3.0f) {
        Kp = 6.0f; Ki = 0.0f; Kd = 2.0f;  // 低频
    } else if (current_freq <= 4.0f) {
        Kp = 5.5f; Ki = 0.0f; Kd = 1.5f;  // 中频
    } else {
        Kp = 5.0f; Ki = 0.0f; Kd = 1.0f;  // 高频
    }
    
    if (pid_motor1 != NULL) {
        pid_motor1->Kp = Kp; pid_motor1->Ki = Ki; pid_motor1->Kd = Kd;
    }
    if (pid_motor3 != NULL) {
        pid_motor3->Kp = Kp; pid_motor3->Ki = Ki; pid_motor3->Kd = Kd;
    }
}

// Q15乘法: (a * b) >> 15
int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * (int32_t)b) >> 15);
}

// ==================== 转向控制 ====================

void Calculate_Dynamic_Turn(int16_t yaw_input, TurnControl_t* turn_data) {
    static int16_t last_yaw_input = 0;
    
    // 输入滤波（α=0.5）
    int16_t smooth_yaw = (last_yaw_input + yaw_input) / 2;
    last_yaw_input = smooth_yaw;
    
    // base_amp=1024对应90°总幅度
    turn_data->base_amp = 1024;
    
    float input_norm = (float)constrain(smooth_yaw, -100, 100) / 100.0f;
    float turn_curve = input_norm * fabsf(input_norm);
    
    turn_data->turn_ratio = (int16_t)(turn_curve * 100);
    turn_data->phase_diff = (int16_t)(fabsf(turn_curve) * 3);
    turn_data->bias_angle = (int16_t)(turn_curve * 40);
    
    // 参数滤波（α=0.4-0.8）
    static int16_t last_turn_ratio = 0;
    static int16_t last_phase_diff = 0;
    static int16_t last_bias_angle = 0;
    
    float abs_input = fabsf(input_norm);
    float alpha = 0.4f + abs_input * 0.4f;
    
    turn_data->turn_ratio = (int16_t)(alpha * turn_data->turn_ratio + (1-alpha) * last_turn_ratio);
    turn_data->phase_diff = (int16_t)(alpha * turn_data->phase_diff + (1-alpha) * last_phase_diff);
    turn_data->bias_angle = (int16_t)(alpha * turn_data->bias_angle + (1-alpha) * last_bias_angle);
    
    last_turn_ratio = turn_data->turn_ratio;
    last_phase_diff = turn_data->phase_diff;
    last_bias_angle = turn_data->bias_angle;
}

// ==================== 主执行函数 ====================

void Execute_Flap_Step(int16_t yaw_input, TurnControl_t* turn_ctrl,
                       int16_t motor_M3_ready, int16_t motor_M1_ready) {
    uint32_t current_tick = HAL_GetTick();
    if (current_tick < sm_next_tick) return;
    
    // 更新索引
    sm_idx += sm_dir;
    if (sm_idx >= 24) { sm_idx = 24; sm_dir = -1; actual_flap_count++; }
    if (sm_idx <= 0) { sm_idx = 0; sm_dir = 1; actual_flap_count++; }
    
    // 计算步进时间
    uint8_t is_downstroke = (sm_dir == 1) ? 1 : 0;
    uint32_t step_ms = Calculate_Biomimetic_Step_Time(thr, is_downstroke);
    sm_next_tick = current_tick + step_ms;
    
    // 计算频率和幅度
    float current_freq = 1000.0f / (step_ms * 25.0f);
    int16_t ampR_base = turn_ctrl->base_amp;
    int16_t ampL_base = turn_ctrl->base_amp;
    
    // 应用转向差动
    if (yaw_input < 0) {
        ampR_base = (int16_t)(ampR_base * (100 + turn_ctrl->turn_ratio) / 100.0f);
        ampL_base = (int16_t)(ampL_base * (100 - turn_ctrl->turn_ratio) / 100.0f);
    } else if (yaw_input > 0) {
        ampR_base = (int16_t)(ampR_base * (100 - turn_ctrl->turn_ratio) / 100.0f);
        ampL_base = (int16_t)(ampL_base * (100 + turn_ctrl->turn_ratio) / 100.0f);
    }
    
    // 限制幅度范围
    ampR_base = constrain(ampR_base, 600, 1100);
    ampL_base = constrain(ampL_base, 600, 1100);
    
    // 应用幅度补偿和自适应PID
    int16_t ampR = Apply_Amplitude_Compensation(ampR_base, current_freq);
    int16_t ampL = Apply_Amplitude_Compensation(ampL_base, current_freq);
    Adapt_PID_For_Frequency(current_freq, &motor_1_pid, &motor_3_pid);
    
    // 调试输出
    dbg_amp = ampL;
    dbg_freq = current_freq;
    
    // 计算相位偏移
    int8_t phase_offset = (yaw_input < 0) ? -turn_ctrl->phase_diff : turn_ctrl->phase_diff;
    int8_t idxR = sm_idx;
    int8_t idxL = sm_idx + phase_offset;
    if (idxL < 0) idxL = 0;
    if (idxL > 24) idxL = 24;
    
    // 获取波形值
    int16_t cR = BIOMIMETIC_WAVE[idxR];
    int16_t cL = BIOMIMETIC_WAVE[idxL];
    
    // 计算偏置
    int16_t biasR = -turn_ctrl->bias_angle;
    int16_t biasL = turn_ctrl->bias_angle;
    
    // 计算目标角度: midpoint + bias - q15_mul(amp, wave)
    int16_t raw_target_R = (int16_t)(motor_M1_ready + biasR - q15_mul(ampR, cR));
    int16_t raw_target_L = (int16_t)(motor_M3_ready + biasL - q15_mul(ampL, cL));
    
    // 输出到电机
    Wings_Data.Wings_motor[0].Target_Angle = raw_target_R;
    Wings_Data.Wings_motor[2].Target_Angle = raw_target_L;
    
    // 调试变量
    #ifdef DEBUG_MODE
    dbg_wave_value = cL;
    dbg_sm_idx = sm_idx;
    #endif
}

// ==================== 姿态估计 ====================

void Estimate_Attitude(void) {
    // 简化版：只使用M1和M3计算姿态
    int16_t pos_M1 = motor_1_pid.Corrective_Angle;
    int16_t pos_M3 = motor_3_pid.Corrective_Angle;
    
    // 计算左右翼平均位置
    int32_t avg_position = ((int32_t)pos_M1 + (int32_t)pos_M3) / 2;
    
    // 计算左右翼位置差（用于估计滚转）
    int16_t position_diff = pos_M1 - pos_M3;
    
    // 更新姿态结构
    attitude.roll_angle = (float)(position_diff) * 360.0f / 4096.0f;
    attitude.pitch_angle = (float)(avg_position - 2048) * 360.0f / 4096.0f;
    attitude.flap_frequency = dbg_freq;
    attitude.flap_amplitude = (float)dbg_amp * 360.0f / 4096.0f;
}
