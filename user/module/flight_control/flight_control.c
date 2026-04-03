#include "flight_control.h"
#include "motor.h"  // 

// ==================== 全局变量定义 ====================

// 姿态数据实例
Attitude_Est_t attitude = {0};

// ====== 定点余弦表（Q15），9点：0°~180°（步长22.5°） ======
// 索引0=0°(cos=1.0), 索引4=90°(cos=0), 索引8=180°(cos=-1.0)
// 用于扑动运动生成正弦波形
static const int16_t COS_Q15_15[9] = {
    30784, 25133,  16384,
    11207,     0, -11207,
   -16384,-25133,-30784
};

// ====== 非阻塞状态机的静态状态 ======
uint8_t  sm_idx = 0;           // 0..8，余弦表索引，从0开始（翅膀向上初始位置）
int8_t   sm_dir = 1;          // +1 正向(上→下) / -1 反向(下→上)，初始向下扑动
uint32_t sm_next_tick = 0;    // 下次步进的时刻（ms）
uint8_t  thr = 0;             // 扑动频率控制

// 扑动中心向下偏移量（增加升力）
#define FLAP_CENTER_OFFSET 200  // 向下偏移200编码器值

// ==================== 函数实现 ====================

// 限制函数
int16_t constrain(int16_t val, int16_t min, int16_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// 重置扑动状态机（用于模式切换或开关关闭时）
void reset_flap_state(void)
{
    sm_idx = 0;                     // 索引归零
    sm_dir = 1;                     // 方向正向
    sm_next_tick = HAL_GetTick();   // 重置时间戳
}

/**
************************************************************************************************
* @brief    姿态估计函数 - 基于4电机位置推算飞行姿态
* @param    None
* @return   None
* @说明     计算横滚、俯仰、偏航速率和升力平衡度
************************************************************************************************
**/
void Estimate_Attitude(void)
{
    // 计算4电机平均位置
    int32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += Wings_Data.Wings_motor[i].Corrective_Angle;
    }
    attitude.avg_position = (int16_t)(sum / 4);
    
    // 计算各电机同步误差（与平均位置的偏差）
    for (uint8_t i = 0; i < 4; i++) {
        attitude.sync_error[i] = Wings_Data.Wings_motor[i].Corrective_Angle - attitude.avg_position;
    }
    
    // 左右翅膀平均位置差 → 横滚角（正值=左侧高）
    int16_t left_avg = (Wings_Data.Wings_motor[1].Corrective_Angle + 
                        Wings_Data.Wings_motor[2].Corrective_Angle) / 2;
    int16_t right_avg = (Wings_Data.Wings_motor[0].Corrective_Angle + 
                         Wings_Data.Wings_motor[3].Corrective_Angle) / 2;
    attitude.roll = left_avg - right_avg;
    
    // 前后翅膀平均位置差 → 俯仰角（正值=前侧高）
    int16_t front_avg = (Wings_Data.Wings_motor[0].Corrective_Angle + 
                         Wings_Data.Wings_motor[2].Corrective_Angle) / 2;
    int16_t back_avg = (Wings_Data.Wings_motor[1].Corrective_Angle + 
                        Wings_Data.Wings_motor[3].Corrective_Angle) / 2;
    attitude.pitch = front_avg - back_avg;
    
    // 升力平衡度（4电机位置方差，越小越平衡）
    int32_t variance = 0;
    for (uint8_t i = 0; i < 4; i++) {
        int32_t diff = attitude.sync_error[i];
        variance += diff * diff;
    }
    attitude.lift_balance = (int16_t)(variance / 4);
}

/**
************************************************************************************************
* @brief    电机同步补偿 - 使4电机保持同步
* @param    None
* @return   None
* @说明     当电机位置差异过大时，微调目标角度使电机趋向同步
************************************************************************************************
**/
void Motor_Sync_Compensate(void)
{
    // 计算最大位置差
    int16_t max_diff = 0;
    for (uint8_t i = 0; i < 4; i++) {
        int16_t abs_diff = abs16_fast(attitude.sync_error[i]);
        if (abs_diff > max_diff) {
            max_diff = abs_diff;
        }
    }
    
    // 如果最大偏差超过阈值（50编码器值≈5°），启用同步补偿
    if (max_diff > 50) {
        const int16_t SYNC_GAIN = 3;  // 同步补偿增益
        for (uint8_t i = 0; i < 4; i++) {
            // 向平均位置靠拢，减小差异
            int16_t compensation = (attitude.sync_error[i] * SYNC_GAIN) / 10;
            Wings_Data.Wings_motor[i].Target_Angle -= compensation;
        }
    }
}

/**
************************************************************************************************
* @brief    动态转向计算 - 实现小半径转向
* @param    yaw_input: 转向输入（-100 ~ +100，来自elrs_data.Yaw）
* @param    turn_data: 输出转向参数结构
* @return   None
* @说明     根据摇杆输入计算动态振幅、相位差和扑动中心偏置
************************************************************************************************
**/
void Calculate_Dynamic_Turn(int16_t yaw_input, TurnControl_t* turn_data)
{
    // 基础振幅 - 配合MIN_AMP=600和MAX_AMP=1200
    turn_data->base_amp = 800;
    
    // 将输入映射到 -1.0 ~ +1.0（修正范围从500改为100，匹配elrs_data.Yaw的范围）
    float input_norm = (float)constrain(yaw_input, -100, 100) / 100.0f;
    
    // 非线性转向曲线：小输入精细控制，大输入快速转向
    // 左转（Yaw < 0）→ turn_ratio < 0 → 右翼振幅增大 → 飞机向左转
    // 右转（Yaw > 0）→ turn_ratio > 0 → 左翼振幅增大 → 飞机向右转
    float turn_curve = input_norm * fabsf(input_norm);
    
    // 转向比例（用于振幅差动）
    turn_data->turn_ratio = (int16_t)(turn_curve * 100);  // -100 ~ +100
    
    // 相位差（产生偏航力矩，缩小范围以减少姿态影响）
    turn_data->phase_diff = (int16_t)(fabsf(turn_curve) * 2);  // 0 ~ 2
    
    // 扑动中心偏置（转向时改变左右翼扑动中心位置）
    // 注意：编码器值减小=翅膀上移，编码器值增大=翅膀下移
    // 左转（Yaw<0）：需要左翼中心上移（值减小），右翼中心下移（值增大）
    // 右转（Yaw>0）：需要右翼中心上移（值减小），左翼中心下移（值增大）
    turn_data->bias_angle = (int16_t)(turn_curve * 30);  // -30 ~ +30（增大范围）
}

/**
************************************************************************************************
* @brief    计算扑动步进时间间隔
* @param    throttle: 油门值（控制频率）
* @return   步进时间间隔（毫秒）
************************************************************************************************
**/
uint32_t Calculate_Flap_Step_Time(uint8_t throttle)
{
    // 防止除零错误
    if (throttle == 0) throttle = 5;
    
    const uint32_t STEPS = 30U;
    uint32_t step_ms = (5000U + (STEPS * throttle / 2U)) / (STEPS * throttle);
    if (step_ms == 0) step_ms = 1;
    return step_ms;
}

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
void Execute_Flap_Step(int16_t yaw_input,
                       TurnControl_t* turn_ctrl,
                       int16_t motor_front_L_ready,
                       int16_t motor_front_R_ready,
                       int16_t motor_back_L_ready,
                       int16_t motor_back_R_ready)
{
    // 根据转向方向确定相位偏移（左转为负，右转为正）
    int8_t phase_offset = (yaw_input < 0) ? -turn_ctrl->phase_diff : turn_ctrl->phase_diff;

    // 计算动态振幅（带转向差动）
    // 左转（Yaw<0，turn_ratio<0）：右翼振幅增大，左翼振幅减小
    // 右转（Yaw>0，turn_ratio>0）：左翼振幅增大，右翼振幅减小
    int16_t amp_diff = (turn_ctrl->turn_ratio * 25) / 10;  // 系数2.5，最大±250，减小差动幅度使转向更平滑
    // 注意：amp_diff的符号与Yaw相同
    // Yaw<0时amp_diff<0，需要ampR增大 → ampR = base - amp_diff（减负=加正）
    // Yaw>0时amp_diff>0，需要ampL增大 → ampL = base + amp_diff
    int16_t ampR = turn_ctrl->base_amp - amp_diff;   // 右翼振幅
    int16_t ampL = turn_ctrl->base_amp + amp_diff;   // 左翼振幅

    // 限制振幅范围（保证最小升力，防止转向时一侧振幅过小）
    const int16_t MIN_AMP = 750;  // 最小振幅保护，从600提高到750，防止小振幅时电机卡顿
    const int16_t MAX_AMP = 1200; // 最大振幅限制
    ampR = constrain(ampR, MIN_AMP, MAX_AMP);
    ampL = constrain(ampL, MIN_AMP, MAX_AMP);

    // 计算左右翼相位索引（带转向相位差）
    int8_t idxR = sm_idx;                           // 右翼索引
    int8_t idxL = sm_idx + phase_offset;            // 左翼索引（带偏移）

    // 限制索引范围（镜像对称）
    if (idxL < 0) idxL = 0;
    if (idxL > 8) idxL = 8;

    // 获取余弦值
    int16_t cR = COS_Q15_15[idxR];
    int16_t cL = COS_Q15_15[idxL];

    // 扑动中心偏置（分别应用到左右翼）
    // 注意：编码器值减小=翅膀上移，编码器值增大=翅膀下移
    // 左转（Yaw<0，bias_angle<0）：左翼中心上移（值减小），右翼中心下移（值增大）
    // 右转（Yaw>0，bias_angle>0）：右翼中心上移（值减小），左翼中心下移（值增大）
    int16_t biasR = -turn_ctrl->bias_angle;  // 右翼偏置（与Yaw同向）
    int16_t biasL = turn_ctrl->bias_angle;   // 左翼偏置（与Yaw反向）

    // 设置目标角度（带动态振幅和相位差）
    // 添加 FLAP_CENTER_OFFSET 使扑动中心向下偏移，增加升力
    // 右翼：电机0(右前) + 电机3(右后)
    Wings_Data.Wings_motor[0].Target_Angle = (int16_t)(motor_front_R_ready + biasR + FLAP_CENTER_OFFSET - q15_mul(ampR, cR));
    Wings_Data.Wings_motor[3].Target_Angle = (int16_t)(motor_back_R_ready  + biasR + FLAP_CENTER_OFFSET - q15_mul(ampR, cR));

    // 左翼：电机2(左前) + 电机1(左后)
    Wings_Data.Wings_motor[2].Target_Angle = (int16_t)(motor_front_L_ready + biasL + FLAP_CENTER_OFFSET - q15_mul(ampL, cL));
    Wings_Data.Wings_motor[1].Target_Angle = (int16_t)(motor_back_L_ready  + biasL + FLAP_CENTER_OFFSET - q15_mul(ampL, cL));

    // 更新索引与方向
    if (sm_dir > 0) {
        if (++sm_idx >= 8) { sm_idx = 8; sm_dir = -1; }  // 正向：0→8（上→下）
    } else {
        if (sm_idx-- == 0) { sm_idx = 0; sm_dir = +1; }  // 反向：8→0（下→上）
    }
}
