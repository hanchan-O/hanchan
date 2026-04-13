/**
 * @file    flight_control.c
 * @brief   仿生扑动控制模块（V2.0优化版）
 * @author  hanchan
 * @date    2026-04-10
 *
 * ## 模块功能
 * 本模块实现仿生蝴蝶的扑动控制算法，包括：
 * - 非对称波形生成（下拍快40%，上挥慢60%）
 * - 极限位置缓冲保护（防止齿轮箱冲击）
 * - 动态频率-幅度补偿（校准高频衰减）
 * - 姿态估计与电机同步补偿
 * - 动态转向控制（振幅差动+相位差+中心偏置）
 *
 * ## 调试要点
 * 所有可调参数都在文件开头定义，带注释说明作用范围
 * 修改后重新编译即可生效
 */

// 【关键】必须与main.c保持一致，否则调试代码不会被编译！
#define DEBUG_MODE

#include "flight_control.h"
#include "motor.h"  
#include <math.h>

// ==================== 全局变量定义 ====================

/**
 * @brief 姿态数据实例
 * 存储基于2电机位置推算的飞行姿态信息（双电机版本）
 */
Attitude_Est_t attitude = {0};

// ====== 【核心】仿生非对称波形表（Q15格式） ======
// 🎯 可调参数：修改此数组可自定义扑动轨迹
//
// 特点：下拍快(40%时间)，上挥慢(60%时间)，极限位置有缓冲
// 总共25点，覆盖完整周期（上极限→下极限→上极限）
//
// 数值范围：-32768 ~ +32767（Q15定点数格式）
// +32767 = 上极限位置（翅膀最高点）
// -28900 ≈ 下极限位置（保留缓冲区，不完全到达最低点）
//
// 时间分配：
// 索引 0-12 (13点)：下拍阶段，占40%时间 → 快速下降产生升力
// 索引 13    (1点) ：极限缓冲，短暂停顿 → 避免机械冲击
// 索引 14-24 (11点)：上挥阶段，占60%时间 → 缓慢恢复节省能量
// 【修复】保持与头文件一致，去掉static，保持const
const int16_t BIOMIMETIC_WAVE[25] = {
    // 下拍阶段（索引0-12，占40%时间）⬇️ 快速
    32767,   // [0]  上极限起点（cos=1.0）← 从这里开始
    32000,   // [1]  开始加速下拍
    29491,   // [2]  加速中（速度增加）
    24500,   // [3]  快速下拍（最大速度区域）
    18204,   // [4]  继续下拍
    11300,   // [5]  接近中位（过水平位置）
     6393,   // [6]  过中位以下
        0,   // [7]  中位以下（cos=0°）
    -6393,   // [8]  继续下拍
   -12540,   // [9]  接近下极限
   -18204,   // [10] 准备进入缓冲区
   -23170,   // [11] 减速中（为缓冲做准备）
   -27146,   // [12] 进入极限缓冲区
    
    // 极限位置缓冲（索引13，短暂停顿）⏸️
   -28900,   // [13] 下极限（接近但不完全到达，保护塑料齿轮箱！）
    
    // 上挥阶段（索引14-24，占60%时间）⬆️ 缓慢
   -26384,   // [14] 缓慢开始上挥（比下拍慢得多）
   -22986,   // [15] 慢速上挥
   -18794,   // [16] 继续（空气阻力帮助减速）
   -13856,   // [17]
    -8200,   // [18]
    -3000,   // [19]
     2500,   // [20]
     8000,   // [21]
    13500,   // [22]
    20000,   // [23]
    28000    // [24] 接近上极限（准备回到[0]）
};

// ====== 非阻塞状态机的静态状态 ======
uint8_t  sm_idx = 0;           // 波形表当前索引（0~24）
int8_t   sm_dir = 1;          // 扫描方向：+1=正向(0→24), -1=反向(24→0)
uint32_t sm_next_tick = 0;    // 下次步进的时刻戳（ms），用于非阻塞定时
uint8_t  thr = 0;             // 当前油门值（5-15），控制扑动频率

// ==================== 【重要】可调参数配置区 ====================
// 🔧 调试时主要修改这里的参数！

/**
 * @brief 扑动中心向下偏移量
 * 
 * 作用：将整个扑动范围向下偏移，使翅膀平均位置更低
 * 效果：增大升力（类似鸟类调整迎角）
 * 
 * 可调范围：100 ~ 300
 * 当前值：250（中等偏移，V6.4优化：增大升力）
 * 建议：
 *   - 升力不足 → 增大到280-300
 *   - 冲击太大 → 减小到200
 */
#define FLAP_CENTER_OFFSET 250

/**
 * @brief 下拍时间占比（百分比）
 * 
 * 作用：控制下拍阶段的相对速度
 * 效果：较小的值 = 更快的下拍 = 更大的瞬时升力
 * 
 * 可调范围：30 ~ 50
 * 当前值：35（V6.4优化：更快下拍，增大升力）
 * 建议：
 *   - 想要更大升力 → 改为30（最快下拍）
 *   - 想要更平滑 → 改为40（稍慢）
 */
#define DOWNSTROKE_RATIO 35

/**
 * @brief 上挥时间占比（百分比）
 * 
 * 注意：UPSTROKE_RATIO + DOWNSTROKE_RATIO 应该 ≈ 100
 * 上挥慢可以减少空气阻力，节省能量
 * 
 * 可调范围：50 ~ 70
 * 当前值：65（V6.4优化：更慢上挥，节省能量）
 * 建议：
 *   - 想要更省电 → 改为70（最慢上挥）
 *   - 想要更高频 → 改为60（稍快）
 */
#define UPSTROKE_RATIO   65

/**
 * @brief 极限位置缓冲区大小（编码器单位）
 * 
 * 作用：在到达物理极限前提前停止/转向，避免冲击损坏齿轮箱
 * 原理：检测到方向变化时，目标角度不达到极限，而是留出安全距离
 * 
 * ⚠️ 安全警告：太小会导致"咔咔"冲击声，甚至损坏塑料齿轮箱！
 * 
 * 可调范围：80 ~ 200
 * 当前值：120（V6.4优化：增大保护，适应高频扑动）
 * 建议：
 *   - 有冲击声 → 增大到150
 *   - 幅度不够大 → 可以尝试减小到100（但有风险！）
 */
#define BUFFER_ZONE_SIZE 120

/**
 * @brief 最大允许扑动频率（Hz）
 * 
 * 限制原因：基于电机物理特性计算的安全上限
 * 电机规格：360 RPM空载转速 @ 7.4V
 * 理论最大频率：360/60 = 6 Hz（无负载时）
 * 实际工作频率应 ≤ 7 Hz（考虑负载和齿轮箱寿命）
 * 
 * ⚠️ 超过7Hz可能导致：
 *   - 电机过热
 *   - PID跟踪不上（幅度衰减严重）
 *   - 齿轮箱快速磨损
 * 
 * 可调范围：4 ~ 7
 * 当前值：7（V6.4优化：支持更高频扑动）
 * 建议：
 *   - 保护优先 → 改为5或6
 *   - 需要高性能 → 保持7（但密切监测温度）
 */
#define MAX_BIOMIMETIC_FREQ 7

/**
 * @brief 最小步进时间间隔（毫秒）
 * 
 * 作用：限制状态机更新频率，防止过于频繁的计算
 * 影响：与MAX_BIOMIMETIC_FREQ配合使用
 * 
 * 计算公式：MIN_STEP_MS ≈ 1000 / (MAX_FREQ × 总步数)
 * 当前设置：12ms（V6.4优化：支持7Hz高频扑动）
 * 
 * 可调范围：10 ~ 20
 * 当前值：12
 * 建议：
 *   - 需要更高频 → 减小到10（但不要低于10）
 *   - 需要更稳定 → 增大到16
 */
#define MIN_STEP_MS 12

// ==================== 动态补偿系统内部变量 ====================
static volatile uint16_t actual_flap_count = 0;       // 实际扑动次数统计（用于调试）
static volatile float amplitude_compensation = 1.0f;  // 幅度补偿系数（自动计算）
static uint32_t last_period_time = 0;        // 上次周期完成时间（用于测频）

// ==================== 函数实现 ====================

/**
 * @brief 限制函数 - 将数值约束在指定范围内
 * 
 * 用途：防止参数越界导致异常行为
 * 
 * @param val: 输入值
 * @param min: 最小允许值
 * @param max: 最大允许值
 * @return: 约束后的值（min ≤ return ≤ max）
 */
int16_t constrain(int16_t val, int16_t min, int16_t max) {
    if (val < min) return min;
    if (val > max) return val;
    return val;
}

/**
 * @brief 重置扑动状态机
 * 
 * 触发时机：
 * - 模式切换时（Mode1→Mode2或Mode2→Mode0）
 * - 开关关闭时
 * - 异常恢复时
 * 
 * 作用：将状态机归零，确保下次启动从正确位置开始
 */
void reset_flap_state(void)
{
    sm_idx = 0;                     // 索引归零（从上极限开始）
    sm_dir = 1;                     // 方向设为正向（先下拍）
    sm_next_tick = HAL_GetTick();   // 重置时间戳为当前时刻
    actual_flap_count = 0;          // 清零扑动计数器
}

/**
************************************************************************************************
* @brief    计算仿生扑动步进时间（非对称时序）⭐核心算法
*
* ## 背景
* 传统对称波形每个步进时间相同，但真实蝴蝶的下拍明显快于上挥。
* 本函数根据当前处于哪个阶段（下拍/上挥），返回不同的步进时间。
*
* ## 思路
* 将总周期按照 DOWNSTROKE_RATIO : UPSTROKE_RATIO 的比例分配给两个阶段。
* 下拍阶段步进数少但时间短 → 每步速度快
* 上挥阶段步进数多但时间长 → 每步速度慢
*
* ## 步骤
* 1. 根据油门值计算目标周期（5000ms / throttle）
* 2. 限制最小周期（不超过6Hz）
* 3. 判断当前是下拍还是上挥阶段
* 4. 按比例计算该阶段的步进时间
* 5. 应用最小步进时间限制
*
* ## 参数说明
* @param throttle: 油门值（5-15，来自遥控器通道2）
*                 - 5 = 最慢（约2-3Hz）
*                 - 10 = 中等（约4-5Hz）
*                 - 15 = 最快（约6Hz，受限于MAX_BIOMIMETIC_FREQ）
* @param is_downstroke: 是否为下拍阶段（0=否/上挥，1=是/下拍）
*
* @return 步进时间间隔（毫秒），典型值：10-33ms
*
* ## 示例
* throttle=10, is_downstroke=1:
*   目标周期 = 5000/10 = 500ms
*   下拍时间 = 500×40% = 200ms
*   下拍步数 = 13步
*   步进时间 = 200/13 ≈ 15ms
*
* ## 异常
* - throttle=0: 自动修正为5（防止除零错误）
* - 计算结果<16ms: 强制限制为16ms（保护电机）
************************************************************************************************
**/
uint32_t Calculate_Biomimetic_Step_Time(uint8_t throttle, uint8_t is_downstroke)
{
    if (throttle == 0) throttle = 5;
    
    const uint32_t BASE_PERIOD_MS = 3000U;  // 基准周期3秒（V6.4优化：支持4-5Hz）
    
    uint32_t target_period_ms = BASE_PERIOD_MS / throttle;
    
    const uint32_t MIN_PERIOD_MS = 143U;  // 7Hz对应的最小周期（143ms≈1000/7）
    if (target_period_ms < MIN_PERIOD_MS) target_period_ms = MIN_PERIOD_MS;
    
    // V6.6修复：使用实际的波形表步数
    // 下拍区域：索引0-13，共14步
    // 上挥区域：索引14-24，共11步
    const uint8_t down_steps = 14;  // 索引0-13
    const uint8_t up_steps = 11;    // 索引14-24
    
    uint32_t step_ms;
    if (is_downstroke) {
        // 下拍：35%的时间分配给14步
        step_ms = (target_period_ms * DOWNSTROKE_RATIO / 100) / down_steps;
    } else {
        // 上挥：65%的时间分配给11步
        step_ms = (target_period_ms * UPSTROKE_RATIO / 100) / up_steps;
    }
    
    if (step_ms < MIN_STEP_MS) step_ms = MIN_STEP_MS;
    
    return step_ms;
}

/**
************************************************************************************************
* @brief    动态幅度补偿 - 校准理论幅度与实际幅度的偏差
*
* ## 背景
* 高频扑动时，由于以下因素，实际振幅会小于理论设定值：
* - 编码器滤波延迟（α=0.4的一阶低通滤波）
* - PID响应滞后（Kp=12-15的位置式PID）
* - 电机惯性（空心杯+减速比的机械惯性）
* - 空气阻力（随频率平方增长）
*
* 实验数据表明：
*   3Hz以下：衰减<5%（可忽略）
*   3-5Hz：衰减约8%
*   5Hz以上：衰减约15%
*
* ## 思路
* 根据当前扑动频率，应用预计算的补偿系数放大目标振幅，
* 使最终实际振幅接近理论设定值。
*
* ## 步骤
* 1. 判断当前频率属于哪个区间
* 2. 选择对应的补偿系数
* 3. 将基准振幅乘以系数得到补偿后的振幅
*
* ## 参数说明
* @param base_amplitude: 基准振幅（编码器值，通常800-1100）
* @param current_freq: 当前扑动频率（Hz，浮点数）
*
* @return 补偿后的振幅（可能超过base_amplitude 8-15%）
*
* ## 调优建议
* 如果发现高频时幅度仍然不足：
* - 将中频补偿从1.08改为1.10-1.12
* - 将高频补偿从1.15改为1.18-1.20
* 但注意不要过度补偿（>1.25可能导致超调抖动）
************************************************************************************************
**/
int16_t Apply_Amplitude_Compensation(int16_t base_amplitude, float current_freq)
{
    float compensation_factor;
    
    if (current_freq <= 3.0f) {
        compensation_factor = 1.0f;
    } else if (current_freq <= 5.0f) {
        compensation_factor = 1.08f;
    } else {
        compensation_factor = 1.15f;
    }
    
    return (int16_t)(base_amplitude * compensation_factor);
}

/**
************************************************************************************************
* @brief    自适应PID参数调整 - 根据频率动态优化控制参数 ⭐V6.4核心优化
*
* ## 背景
* 不同扑动频率下，电机和机械系统的响应特性差异很大：
* - 低频（≤3Hz）：电机有充足时间响应，可以使用激进参数追求快速跟踪
* - 中频（3-5Hz）：响应时间适中，需要平衡响应速度和稳定性
* - 高频（>5Hz）：响应时间紧张，必须使用保守参数避免超调和震荡
*
* ## 思路
* 根据当前扑动频率，动态调整PID参数：
* - Kp（比例增益）：低频大，高频小（控制响应速度）
* - Ki（积分增益）：低频大，高频小（消除稳态误差）
* - Kd（微分增益）：低频小，高频大（增强阻尼，抑制超调）
*
* ## 参数说明
* @param current_freq: 当前扑动频率(Hz)
* @param pid_motor1: M1(右前)电机的PID控制器指针
* @param pid_motor3: M3(左前)电机的PID控制器指针
*
* ## 调优建议
* 如果出现以下问题：
* - 低频跟踪慢 → 增大低频Kp（18→20）
* - 高频超调大 → 增大高频Kd（5→6）
* - 中频震荡 → 减小中频Kp（14→12）
************************************************************************************************
**/
void Adapt_PID_For_Frequency(float current_freq, pid_type_def* pid_motor1, pid_type_def* pid_motor3)
{
    float Kp, Ki, Kd;
    
    // V6.6调试：进一步降低Kp（刹车），大幅增大Kd（阻尼）
    // 所有频率段都使用保守参数，优先稳定
    
    if (current_freq <= 3.0f) {
        // 低频模式（≤3Hz）：V6.6调试参数
        Kp = 6.0f;    // V6.6大幅降低：8→6（更强刹车）
        Ki = 0.01f;   // V6.6大幅降低：0.02→0.01（减少积分累积）
        Kd = 10.0f;   // V6.6大幅增大：8→10（更强阻尼，刹车）
    } 
    else if (current_freq <= 5.0f) {
        // 中频模式（3-5Hz）：V6.6调试参数
        Kp = 5.0f;    // V6.6大幅降低：6→5（更强刹车）
        Ki = 0.008f;  // V6.6大幅降低：0.01→0.008
        Kd = 12.0f;   // V6.6大幅增大：10→12（更强阻尼）
    } 
    else {
        // 高频模式（>5Hz）：V6.6调试参数
        Kp = 4.0f;    // V6.6大幅降低：5→4（最强刹车）
        Ki = 0.005f;  // V6.6大幅降低：0.005→0.005
        Kd = 15.0f;   // V6.6大幅增大：12→15（最强阻尼）
    }
    
    // 应用到两个电机（双电机版本）
    if (pid_motor1 != NULL) {
        pid_motor1->Kp = Kp;
        pid_motor1->Ki = Ki;
        pid_motor1->Kd = Kd;
    }
    if (pid_motor3 != NULL) {
        pid_motor3->Kp = Kp;
        pid_motor3->Ki = Ki;
        pid_motor3->Kd = Kd;
    }
}

/**
************************************************************************************************
* @brief    动态缓冲区大小计算 - 根据频率调整极限保护强度 ⭐V6.4新增
*
* ## 背景
* 不同频率下，电机在极限位置的惯性冲击力不同：
* - 低频：惯性小，可以使用小缓冲区，获得更大的有效行程
* - 高频：惯性大，必须使用大缓冲区，保护齿轮箱
*
* ## 思路
* 根据频率动态调整BUFFER_ZONE_SIZE：
* - 频率越高，缓冲区越大
* - 使用分段线性插值，平滑过渡
*
* ## 参数说明
* @param current_freq: 当前扑动频率(Hz)
* @return 动态缓冲区大小（编码器单位）
*
* ## 效果对比
* 固定缓冲区（100）：
*   低频：浪费行程，振幅不足
*   高频：保护不足，可能冲击
*
* 动态缓冲区：
*   低频（2Hz）：缓冲80，有效行程大
*   中频（4Hz）：缓冲120，平衡
*   高频（6Hz）：缓冲160，强保护
************************************************************************************************
**/
int16_t Get_Dynamic_Buffer_Size(float current_freq)
{
    int16_t buffer_size;
    
    if (current_freq <= 3.0f) {
        // 低频：小缓冲，响应快
        buffer_size = 80;
    } 
    else if (current_freq <= 5.0f) {
        // 中频：线性插值 80→160
        // 公式：80 + (freq - 3) / (5 - 3) * (160 - 80)
        buffer_size = (int16_t)(80 + (current_freq - 3.0f) * 40.0f);
    } 
    else {
        // 高频：大缓冲，强保护
        buffer_size = 160;
    }
    
    return buffer_size;
}

/**
************************************************************************************************
* @brief    极限位置缓冲处理 - 防止冲击损坏齿轮箱 ⭐关键保护机制
*
* ## 背景
* 当电机到达扑动的上下极限位置并立即反转方向时，
* 会产生巨大的惯性冲击力（F = m × Δv / Δt）。
* 对于塑料齿轮箱（最大承受400 gf·cm扭矩），
* 这种冲击可能导致齿轮崩齿或轴断裂。
*
* ## 思路
* 通过检测目标角度的变化方向（速度符号变化），
* 判断是否即将到达极限位置。
* 如果检测到方向变化，则不将目标设为完全的极限位置，
* 而是提前BUFFER_ZONE_SIZE的距离停止，
* 让PID自然减速后再反转。
*
* ## 步骤
* 1. 计算当前速度（本次目标 - 上次目标）
* 2. 检测速度符号是否改变（正变负 或 负变正）
* 3. 如果方向改变：
*    - 正→负：到达上极限，目标下移BUFFER_ZONE_SIZE
*    - 负→正：到达下极限，目标上移BUFFER_ZONE_SIZE
* 4. 否则正常传递目标值
*
* ## 参数说明
* @param raw_target: 原始目标角度（未经处理的）
* @param motor_index: 电机索引（0-3，用于独立记录每个电机的状态）
*
* @return 缓冲处理后的目标角度
*
* ## 效果对比
* 无缓冲：目标直接跳变到极限 → PID全力输出 → 冲击力大
* 有缓冲：目标提前停止 → PID自然减速 → 平滑过渡
*
* ## 调优建议
* 如果仍有轻微"咔咔"声：
* - 增大BUFFER_ZONE_SIZE到150
* - 或者降低MAX_AMP到1000
************************************************************************************************
**/
int16_t Apply_Limit_Buffer(int16_t raw_target, uint8_t motor_index)
{
    // 双电机版本：数组大小优化为[2]，节省RAM
    // 索引映射：motor_index=0(M1) → array[0], motor_index=2(M3) → array[1]
    static int16_t last_target[2] = {0};  
    static int16_t velocity[2] = {0};     
    uint8_t idx = (motor_index == 2) ? 1 : 0;  // 索引映射
    
    int16_t new_velocity = raw_target - last_target[idx];
    
    if ((velocity[idx] > 0 && new_velocity < 0) || 
        (velocity[idx] < 0 && new_velocity > 0)) {
        
        int16_t buffer_target;
        
        if (new_velocity < 0) {
            buffer_target = raw_target + BUFFER_ZONE_SIZE;
        } else {
            buffer_target = raw_target - BUFFER_ZONE_SIZE;
        }
        
        last_target[idx] = buffer_target;
        velocity[idx] = new_velocity;
        return buffer_target;
    }
    
    last_target[idx] = raw_target;
    velocity[idx] = new_velocity;
    return raw_target;
}

/**
************************************************************************************************
* @brief    姿态估计函数 - 基于2电机位置推算飞行姿态（双电机版本）
*
* ## 功能
* 由于本项目没有IMU（惯性测量单元），无法直接获取飞机的姿态角。
* 本函数通过分析2个电机的当前位置（M1右前+M3左前），间接推断飞行姿态：
* - roll（横滚）：左右翼高度差
* - pitch（俯仰）：设为0（双电机版本无前后电机）
* - lift_balance（升力平衡）：2电机位置的方差
*
* ## 输出
* 更新全局变量 attitude 结构体
*
* ## 用途
* 主要用于调试和未来扩展（如自动平衡控制）
************************************************************************************************
**/
void Estimate_Attitude(void)
{
    int32_t sum = 0;
    // 双电机版本：只累加 M1[0] 和 M3[2]
    sum += Wings_Data.Wings_motor[0].Corrective_Angle;
    sum += Wings_Data.Wings_motor[2].Corrective_Angle;
    attitude.avg_position = (int16_t)(sum / 2);  // 双电机版本：2个电机的平均

    // 计算同步误差（相对于平均值）
    attitude.sync_error[0] = Wings_Data.Wings_motor[0].Corrective_Angle - attitude.avg_position;
    attitude.sync_error[2] = Wings_Data.Wings_motor[2].Corrective_Angle - attitude.avg_position;

    // 计算横滚角（左翼 - 右翼）
    int16_t left_avg = Wings_Data.Wings_motor[2].Corrective_Angle;   // M3(左前)
    int16_t right_avg = Wings_Data.Wings_motor[0].Corrective_Angle;  // M1(右前)
    attitude.roll = left_avg - right_avg;

    // 双电机版本：pitch 无意义（已删除前后电机）
    attitude.pitch = 0;

    // 计算升力平衡（方差）
    int32_t variance = 0;
    variance += attitude.sync_error[0] * attitude.sync_error[0];
    variance += attitude.sync_error[2] * attitude.sync_error[2];
    attitude.lift_balance = (int16_t)(variance / 2);
}

/**
************************************************************************************************
* @brief    电机同步补偿 - 使2电机保持同步（双电机版本）
*
* ## 背景
* 由于制造公差、安装误差、PID参数差异等原因，
* 2个电机的实际位置会逐渐出现偏差。
* 如果偏差过大，会导致：
* - 飞行不稳定（一侧升力大，另一侧小）
* - 偏航（向一侧倾斜）
* - 机械应力不均匀
*
* ## 思路
* 定期检查2个电机与平均位置的偏差（sync_error），
* 如果某个电机偏离太远（>30编码器值），
* 则微调其目标角度，使其趋向平均值。
*
* ## 参数说明
* SYNC_GAIN = 3: 同步增益（V6.5降低：5→3，防止过调导致不平衡）
*   - 太小（如2）：同步太慢，效果不明显
*   - 太大（如10）：可能引起振荡
*   - 推荐：3-5（V6.5推荐偏保守的3）
*
* THRESHOLD = 30: 同步阈值（编码器值）
*   - 太小（如10）：频繁干预，影响正常扑动
*   - 太大（如100）：几乎不同步，失去作用
*   - 推荐：30-50
************************************************************************************************
**/
void Motor_Sync_Compensate(void)
{
    int16_t max_diff = 0;
    // 双电机版本：只检查 M1[0] 和 M3[2]
    for (uint8_t i = 0; i < 4; i += 2) {  // 步长=2，遍历索引0和2
        int16_t abs_diff = abs16_fast(attitude.sync_error[i]);
        if (abs_diff > max_diff) {
            max_diff = abs_diff;
        }
    }

    if (max_diff > 30) {
        const int16_t SYNC_GAIN = 3;  // V6.5降低：5→3，防止过调
        // 双电机版本：只补偿 M1[0] 和 M3[2]
        for (uint8_t i = 0; i < 4; i += 2) {  // 步长=2，遍历索引0和2
            int16_t compensation = (attitude.sync_error[i] * SYNC_GAIN) / 10;
            Wings_Data.Wings_motor[i].Target_Angle -= compensation;
        }
    }
}

/**
************************************************************************************************
* @brief    动态转向计算 - 实现小半径转向（优化版 - 带三重滤波）⭐V6.4优化
*
* ## 背景
* 原版问题：参数瞬间跳变 → 目标角度突变 → 机械冲击
* 
* ## 优化思路
* 1. 输入信号滤波：消除遥控器抖动
* 2. 自适应滤波系数：小转向时更平滑，大转向时响应快
* 3. 参数级滤波：消除计算跳变
*
* ## 参数说明
* @param yaw_input: Yaw通道输入值（-100 ~ +100，来自elrs_data.Yaw）
*                   -100 = 最大左转
*                    0  = 直飞
*                  +100 = 最大右转
* @param turn_data: 输出参数结构体（TurnControl_t类型）
*
* ## 转向曲线
* 使用二次曲线（input²）而非线性映射，使：
* - 小幅摇杆移动 → 轻微转向（精细控制）
* - 大幅摇杆移动 → 急转弯（快速响应）
************************************************************************************************
**/
void Calculate_Dynamic_Turn(int16_t yaw_input, TurnControl_t* turn_data)
{
    // ====== 静态变量保存历史状态 ======
    static int16_t last_turn_ratio = 0;
    static int16_t last_phase_diff = 0;
    static int16_t last_bias_angle = 0;
    static int16_t last_yaw_input = 0;
    
    // ====== 第一步：输入信号预处理 ======
    // 对yaw输入本身进行滤波，消除遥控器抖动
    int16_t smooth_yaw = (last_yaw_input * 7 + yaw_input) / 8;  // α=0.125
    last_yaw_input = smooth_yaw;
    
    // ====== 第二步：计算原始参数 ======
    turn_data->base_amp = 800;  // 基础振幅
    
    float input_norm = (float)constrain(smooth_yaw, -100, 100) / 100.0f;
    float turn_curve = input_norm * fabsf(input_norm);  // 二次曲线：x²
    
    int16_t raw_turn_ratio = (int16_t)(turn_curve * 100);     // 范围：-100~+100
    int16_t raw_phase_diff = (int16_t)(fabsf(turn_curve) * 3); // V6.4优化：增大到3（范围：0~3）
    int16_t raw_bias_angle = (int16_t)(turn_curve * 40);       // V6.4优化：增大到40（范围：-40~+40）
    
    // ====== 第三步：自适应滤波系数 ======
    // 小转向时：强滤波（α=0.1），追求平滑
    // 大转向时：弱滤波（α=0.4），追求响应
    float abs_input = fabsf(input_norm);
    float alpha = 0.1f + abs_input * 0.3f;  // α ∈ [0.1, 0.4]
    
    // ====== 第四步：应用一阶低通滤波 ======
    // 公式：output = α × new + (1-α) × old
    turn_data->turn_ratio = (int16_t)(alpha * raw_turn_ratio + (1-alpha) * last_turn_ratio);
    turn_data->phase_diff = (int16_t)(alpha * raw_phase_diff + (1-alpha) * last_phase_diff);
    turn_data->bias_angle = (int16_t)(alpha * raw_bias_angle + (1-alpha) * last_bias_angle);
    
    // ====== 第五步：保存当前值供下次使用 ======
    last_turn_ratio = turn_data->turn_ratio;
    last_phase_diff = turn_data->phase_diff;
    last_bias_angle = turn_data->bias_angle;
}

/**
 * @brief 兼容旧接口的步进时间计算
 * 内部调用Calculate_Biomimetic_Step_Time()
 */
uint32_t Calculate_Flap_Step_Time(uint8_t throttle)
{
    // V6.6修复：is_downstroke判断逻辑
    // 波形表：索引0-13是下拍区域，索引14-24是上挥区域
    // sm_idx<=13是下拍（需要快速），sm_idx>13是上挥（需要慢速）
    uint8_t is_downstroke = (sm_idx <= 13);
    return Calculate_Biomimetic_Step_Time(throttle, is_downstroke);
}

/**
************************************************************************************************
* @brief    执行仿生扑动步进 - 更新电机目标角度（核心优化版）⭐⭐⭐最重要的函数
*
* ## 功能
* 这是整个扑动系统的核心执行函数，每调用一次就推进一个步进：
* 1. 从BIOMIMETIC_WAVE[]读取当前波形值
* 2. 计算动态振幅（考虑转向差动和频率补偿）
* 3. 计算相位偏移（用于转向时的左右翼异步）
* 4. 计算每个电机的目标角度
* 5. 应用极限位置缓冲
* 6. 更新状态机索引和方向
*
* ## 调用时机
* 由main.c的主循环调用，通过非阻塞定时器控制调用频率：
* if (HAL_GetTick() >= sm_next_tick) {
*     Execute_Flap_Step(...);
*     sm_next_tick += step_ms;
* }
*
* ## 参数说明
* @param yaw_input: 转向输入（-100 ~ +100）
* @param turn_ctrl: 转向控制参数（由Calculate_Dynamic_Turn()生成）
* @param motor_M3_ready: M3(左前)电机基准位置（中点+微调）
* @param motor_M1_ready: M1(右前)电机基准位置
*
* ⚠️ 双电机版本：只使用 M1(右前) 和 M3(左前) 两个电机
*
* ## 关键可调常量（在本函数内定义）
* MIN_AMP = 600: 最小振幅（防止扑动幅度过小卡死）
* MAX_AMP = 1100: 最大振幅（保护齿轮箱，不要超过1200！）
*
* ## 状态机逻辑
* 正向下拍（sm_dir=+1）：sm_idx: 0 → 1 → 2 → ... → 24 → 反转
* 反向上挥（sm_dir=-1）：sm_idx: 24 → 23 → 22 → ... → 0 → 反转
* 到达边界时切换方向，actual_flap_count++（完成半个周期）
************************************************************************************************
**/
void Execute_Flap_Step(int16_t yaw_input,
                       TurnControl_t* turn_ctrl,
                       int16_t motor_M3_ready,
                       int16_t motor_M1_ready)
{
    int16_t wave_value = BIOMIMETIC_WAVE[sm_idx];
    
    int16_t amp_diff = (turn_ctrl->turn_ratio * 25) / 10;
    int16_t ampR_base = turn_ctrl->base_amp - amp_diff;
    int16_t ampL_base = turn_ctrl->base_amp + amp_diff;
    
    float current_freq = 1000.0f / (Calculate_Biomimetic_Step_Time(thr, 1) * 25);
    int16_t ampR = Apply_Amplitude_Compensation(ampR_base, current_freq);
    int16_t ampL = Apply_Amplitude_Compensation(ampL_base, current_freq);
    
    // V6.4优化：增大振幅范围，支持100度扑动
    // 100度 ≈ 1138编码器单位
    const int16_t MIN_AMP = 700;   // V6.4: 从600→700，提高最小振幅
    const int16_t MAX_AMP = 1200;  // V6.4: 从1100→1200，支持100度振幅（约105度）
    ampR = constrain(ampR, MIN_AMP, MAX_AMP);
    ampL = constrain(ampL, MIN_AMP, MAX_AMP);
    
    int8_t phase_offset = (yaw_input < 0) ? -turn_ctrl->phase_diff : turn_ctrl->phase_diff;
    
    int8_t idxR = sm_idx;
    int8_t idxL = sm_idx + phase_offset;
    
    if (idxL < 0) idxL = 0;
    if (idxL > 24) idxL = 24;
    
    int16_t cR = BIOMIMETIC_WAVE[idxR];
    int16_t cL = BIOMIMETIC_WAVE[idxL];
    
    int16_t biasR = -turn_ctrl->bias_angle;
    int16_t biasL = turn_ctrl->bias_angle;
    
    int16_t raw_target_R = (int16_t)(motor_M1_ready + biasR + FLAP_CENTER_OFFSET - q15_mul(ampR, cR));
    int16_t raw_target_L = (int16_t)(motor_M3_ready + biasL + FLAP_CENTER_OFFSET - q15_mul(ampL, cL));
    
    Wings_Data.Wings_motor[0].Target_Angle = Apply_Limit_Buffer(raw_target_R, 0);
    Wings_Data.Wings_motor[2].Target_Angle = Apply_Limit_Buffer(raw_target_L, 2);
    
    // 【调试】同步关键变量到全局，用于STM-Studio观察
    #ifdef DEBUG_MODE
    extern volatile int16_t dbg_wave_value;
    extern volatile int16_t dbg_q15_result;
    extern volatile int8_t  dbg_current_idx;
    dbg_wave_value = cL;                           // M3使用的波形值
    dbg_q15_result = q15_mul(ampL, cL);            // q15乘法结果
    dbg_current_idx = (int8_t)sm_idx;              // 当前索引（递增前）
    #endif
    
    // V6.7修复：完全重写波形表扫描逻辑
    // 波形表结构：
    //   索引0-12:  下拍区域（从上极限到下极限）- 快速
    //   索引13:    下极限缓冲点
    //   索引14-24: 上挥区域（从下极限回到上极限）- 慢速
    // 
    // 扫描逻辑：
    //   下拍：正向扫描 0 → 13（14步，快速）
    //   上挥：正向扫描 14 → 24（11步，慢速）
    //   回到起点：从索引24跳回索引0
    
    // 使用后置递增，确保每个索引都被访问
    sm_idx++;
    
    if (sm_idx > 24) { 
        sm_idx = 0;  // 到达上极限，跳回起点
        actual_flap_count++;  // 完成一个完整周期
    }
}
