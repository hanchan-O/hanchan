/**
 * @file    AS5600_PWM.c
 * @brief   AS5600磁编码器角度采集模块（带滤波）
 * @author  hanchan
 * @date    2026-04-10
 *
 * ## 模块功能
 * 本模块负责采集4个AS5600磁编码器的角度信息，经过双重滤波处理后，
 * 转换为相对角度值供PID控制器使用。
 *
 * ## 信号链路
 * AS5600传感器 → PWM输出 → ADC采样(500Hz) → 限幅滤波 → 低通滤波 → 角度转换
 *
 * ## 关键参数
 * - 采样频率：500Hz（每2ms一次）
 * - 滤波方式：限幅滤波 + 一阶低通滤波组合
 * - 输出范围：0-4095（12位精度）
 */

#include "AS5600_PWM.h"
#include "motor.h"

// ==================== 【重要】可调参数配置区 ====================
// 🔧 调试时主要修改这里的参数！

/**
 * @brief 一阶低通滤波系数（浮点格式）
 * 
 * 作用：控制滤波强度，决定新旧数据的权重
 * 公式：output = α × new_value + (1-α) × old_value
 * 
 * 可调范围：0.1 ~ 0.7
 * 当前值：0.4（平衡响应速度和噪声抑制）
 * 建议：
 *   - 数据太抖动（噪声大）→ 增大到0.5-0.6（更平滑）
 *   - 响应太慢（延迟大）→ 减小到0.2-0.3（更快跟踪）
 *   - 一般情况保持0.4即可
 */
#define ALPHA_FLOAT 0.4f

/** @brief 预计算：(1 - ALPHA_FLOAT)，用于优化计算速度 */
#define ONE_MINUS_ALPHA_FLOAT (1.0f - ALPHA_FLOAT)

/**
 * @brief ADC采样间隔（毫秒）
 * 
 * 作用：控制ADC采样频率 = 1000 / ADC_INTERVAL_MS
 * 
 * 可调范围：1 ~ 5
 * 当前值：2ms（对应500Hz采样率）
 * 建议：
 *   - 需要更高精度 → 减小到1ms（1000Hz，但CPU负载增加）
 *   - CPU资源紧张 → 增大到3-4ms（250-333Hz，仍足够扑动控制）
 *   - 不要超过5ms（200Hz可能影响控制性能）
 */
#define ADC_INTERVAL_MS 2

/**
 * @brief 限幅滤波阈值（编码器单位）
 * 
 * 作用：防止ADC读数突变（可能是干扰或接触不良）
 * 如果两次采样的差值超过此阈值，则丢弃新值，使用旧值
 * 
 * ⚠️ 注意：这个值不能太小！电机正常运动时每2ms的变化量：
 *   - 低速（<3Hz）：约20-50编码器值/2ms
 *   - 高速（>5Hz）：约80-120编码器值/2ms
 *   所以150是一个安全的阈值
 * 
 * 可调范围：80 ~ 300
 * 当前值：150（推荐值）
 * 建议：
 *   - 有明显噪声尖峰 → 减小到100（更严格过滤）
 *   - 高频时数据丢失 → 增大到200-250（允许更快变化）
 */
#define LIMIT_THRESHOLD 150

// ==================== 全局变量定义 ====================

/**
 * @brief 一阶低通滤波历史值（每个通道独立）
 * 存储上一次的滤波结果，用于递推计算
 */
static float filtered_value_float[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/**
 * @brief 上次ADC原始值（用于限幅滤波判断）
 * 与新值比较，如果差值过大则判定为异常
 */
static uint16_t last_raw_value[4] = {0};

/**
 * @brief 上次ADC采样时间戳（毫秒）
 * 用于实现非阻塞定时采样
 */
static uint32_t last_adc_time = 0;

/**
 * @brief ADC原始值数组（DMA直接写入）
 * 
 * 重要说明：
 * - 这是DMA的目标地址，ADC转换完成后自动填充
 * - DMA配置为16位传输模式（uint16_t）
 * - 索引对应关系：
 *   [0] = 电机0（右前）的编码器
 *   [1] = 电机1（左后）的编码器
 *   [2] = 电机2（左前）的编码器
 *   [3] = 电机3（右后）的编码器
 * 
 * 数值范围：0 ~ 4095（对应0° ~ 360°）
 */
uint16_t AD_Value[4];

/** @brief ADC操作状态（调试用） */
HAL_StatusTypeDef hhStatue;

// ========== 调试模式开关 ==========
// 取消注释下面的宏定义可以启用调试输出
// #define DEBUG_MODE

#ifdef DEBUG_MODE
volatile uint8_t adc_init_flag = 0;      // ADC初始化标志
volatile uint8_t adc_start_count = 0;     // ADC启动次数统计
volatile uint8_t adc_error_count = 0;     // ADC错误次数统计
volatile uint32_t last_adc_value = 0;     // 上次ADC原始值（打包存储）
#endif

/**
************************************************************************************************
* @brief    限幅滤波 + 一阶低通滤波组合 ⭐核心滤波算法
*
* ## 背景
* ADC采样容易受到电磁干扰、电源纹波等影响，产生突变噪声。
* 单纯的低通滤波无法有效处理突发的大幅度干扰，
* 因此采用"限幅+低通"的双重滤波策略。
*
* ## 思路
* 第一层：限幅滤波（门控）
*   - 检测本次采样与上次采样的差值
*   - 如果差值 > 阈值 → 判定为干扰，丢弃新值
*   - 如果差值 ≤ 阈值 → 判定为正常变化，接受新值
*
* 第二层：一阶低通滤波（平滑）
*   - 对通过第一层的数据进行加权平均
*   - 新数据权重α，历史数据权重(1-α)
*   - 有效抑制高频噪声
*
* ## 步骤
* 1. 首次调用时初始化（直接返回新值）
* 2. 计算差值并应用限幅滤波
* 3. 应用一阶低通滤波公式
* 4. 返回四舍五入后的整数值
*
* ## 参数说明
* @param new_value: 本次ADC采样的原始值（0~4095）
* @param channel: 通道号（0~3，对应4个电机）
*
* @return 滤波后的角度值（0~4095）
*
* ## 示例
* channel=0, 上次值=2000, 新值=2100, 阈值=150:
*   差值 = 2100-2000 = 100 < 150 ✓ 接受
*   滤波结果 = 0.4×2100 + 0.6×2000 = 2040
*
* channel=0, 上次值=2000, 新值=2200, 阈值=150:
*   差值 = 2200-2000 = 200 > 150 ✗ 丢弃
*   使用上次值2000进行滤波
************************************************************************************************
**/
uint16_t limit_and_lowpass_filter(uint16_t new_value, uint8_t channel)
{
	// 首次初始化：直接赋值返回（避免启动时的异常数据）
	if (filtered_value_float[channel] == 0.0f && last_raw_value[channel] == 0) {
		filtered_value_float[channel] = (float)new_value;
		last_raw_value[channel] = new_value;
		return new_value;
	}
	
	// ====== 第一层：限幅滤波 ======
	int16_t diff = (int16_t)new_value - (int16_t)last_raw_value[channel];
	uint16_t limited_value;
	
	if (diff > LIMIT_THRESHOLD || diff < -LIMIT_THRESHOLD) {
		// 差值超阈值：判定为干扰，保留旧值
		limited_value = last_raw_value[channel];
	} else {
		// 差值在范围内：正常变化，接受新值
		limited_value = new_value;
	}
	
	// 更新历史值（无论是否接受都更新，避免持续触发限幅）
	last_raw_value[channel] = limited_value;
	
	// ====== 第二层：一阶低通滤波 ======
	filtered_value_float[channel] = ALPHA_FLOAT * (float)limited_value + 
	                                ONE_MINUS_ALPHA_FLOAT * filtered_value_float[channel];
	
	return (uint16_t)(filtered_value_float[channel] + 0.5f);  // 四舍五入
}

/**
************************************************************************************************
* @brief    执行ADC采样（非阻塞定时版本）
*
* ## 功能
* 通过时间戳判断是否到达采样时刻，如果到了就启动ADC+DMA转换。
* 采用非阻塞设计，不会占用过多CPU时间。
*
* ## 采样流程
* 1. 检查距离上次采样是否已经过ADC_INTERVAL_MS毫秒
* 2. 启动ADC1的DMA模式转换（4通道连续采样）
* 3. 等待DMA传输完成（带超时保护）
* 4. 停止ADC和DMA，释放总线
* 5. 更新时间戳
*
* ## 错误处理
* 如果ADC启动失败或DMA超时：
* - 使用上次的有效值填充AD_Value[]数组
* - 记录错误次数（DEBUG_MODE下）
* - 继续运行，不中断系统
*
* ## 参数说明
* @param current_time: 当前系统时间戳（毫秒，来自HAL_GetTick()）
************************************************************************************************
**/
void ADC_Sample(uint32_t current_time)
{
	// 时间检查：是否到达采样时刻？
	if ((int32_t)(current_time - last_adc_time) >= ADC_INTERVAL_MS) {
		
		#ifdef DEBUG_MODE
		if (adc_init_flag == 0) {
			adc_init_flag = 1;  // 标记首次采样完成
		}
		#endif
		
		// 启动ADC+DMA转换
		// hadc1: ADC1句柄（在main.c中初始化）
		// AD_Value: DMA目标地址（uint16_t[4]数组）
		// 4: 采样通道数量
		hhStatue = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AD_Value, 4);
		
		#ifdef DEBUG_MODE
		adc_start_count++;  // 统计采样次数
		#endif
		
		if (hhStatue != HAL_OK) {
			// ❌ ADC启动失败（可能是DMA忙或硬件错误）
			// 使用历史值兜底，保证系统继续运行
			AD_Value[0] = last_raw_value[0];
			AD_Value[1] = last_raw_value[1];
			AD_Value[2] = last_raw_value[2];
			AD_Value[3] = last_raw_value[3];
			#ifdef DEBUG_MODE
			adc_error_count++;  // 记录错误
			#endif
		} else {
			// ✅ ADC启动成功，等待DMA传输完成
			uint32_t timeout = 1000;  // 超时计数器（防止死循环）
			
			// 轮询等待DMA状态变为READY（表示传输完成）
			while (hdma_adc1.State != HAL_DMA_STATE_READY && timeout > 0) {
				timeout--;
			}
			
			// 停止ADC和DMA（释放资源给其他外设使用）
			HAL_ADC_Stop_DMA(&hadc1);
			
			#ifdef DEBUG_MODE
			// 将4个通道值打包到一个变量中（方便调试观察）
			last_adc_value = ((uint32_t)AD_Value[0] << 24) | 
			                  ((uint32_t)AD_Value[1] << 16) | 
			                  ((uint32_t)AD_Value[2] << 8)  | 
			                  (uint32_t)AD_Value[3];
			#endif
		}
		
		// 更新时间戳（无论成功与否都更新，避免堆积采样请求）
		last_adc_time = current_time;
	}
}

/**
************************************************************************************************
* @brief    主函数入口 - 获取处理后的角度数据
*
* ## 功能
* 这是本模块对外暴露的主接口函数，由主循环调用。
* 完成以下工作：
* 1. 执行ADC采样（如果时间到）
* 2. 对4个通道分别应用双重滤波
* 3. 进行方向反转处理（针对安装方向相反的电机）
* 4. 转换为相对角度（以中点为基准）
* 5. 写入Wings_Data结构体供PID模块使用
*
* ## 方向反转说明
* 由于机械安装原因，部分电机的编码器方向与标准方向相反。
* 对于这些电机，需要同时反转原始值和中点值：
* 反转公式：corrected_value = 4096 - original_value
* 这样可以统一所有电机的坐标系。
*
* ## 电机映射
* AD_Value[0] → Wings_motor[0]（右前）→ 正常方向
* AD_Value[1] → Wings_motor[1]（左后）→ 正常方向
* AD_Value[2] → Wings_motor[2]（左前）→ 反转方向
* AD_Value[3] → Wings_motor[3]（右后）→ 反转方向
************************************************************************************************
**/
void StarAndGetResult(void){
	uint32_t current_time = HAL_GetTick();
	
	// 步骤1：执行ADC采样（内部有定时控制）
	ADC_Sample(current_time);
	
	// 步骤2：对4个通道分别应用限幅+低通滤波
	uint16_t filtered_ad[4];
	filtered_ad[0] = limit_and_lowpass_filter(AD_Value[0], 0);  // 右前
	filtered_ad[1] = limit_and_lowpass_filter(AD_Value[1], 1);  // 左后
	filtered_ad[2] = limit_and_lowpass_filter(AD_Value[2], 2);  // 左前
	filtered_ad[3] = limit_and_lowpass_filter(AD_Value[3], 3);  // 右后
	
	// 步骤3：角度转换（统一处理，无方向反转）
	//
	// ⚠️ 【V6.2重要】前提条件：所有4个磁编码器的磁铁安装方向必须一致！
	//   统一标准：顺时针转 → 编码器值增大 → 翅膀向下运动
	//   验证方法：手动把每个翅膀往下压，观察编码器读数应该变大
	//   如果某个编码器变小了 → 把那个编码器的磁铁旋转180°重新安装
	//
	// PROCESS_VALUE宏执行：relative_angle = (raw_value + 4096 - (midpoint + 3072)) & 0x0FFF

	// 电机0（右前）：PA0(ADC_CH0)
	Wings_Data.Wings_motor[0].Corrective_Angle = PROCESS_VALUE(filtered_ad[0], MOTOR1_MIDPOINT);

	// 电机1（左后）：PA1(ADC_CH1)
	Wings_Data.Wings_motor[1].Corrective_Angle = PROCESS_VALUE(filtered_ad[1], MOTOR2_MIDPOINT);

	// 电机2（左前）：PA6(ADC_CH6)
	Wings_Data.Wings_motor[2].Corrective_Angle = PROCESS_VALUE(filtered_ad[2], MOTOR3_MIDPOINT);

	// 电机3（右后）：PA7(ADC_CH7)
	Wings_Data.Wings_motor[3].Corrective_Angle = PROCESS_VALUE(filtered_ad[3], MOTOR4_MIDPOINT);
}
