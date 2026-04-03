#include "AS5600_PWM.h"
#include "motor.h"

/**
************************************************************************************************
* @brief    AS5600_PWM角度读取（原始版本 - 稳定可靠）
* @param    None
* @return   None
* @author   hanchan 2026.01.30
************************************************************************************************
**/
// #define DEBUG_MODE

// ========== 滤波参数配置 ==========
// 一阶低通滤波系数（浮点格式）
#define ALPHA_FLOAT 0.4f
#define ONE_MINUS_ALPHA_FLOAT (1.0f - ALPHA_FLOAT)

// ADC采样间隔（ms）
#define ADC_INTERVAL_MS 2

// 限幅滤波阈值
#define LIMIT_THRESHOLD 150

// ========== 全局变量 ==========
// 一阶低通滤波历史值（浮点格式）
static float filtered_value_float[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// 上次ADC采样值（用于限幅滤波）
static uint16_t last_raw_value[4] = {0};

// 上次ADC采样时间戳
static uint32_t last_adc_time = 0;

// ADC原始值（DMA读取结果）
// 重要：DMA配置为16位传输模式，所以使用uint16_t数组
uint16_t AD_Value[4];

// ADC状态
HAL_StatusTypeDef hhStatue;

// 调试变量
#ifdef DEBUG_MODE
volatile uint8_t adc_init_flag = 0;
volatile uint8_t adc_start_count = 0;
volatile uint8_t adc_error_count = 0;
volatile uint32_t last_adc_value = 0;
#endif

/**
************************************************************************************************
* @brief    限幅滤波 + 一阶低通滤波组合
************************************************************************************************
**/
uint16_t limit_and_lowpass_filter(uint16_t new_value, uint8_t channel)
{
	// 第一次初始化
	if (filtered_value_float[channel] == 0.0f && last_raw_value[channel] == 0) {
		filtered_value_float[channel] = (float)new_value;
		last_raw_value[channel] = new_value;
		return new_value;
	}
	
	// 步骤1: 限幅滤波
	int16_t diff = (int16_t)new_value - (int16_t)last_raw_value[channel];
	uint16_t limited_value;
	
	if (diff > LIMIT_THRESHOLD || diff < -LIMIT_THRESHOLD) {
		limited_value = last_raw_value[channel];
	} else {
		limited_value = new_value;
	}
	
	last_raw_value[channel] = limited_value;
	
	// 步骤2: 一阶低通滤波（浮点运算）
	filtered_value_float[channel] = ALPHA_FLOAT * (float)limited_value + 
	                                ONE_MINUS_ALPHA_FLOAT * filtered_value_float[channel];
	
	return (uint16_t)(filtered_value_float[channel] + 0.5f);
}

/**
************************************************************************************************
* @brief    执行ADC采样
************************************************************************************************
**/
void ADC_Sample(uint32_t current_time)
{
	if ((int32_t)(current_time - last_adc_time) >= ADC_INTERVAL_MS) {
		
		#ifdef DEBUG_MODE
		if (adc_init_flag == 0) {
			adc_init_flag = 1;
		}
		#endif
		
		// 启动ADC+DMA转换
		// 注意：DMA配置为16位模式，AD_Value是uint16_t数组
		hhStatue = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AD_Value, 4);
		
		#ifdef DEBUG_MODE
		adc_start_count++;
		#endif
		
		if (hhStatue != HAL_OK) {
			// ADC启动失败，使用历史值
			AD_Value[0] = last_raw_value[0];
			AD_Value[1] = last_raw_value[1];
			AD_Value[2] = last_raw_value[2];
			AD_Value[3] = last_raw_value[3];
			#ifdef DEBUG_MODE
			adc_error_count++;
			#endif
		} else {
			// 等待DMA传输完成
			uint32_t timeout = 1000;
			while (hdma_adc1.State != HAL_DMA_STATE_READY && timeout > 0) {
				timeout--;
			}
			
			// 停止ADC
			HAL_ADC_Stop_DMA(&hadc1);
			
			#ifdef DEBUG_MODE
			last_adc_value = ((uint32_t)AD_Value[0] << 24) | 
			                  ((uint32_t)AD_Value[1] << 16) | 
			                  ((uint32_t)AD_Value[2] << 8)  | 
			                  (uint32_t)AD_Value[3];
			#endif
		}
		
		last_adc_time = current_time;
	}
}

/**
************************************************************************************************
* @brief    获取角度结果
************************************************************************************************
**/
void StarAndGetResult(void){
	uint32_t current_time = HAL_GetTick();
	
	ADC_Sample(current_time);
	
	// 应用限幅+一阶低通滤波
	// AD_Value是uint16_t数组，直接传入
	uint16_t filtered_ad[4];
	filtered_ad[0] = limit_and_lowpass_filter(AD_Value[0], 0);
	filtered_ad[1] = limit_and_lowpass_filter(AD_Value[1], 1);
	filtered_ad[2] = limit_and_lowpass_filter(AD_Value[2], 2);
	filtered_ad[3] = limit_and_lowpass_filter(AD_Value[3], 3);
	
	// 使用滤波后的值计算角度
	// 对于编码器方向反转的电机，同时反转原始值和中点值
	// 电机0（右前）- 正常
	Wings_Data.Wings_motor[0].Corrective_Angle = PROCESS_VALUE(filtered_ad[0], MOTOR1_MIDPOINT);
	// 电机1（左后）- 正常
	Wings_Data.Wings_motor[1].Corrective_Angle = PROCESS_VALUE(filtered_ad[1], MOTOR2_MIDPOINT);
	// 电机2（左前）- 反转：原始值和中点值都反转
	Wings_Data.Wings_motor[2].Corrective_Angle = PROCESS_VALUE(
		(4096u - filtered_ad[2]), 
		(4096u - MOTOR3_MIDPOINT));
	// 电机3（右后）- 反转：原始值和中点值都反转
	Wings_Data.Wings_motor[3].Corrective_Angle = PROCESS_VALUE(
		(4096u - filtered_ad[3]), 
		(4096u - MOTOR4_MIDPOINT));
}
