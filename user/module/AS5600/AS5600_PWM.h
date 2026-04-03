#ifndef __AS5600_PWM_H__
#define __AS5600_PWM_H__
#include "main.h"
#include "adc.h"

// 调试模式开关 - 在main.c中定义，这里只是外部声明
#ifndef DEBUG_MODE
// 如果main.c中没有定义，这里也不定义
#else
// 调试模式已启用
#endif

#define MAX_VALUE 4095  // 输入范围 0~4095，所以最大值为 4096

// 各电机1024点位定义（新编码器的原始中点值）
#define MOTOR1_MIDPOINT 1508 // 电机0（右前）→ M1通道
#define MOTOR2_MIDPOINT 577	// 电机1（左后）→ M4通道
#define MOTOR3_MIDPOINT 3172   // 电机2（左前）→ M3通道
#define MOTOR4_MIDPOINT 3580 // 电机3（右后）→ M2通道

// 编码器方向反转标志（1=反转，0=正常）
// 电机2（左前）和电机3（右后）的编码器方向与其他电机相反
#define MOTOR1_REVERSE 0  // 电机0（右前）- 正常
#define MOTOR2_REVERSE 0  // 电机1（左后）- 正常
#define MOTOR3_REVERSE 1  // 电机2（左前）- 反转（逆时针转时编码器值减小）
#define MOTOR4_REVERSE 1  // 电机3（右后）- 反转（顺时针转时编码器值减小）

// PROCESS_VALUE宏：将原始编码器值转换为相对角度值
// 原理：将原始值相对于零点(zero)进行偏移，使zero位置对应1024（垂直位置）
// 公式：result = (raw - zero + 1024) mod 4096
// 其中1024对应90°（垂直位置），0对应0°，2048对应180°，3072对应270°
#define PROCESS_VALUE(raw, zero) (((raw) + 4096u - (((zero) + 3072u) & 0x0FFFu)) & 0x0FFFu)



extern void StarAndGetResult(void);

// ADC原始值（DMA读取结果）
// 重要：DMA配置为16位传输模式，所以使用uint16_t数组
extern uint16_t AD_Value[4];

// 调试变量 - 用于排查ADC问题（仅在DEBUG_MODE定义时启用）
#ifdef DEBUG_MODE
extern volatile uint8_t adc_init_flag;      // ADC初始化标志
extern volatile uint8_t adc_start_count;    // ADC启动次数计数
extern volatile uint8_t adc_error_count;    // ADC错误次数计数
extern volatile uint32_t last_adc_value;    // 最后一次ADC读取值
#endif

#endif
