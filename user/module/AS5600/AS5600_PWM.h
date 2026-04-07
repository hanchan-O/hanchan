#ifndef __AS5600_PWM_H__
#define __AS5600_PWM_H__
#include "main.h"

#define MAX_VALUE 4095
//+++260407:hww:电机中点角度配置MT6701 PWM 编码器
#define MOTOR1_MIDPOINT 954
#define MOTOR2_MIDPOINT 134
#define MOTOR3_MIDPOINT 2322
#define MOTOR4_MIDPOINT 2504

#define MOTOR1_REVERSE 0
#define MOTOR2_REVERSE 0
#define MOTOR3_REVERSE 1
#define MOTOR4_REVERSE 1

#define PROCESS_VALUE(raw, zero) (((raw) + 4096u - (((zero) + 3072u) & 0x0FFFu)) & 0x0FFFu)

extern void StarAndGetResult(void);

#endif
