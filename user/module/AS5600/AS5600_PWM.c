#include "AS5600_PWM.h"
#include "tim.h"
#include "motor.h"

#define ALPHA_FLOAT 0.4f
#define ONE_MINUS_ALPHA_FLOAT (1.0f - ALPHA_FLOAT)

#define LIMIT_THRESHOLD 150

static float filtered_value_float[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint16_t last_raw_value[4] = {0};

/* MT6701?TIM1 CH1~CH4 ?????ISR ??? 0~4095 */
static volatile uint16_t mt6701_raw[4] = {2048u, 2048u, 2048u, 2048u};
static uint16_t mt6701_last_rise[4];
static uint16_t mt6701_last_high[4];
static uint8_t mt6701_have_high[4];

static const uint16_t mt6701_gpio_pin[4] = {
  MT6701_PIN_CH1,
  MT6701_PIN_CH2,
  MT6701_PIN_CH3,
  MT6701_PIN_CH4,
};

static uint32_t tim_diff16(uint16_t later, uint16_t earlier)
{
  return (uint32_t)(uint16_t)(later - earlier);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t idx;
  uint16_t ccr;
  uint16_t pin;

  if (htim->Instance != TIM1)
  {
    return;
  }

  switch (HAL_TIM_GetActiveChannel(htim))
  {
#if !MOTOR_HW_USE_TIM1_IC_CH34
    case HAL_TIM_ACTIVE_CHANNEL_3:
    case HAL_TIM_ACTIVE_CHANNEL_4:
      return;
#endif
    case HAL_TIM_ACTIVE_CHANNEL_1:
      idx = 0;
      ccr = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_2:
      idx = 1;
      ccr = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_3:
      idx = 2;
      ccr = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
      idx = 3;
      ccr = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
      break;
    default:
      return;
  }

  pin = mt6701_gpio_pin[idx];

  if (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_SET)
  {
    uint32_t period = tim_diff16(ccr, mt6701_last_rise[idx]);
    mt6701_last_rise[idx] = ccr;
    if (period > 50u && period < 200000u && mt6701_have_high[idx])
    {
      uint32_t ang = (uint32_t)mt6701_last_high[idx] * 4096u / period;
      if (ang > 4095u)
      {
        ang = 4095u;
      }
      mt6701_raw[idx] = (uint16_t)ang;
      mt6701_have_high[idx] = 0;
    }
  }
  else
  {
    mt6701_last_high[idx] = (uint16_t)tim_diff16(ccr, mt6701_last_rise[idx]);
    mt6701_have_high[idx] = 1;
  }
}

uint16_t limit_and_lowpass_filter(uint16_t new_value, uint8_t channel)
{
	if (filtered_value_float[channel] == 0.0f && last_raw_value[channel] == 0) {
		filtered_value_float[channel] = (float)new_value;
		last_raw_value[channel] = new_value;
		return new_value;
	}
	
	int16_t diff = (int16_t)new_value - (int16_t)last_raw_value[channel];
	uint16_t limited_value;
	
	if (diff > LIMIT_THRESHOLD || diff < -LIMIT_THRESHOLD) {
		limited_value = last_raw_value[channel];
	} else {
		limited_value = new_value;
	}
	
	last_raw_value[channel] = limited_value;
	
	filtered_value_float[channel] = ALPHA_FLOAT * (float)limited_value + 
	                                ONE_MINUS_ALPHA_FLOAT * filtered_value_float[channel];
	
	return (uint16_t)(filtered_value_float[channel] + 0.5f);
}

void StarAndGetResult(void)
{
	uint16_t filtered_ad[4];
//+++260407:jhb:对 mt6701_raw原始值进行限幅和低通滤波处理
	filtered_ad[0] = limit_and_lowpass_filter(mt6701_raw[0], 0);
	filtered_ad[1] = limit_and_lowpass_filter(mt6701_raw[1], 1);
	filtered_ad[2] = limit_and_lowpass_filter(mt6701_raw[2], 2);
	filtered_ad[3] = limit_and_lowpass_filter(mt6701_raw[3], 3);

	Wings_Data.Wings_motor[0].Corrective_Angle = PROCESS_VALUE(filtered_ad[2], MOTOR3_MIDPOINT);
	Wings_Data.Wings_motor[1].Corrective_Angle = PROCESS_VALUE(filtered_ad[1], MOTOR2_MIDPOINT);
	Wings_Data.Wings_motor[2].Corrective_Angle = PROCESS_VALUE(filtered_ad[0], MOTOR1_MIDPOINT);
	Wings_Data.Wings_motor[3].Corrective_Angle = PROCESS_VALUE(filtered_ad[3], MOTOR4_MIDPOINT);
}
