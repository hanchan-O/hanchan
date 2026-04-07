#include "elrs.h"
#include "usart.h"
#include "motor.h"
#include <string.h>
/**
************************************************************************************************
* @brief    解析ELRS的CRSF协议数据
* @param    None
* @return   None
* @author   hanchan	2025.11.06
************************************************************************************************
**/
float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
    {
        return output_min;
    }

    if (input_value < median)
    {
        return float_Map(input_value, input_min, median, output_min, output_median);
    }
    else
    {
        return float_Map(input_value, median, input_max, output_median, output_max);
    }
}

/**
 * int16 类型通道值映射函数
 */
static inline int16_t int16_Map(uint16_t x,
                                uint16_t in_min, uint16_t in_max,
                                int16_t out_min, int16_t out_max)
{
    if (in_max <= in_min) return out_min;

    if (x < in_min) x = in_min;
    else if (x > in_max) x = in_max;

    int32_t num   = (int32_t)(x - in_min) * (out_max - out_min);
    int32_t denom = (int32_t)(in_max - in_min);

    return (int16_t)(out_min + num / denom);
}

/**
 * 基于中位数的 int16 类型映射
 */
static inline int16_t int16_Map_with_median(uint16_t x,
                                            uint16_t in_min, uint16_t in_max, uint16_t median,
                                            int16_t out_min, int16_t out_max)
{
    if (in_max <= in_min || median <= in_min || median >= in_max)
        return out_min;

    int16_t out_mid = (int16_t)(((int32_t)out_min + out_max) / 2);

    if (x < median)
        return int16_Map(x, in_min, median, out_min, out_mid);
    else
        return int16_Map(x, median, in_max, out_mid, out_max);
}

// ----------------------------------------------------------

/**
 * uint8 类型通道值映射函数
 */
static inline uint8_t int8_Map(uint16_t x,
                              uint16_t in_min, uint16_t in_max,
                              uint8_t out_min, uint8_t out_max)
{
    if (in_max <= in_min) return out_min;

    if (x < in_min) x = in_min;
    else if (x > in_max) x = in_max;

    int16_t num   = (int16_t)(x - in_min) * (out_max - out_min);
    int16_t denom = (int16_t)(in_max - in_min);

    return (uint8_t)(out_min + num / denom);
}

static inline uint8_t int8_Map_with_median(uint16_t x,
                                          uint16_t in_min, uint16_t in_max, uint16_t median,
                                          uint8_t out_min, uint8_t out_max)
{
    if (in_max <= in_min || median <= in_min || median >= in_max)
        return out_min;

    uint8_t out_mid = (uint8_t)(((int16_t)out_min + out_max) / 2);

    if (x < median)
        return int8_Map(x, in_min, median, out_min, out_mid);
    else
        return int8_Map(x, median, in_max, out_mid, out_max);
}

/* 当值小于阈值时返回 0 */
static inline int16_t apply_deadzone(int16_t value, int16_t threshold)
{
    if (value > -threshold && value < threshold) {
        return 0;
    }
    return value;
}
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t elrs_data_temp[36] = {0};

volatile uint32_t last_signal_time = 0;

void ELRS_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, elrs_data_temp, MAX_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

ELRS_Data elrs_data;
void ELRS_UARTE_RxCallback(uint16_t Size)
{
	uint8_t date_i =0;

    if (elrs_data_temp[date_i+0] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {

        if (0)
        {
        }
        else if (elrs_data_temp[date_i+2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            elrs_data.channels[0] = ((uint16_t)elrs_data_temp[date_i+3] >> 0 | ((uint16_t)elrs_data_temp[date_i+4] << 8)) & 0x07FF;
            elrs_data.channels[1] = ((uint16_t)elrs_data_temp[date_i+4] >> 3 | ((uint16_t)elrs_data_temp[date_i+5] << 5)) & 0x07FF;
            elrs_data.channels[2] = ((uint16_t)elrs_data_temp[date_i+5] >> 6 | ((uint16_t)elrs_data_temp[date_i+6] << 2) | ((uint16_t)elrs_data_temp[date_i+7] << 10)) & 0x07FF;
            elrs_data.channels[3] = ((uint16_t)elrs_data_temp[date_i+7] >> 1 | ((uint16_t)elrs_data_temp[date_i+8] << 7)) & 0x07FF;
            elrs_data.channels[4] = ((uint16_t)elrs_data_temp[date_i+8] >> 4 | ((uint16_t)elrs_data_temp[date_i+9] << 4)) & 0x07FF;
            elrs_data.channels[5] = ((uint16_t)elrs_data_temp[date_i+9] >> 7 | ((uint16_t)elrs_data_temp[date_i+10] << 1) | ((uint16_t)elrs_data_temp[date_i+11] << 9)) & 0x07FF;
            /* 解析channels[0]=Yaw?[1]=Pitch?[2]=Throttle?[3]=Roll?[4]=Switch?[5]=Mode */

            elrs_data.Roll      = int16_Map_with_median(elrs_data.channels[3], 174, 1808, 992, -400, 400);
            elrs_data.Throttle  = int16_Map_with_median(elrs_data.channels[2], 174, 1811, 992, 5, 15);
            elrs_data.Yaw       = int16_Map_with_median(elrs_data.channels[0], 174, 1811, 992, -100, 100);
            /* midpoint_1用于：|Yaw| 较小时减小中点偏移量 */
            elrs_data.midpoint_1= int16_Map_with_median(abs16_fast(elrs_data.Yaw), 0, 100, 50, 0, 30);
            elrs_data.midpoint  = apply_deadzone(int16_Map_with_median(elrs_data.channels[1], 174, 1808, 992, -80, 80), 5);
            elrs_data.Switch    = (elrs_data.channels[4] > 1500) ? 1 : 0;
            elrs_data.Mode      = (elrs_data.channels[5] > 1700) ? 2 : ((elrs_data.channels[5] > 900 && elrs_data.channels[5] < 1100) ? 1 : 0);

            last_signal_time = HAL_GetTick();
        }

        else
        {

        }
    }

    memset(elrs_data_temp, 0, sizeof(elrs_data_temp));
	
	    ELRS_Init();
}
