#ifndef __ELRS_H__
#define __ELRS_H__
#include "main.h"
#define MAX_FRAME_SIZE 36 /* CRSF帧最大长度 */

/*
 * https://github.com/crsf-wg/crsf/wiki/Packet-Types  CRSF 协议定义
 */
/*
 * 通道1~4对应摇杆X/Y轴
 * 通道1：右摇杆 X
 * 通道2：右摇杆 Y
 * 通道3：左摇杆 Y
 * 通道4：左摇杆 X
 * 通道5：拨杆 A
 * 通道6：拨杆 B
 * 通道7：拨杆 C
 * 通道8：拨杆 D
 * 通道9：拨杆 E
 * 通道10：拨杆 F
 * 通道11：拨杆 G
 * 通道12：拨杆 H
 * 通道13~16：预留
 */
/*
 * 帧结构 = 地址 + 长度 + 类型 + 数据 + CRC
 */

/* CRSF ??? */
#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_OPENTX_SYNC 0x10
#define CRSF_FRAMETYPE_RADIO_ID 0x3A
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21
#define CRSF_FRAMETYPE_DEVICE_PING 0x28
#define CRSF_FRAMETYPE_DEVICE_INFO 0x29
#define CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0x2B
#define CRSF_FRAMETYPE_PARAMETER_READ 0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE 0x2D
#define CRSF_FRAMETYPE_COMMAND 0x32
#define CRSF_FRAMETYPE_MSP_REQ 0x7A
#define CRSF_FRAMETYPE_MSP_RESP 0x7B
#define CRSF_FRAMETYPE_MSP_WRITE 0x7C
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B

/* ???? */
#define CRSF_ADDRESS_BROADCAST 0x00
#define CRSF_ADDRESS_USB 0x10
#define CRSF_ADDRESS_TBS_CORE_PNP_PRO 0x80
#define CRSF_ADDRESS_RESERVED1 0x8A
#define CRSF_ADDRESS_CURRENT_SENSOR 0xC0
#define CRSF_ADDRESS_GPS 0xC2
#define CRSF_ADDRESS_TBS_BLACKBOX 0xC4
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RESERVED2 0xCA
#define CRSF_ADDRESS_RACE_TAG 0xCC
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC
#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE

#define CHANNELS_Frame_Length 0x18 /* RC 通道数据帧长度 */
#define LINK_Frame_Length 0x0C     /* 链路状态帧长度 */

/* Failsafe超时时间，单位毫秒 */
#define SIGNAL_TIMEOUT_MS 500

typedef struct
{
    int16_t 	Yaw;             /* 偏航 -100~+100 */
    uint16_t 	Throttle;       /* 油门 5~15 */
    int16_t 	Roll;            /* 横滚 -400~+400 */
    int16_t		midpoint;        /* 俯仰 -80~+80 */
    int8_t		midpoint_1;      /* Yaw 相关 0~30 */
    uint8_t 	Switch;          /* 开关量 0/1 */
    uint8_t 	Mode;            /* 0=手动 1=半自动 2=自动 */

    uint16_t channels[16];      /* 16 通道原始 CRSF 数据 */

} ELRS_Data;

extern ELRS_Data elrs_data;

extern volatile uint32_t last_signal_time;

void ELRS_Init(void);
void ELRS_UARTE_RxCallback(uint16_t Size);
#endif
