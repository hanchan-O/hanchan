#ifndef __ELRS_H__
#define __ELRS_H__
#include "main.h"
#define MAX_FRAME_SIZE 36 /* ????????????? */

/*
 * https://github.com/crsf-wg/crsf/wiki/Packet-Types  CRSF ???????
 */
/*
 * ?????????/??????????
 * ??1???? X
 * ??2???? Y
 * ??3???? Y
 * ??4???? X
 * ??5??? A
 * ??6??? B
 * ??7??? C
 * ??8??? D
 * ??9????
 * ??10????
 * ??11???? A
 * ??12???? B
 * ??13~16???
 */
/*
 * ?????? + ?? + ?? + ?? + CRC
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

#define CHANNELS_Frame_Length 0x18 /* RC ??????? */
#define LINK_Frame_Length 0x0C     /* ??????? */

/* Failsafe?????????????? */
#define SIGNAL_TIMEOUT_MS 500

typedef struct
{
    int16_t 	Yaw;             /* ?? -100~+100 */
    uint16_t 	Throttle;       /* ???? 5~15 */
    int16_t 	Roll;            /* ?? -400~+400 */
    int16_t		midpoint;        /* ???? -80~+80 */
    int8_t		midpoint_1;      /* Yaw ?? 0~30 */
    uint8_t 	Switch;          /* ???? 0/1 */
    uint8_t 	Mode;            /* 0=?? 1=?? 2=?? */

    uint16_t channels[16];      /* 16 ?????????? */

} ELRS_Data;

extern ELRS_Data elrs_data;

extern volatile uint32_t last_signal_time;

void ELRS_Init(void);
void ELRS_UARTE_RxCallback(uint16_t Size);
#endif
