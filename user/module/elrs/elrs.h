#ifndef __ELRS_H__
#define __ELRS_H__
#include "main.h"
#define MAX_FRAME_SIZE 36 // 空闲中断接收的最大帧长

/*
https://github.com/crsf-wg/crsf/wiki/Packet-Types CRSF协议定义
 */
/*
通道1：右摇杆 x轴
通道2：右摇杆 y轴
通道3：左摇杆 y轴
通道4：左摇杆 x轴
通道5：拨杆F
通道6：拨杆B
通道7：拨杆C
通道8：拨杆F
通道9：左滑块
通道10：右滑块
通道11：左按键A
通道12：右按键B
通道13：
通道14：
通道15：
通道16：
 */
/*
帧格式定义 地址 + 数据长度 + 帧类型 +  数据 + CRC校验码
 */

// 帧类型定义
#define CRSF_FRAMETYPE_GPS 0x02                      // GPS帧类型
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08           // 电池传感器帧类型
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14          // 链路统计帧类型
#define CRSF_FRAMETYPE_OPENTX_SYNC 0x10              // OpenTX同步帧类型
#define CRSF_FRAMETYPE_RADIO_ID 0x3A                 // 无线电ID帧类型
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16       // 遥控器通道打包帧类型
#define CRSF_FRAMETYPE_ATTITUDE 0x1E                 // 姿态帧类型
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21              // 飞行模式帧类型
#define CRSF_FRAMETYPE_DEVICE_PING 0x28              // 设备Ping帧类型
#define CRSF_FRAMETYPE_DEVICE_INFO 0x29              // 设备信息帧类型
#define CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0x2B // 参数设置条目帧类型
#define CRSF_FRAMETYPE_PARAMETER_READ 0x2C           // 参数读取帧类型
#define CRSF_FRAMETYPE_PARAMETER_WRITE 0x2D          // 参数写入帧类型
#define CRSF_FRAMETYPE_COMMAND 0x32                  // 命令帧类型
#define CRSF_FRAMETYPE_MSP_REQ 0x7A                  // 使用MSP序列作为命令的响应请求
#define CRSF_FRAMETYPE_MSP_RESP 0x7B                 // 以58字节分块二进制形式回复
#define CRSF_FRAMETYPE_MSP_WRITE 0x7C                // 以8字节分块二进制形式写入（OpenTX出站遥测缓冲区限制）
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B                // （CRSFv3）心跳

// 地址定义
#define CRSF_ADDRESS_BROADCAST 0x00         // 广播地址
#define CRSF_ADDRESS_USB 0x10               // USB地址
#define CRSF_ADDRESS_TBS_CORE_PNP_PRO 0x80  // TBS Core PNP Pro地址
#define CRSF_ADDRESS_RESERVED1 0x8A         // 保留地址1
#define CRSF_ADDRESS_CURRENT_SENSOR 0xC0    // 电流传感器地址
#define CRSF_ADDRESS_GPS 0xC2               // GPS地址
#define CRSF_ADDRESS_TBS_BLACKBOX 0xC4      // TBS黑匣子地址
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8 // 飞行控制器地址
#define CRSF_ADDRESS_RESERVED2 0xCA         // 保留地址2
#define CRSF_ADDRESS_RACE_TAG 0xCC          // 比赛标签地址
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA // 无线电发射器地址
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC     // CRSF接收机地址
#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE  // CRSF发射机地址

#define CHANNELS_Frame_Length 0x18 // 通道帧长度
#define LINK_Frame_Length 0x0C     // 连接帧长度

typedef struct
{
    // ==================== 遥控器控制数据（必需）====================
    //
    // ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
    // ┃  ELRS接收机数据结构体（双电机版本）                                         ┃
    // ┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
    // ┃  字段名           │ 数据类型   │  取值范围      │       说明（双电机版本）     ┃
    // ┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
    // ┃  Yaw             │ int16_t   │ -100 ~ +100   │ 偏航控制：左负右正         ┃
    // ┃  Throttle        │ uint16_t  │ 5 ~ 15        │ 油门控制：扑动幅度         ┃
    // ┃  Roll            │ int16_t   │ -400 ~ +400   │ 翻滚控制（未使用）         ┃
    // ┃  midpoint         │ int16_t   │ -80 ~ +80     │ 前后微调：翅膀前后倾斜     ┃
    // ┃  midpoint_1       │ int8_t    │ 0 ~ 30        │ Yaw偏置：左右差动补偿      ┃
    // ┃  Switch          │ uint8_t   │ 0 / 1 / 2     │ 电源开关：三段开关位置     ┃
    // ┃  Mode            │ uint8_t   │ 0 / 1 / 2     │ 飞行模式：0=并翅,1=平翅,2=扑动┃
    // ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
    //
    // 重要说明：
    // 1. 所有数据由ELRS_UARTE_RxCallback从CRSF协议帧中解析
    // 2. 油门值Throttle范围5-15是CRSF协议的原始值，经过处理后用于控制扑动幅度
    // 3. Yaw值用于差动控制：左转时右翼升力增大，左翼升力减小
    // 4. Switch三段开关控制Mode飞行模式

    int16_t 	Yaw;             // 【偏航控制】范围-100~+100，左负右正
                                    // 用于差动控制：左转时右翼升力增大
                                    // 调试方法：Watch窗口观察，偏航摇杆左打为负，右打为正

    uint16_t 	Throttle;       // 【油门控制】范围5-15，控制扑动幅度
                                    // 实际使用：经过处理后控制扑动频率和幅度
                                    // 调试方法：Watch窗口观察，油门摇杆上下对应5-15变化

    int16_t 	Roll;            // 【翻滚控制】范围-400~+400（双电机版本未使用）
                                    // 双电机版本只控制俯仰，不控制翻滚

    int16_t		midpoint;        // 【前后微调】范围-80~+80，调整翅膀前后倾斜
                                    // 正值：翅膀向前倾斜
                                    // 负值：翅膀向后倾斜
                                    // 调试方法：Watch窗口观察，前后微调旋钮调整

    int8_t		midpoint_1;      // 【Yaw偏置】范围0~30，左右差动补偿
                                    // 用于平衡左右电机的基础输出差异
                                    // 调试方法：Watch窗口观察，值越大差动越明显

    uint8_t 	Switch;          // 【电源开关】值0/1/2，三段开关位置
                                    // 0=位置1（上）, 1=位置2（中）, 2=位置3（下）
                                    // 调试方法：Watch窗口观察，开关切换时值在0/1/2之间跳变

    uint8_t 	Mode;            // 【飞行模式】值0/1/2/4，根据channels[6]独立开关切换
                                    // 0=并翅模式：电机保持竖直位置
                                    // 1=平翅模式：电机保持水平位置
                                    // 2=扑动模式：电机周期性扑动
                                    // 4=旋转模式：电机持续旋转（channels[6]>1500时优先进入）
                                    // 注意：Mode由channels[6]独立开关和channels[5]共同控制

    // ==================== 原始通道数据（用于调试）====================
    //
    // channels数组存储16个通道的原始值，来自CRSF协议的RC_CHANNELS_PACKED帧
    // 可用于调试遥控器连接和各通道工作状态
    //
    // ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
    // ┃  CRSF通道映射表（标准16通道）                                            ┃
    // ┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
    // ┃  通道   │  遥控器控件         │  说明                                  ┃
    // ┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
    // ┃  [0]    │  右摇杆X轴         │  未使用                                ┃
    // ┃  [1]    │  右摇杆Y轴         │  未使用                                ┃
    // ┃  [2]    │  左摇杆Y轴         │  → Throttle（油门）                   ┃
    // ┃  [3]    │  左摇杆X轴         │  → Yaw（偏航）                        ┃
    // ┃  [4]    │  拨杆A             │  → Switch（模式开关）                 ┃
    // ┃  [5]    │  拨杆B             │  → Mode（并翅/平翅/扑动）           ┃
    // ┃  [6]    │  拨杆C             │  → Mode4开关（>1500进入旋转模式）  ┃
    // ┃  [7]    │  拨杆D             │  未使用                                ┃
    // ┃  [8-15] │  滑块/按键         │  未使用                                ┃
    // ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

    uint16_t channels[16];      // 16个通道原始数据，范围0-4095

    // ==================== 遥测数据（如需使用可取消注释）====================

    // uint8_t uplink_RSSI_1;      // 上行RSSI：接收机信号强度
    // uint8_t uplink_Link_quality; // 上行链路质量：信号质量百分比
    // int8_t uplink_SNR;          // 上行信噪比：信噪比dB值
    // uint8_t downlink_RSSI;      // 下行RSSI：遥控器信号强度

} ELRS_Data;

extern ELRS_Data elrs_data;
void ELRS_Init(void);
void ELRS_UARTE_RxCallback(uint16_t Size);
#endif
