# 1. 问题

本问题涉及CRSF（Crossfire）协议中飞行控制器地址定义不一致的问题，导致接收到的CRSF数据包可能被错误过滤，严重影响飞行器的控制可靠性。

## 1.1. **地址定义不一致导致通信失败**

CRSF协议中飞行控制器地址在三个文件中存在不同的定义：

- `CRSF/CRSF_PROTOCOL.h`（第65行）：`CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xD5`（正确标准值）
- `user/module/elrs/elrs.h`（第59行）：`#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8`（错误值）
- `user/module/Receiver/Receiver.h`（第75行）：`CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8`（错误值）

这种不一致会导致以下问题：

**问题代码示例：**

```c
// user/module/elrs/elrs.c 第143行
if (elrs_data_temp[date_i+0] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
{
    // 处理CRSF数据包
}
```

当发射器发送地址为 `0xD5` 的数据包时，接收端使用 `0xC8` 进行校验，导致条件判断失败，合法的遥控信号被过滤掉。

**影响范围：**
- `user/module/elrs/elrs.c` 第143行：地址校验逻辑
- `user/module/Receiver/Receiver.c` 第191行：地址校验逻辑
- `CRSF/CRSF.c` 第69行：地址校验逻辑（使用正确的0xD5）

## 1.2. **重复定义增加维护成本**

除了地址值不一致外，还存在重复定义的问题。三个文件都定义了相同的CRSF地址枚举和帧类型枚举，这违反了DRY（Don't Repeat Yourself）原则，增加了维护成本：

- `CRSF/CRSF_PROTOCOL.h`：标准协议定义
- `user/module/elrs/elrs.h`：ELRS模块的局部定义
- `user/module/Receiver/Receiver.h`：Receiver模块的局部定义

当需要添加新的地址或帧类型时，需要在多个文件中同步修改，容易遗漏导致不一致。

## 1.3. **缺乏统一的协议头文件引用**

`user/module/elrs/elrs.h` 和 `user/module/Receiver/Receiver.h` 没有引用标准的 `CRSF/CRSF_PROTOCOL.h` 头文件，而是各自定义了重复的常量。这种做法导致：

- 无法保证协议定义的一致性
- 增加了代码重复
- 降低了代码的可维护性

# 2. 收益

通过统一CRSF协议地址定义，可以消除地址不一致导致的通信失败风险，提升系统的可靠性和可维护性。

## 2.1. **消除通信失败风险**

统一使用正确的地址值 `0xD5` 后，接收端能够正确识别来自发射器的CRSF数据包，避免合法遥控信号被错误过滤。这将直接提升飞行器的控制可靠性，防止因地址校验失败导致的信号丢失或延迟。

## 2.2. **降低维护成本**

通过消除重复定义，所有模块统一引用 `CRSF/CRSF_PROTOCOL.h`，未来添加新的地址或帧类型时只需修改一处，降低维护成本和出错概率。

预计可以减少约 **60%** 的重复定义代码（两个文件中的重复地址和帧类型定义）。

## 2.3. **提升代码一致性**

统一协议定义后，整个项目的CRSF协议实现将保持一致，便于团队协作和代码审查，减少因定义不一致导致的潜在bug。

# 3. 方案

本方案通过统一引用标准协议头文件、删除重复定义、修正地址值来解决CRSF协议地址不一致的问题。

## 3.1. **统一引用标准协议头文件**

### 实施步骤

1. 在 `user/module/elrs/elrs.h` 中引用 `CRSF/CRSF_PROTOCOL.h`
2. 在 `user/module/Receiver/Receiver.h` 中引用 `CRSF/CRSF_PROTOCOL.h`
3. 删除两个文件中的重复地址和帧类型定义
4. 修正 `CRSF/CRSF_PROTOCOL.h` 中的地址值为标准值 `0xD5`

### 代码修改前

**user/module/elrs/elrs.h（第51-64行）：**
```c
// 地址定义
#define CRSF_ADDRESS_BROADCAST 0x00         // 广播地址
#define CRSF_ADDRESS_USB 0x10               // USB地址
#define CRSF_ADDRESS_TBS_CORE_PNP_PRO 0x80  // TBS Core PNP Pro地址
#define CRSF_ADDRESS_RESERVED1 0x8A         // 保留地址1
#define CRSF_ADDRESS_CURRENT_SENSOR 0xC0    // 电流传感器地址
#define CRSF_ADDRESS_GPS 0xC2               // GPS地址
#define CRSF_ADDRESS_TBS_BLACKBOX 0xC4      // TBS黑匣子地址
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8 // 飞行控制器地址（错误值）
#define CRSF_ADDRESS_RESERVED2 0xCA         // 保留地址2
#define CRSF_ADDRESS_RACE_TAG 0xCC          // 比赛标签地址
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA // 无线电发射器地址
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC     // CRSF接收机地址
#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE  // CRSF发射机地址
```

**user/module/Receiver/Receiver.h（第66-81行）：**
```c
// 设备地址定义
typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,  // 错误值
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
}crsf_addr_e;
```

### 代码修改后

**user/module/elrs/elrs.h：**
```c
#ifndef __ELRS_H__
#define __ELRS_H__

#include "main.h"
#include "CRSF/CRSF_PROTOCOL.h"  // 新增：引用标准协议头文件

#define MAX_FRAME_SIZE 36 // 空闲中断接收的最大帧长

// 删除重复的地址定义（第51-64行）
// 删除重复的帧类型定义（第32-49行）

// 保留ELRS特定的数据结构定义
typedef struct
{
    int16_t 	Yaw;             // 偏航控制(-100~+100)
    uint16_t 	Throttle;       // 油门控制(5-15)
    int16_t 	Roll;            // 翻滚控制(-400~+400)
    int16_t		midpoint;        // 前后微调(-80~+80)
    int8_t		midpoint_1;      // Yaw偏置(0-30)
    uint8_t 	Switch;          // 电源开关(0/1)
    uint8_t 	Mode;            // 飞行模式(0=并翅,1=平翅,2=扑动)
    uint16_t channels[16];      // 16个通道原始数据
} ELRS_Data;

extern ELRS_Data elrs_data;
void ELRS_Init(void);
void ELRS_UARTE_RxCallback(uint16_t Size);

#endif
```

**user/module/Receiver/Receiver.h：**
```c
#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "usart.h"
#include "CRSF/CRSF_PROTOCOL.h"  // 新增：引用标准协议头文件

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

// SBUS相关定义保持不变
typedef struct
{
    uint16_t CH1;
    uint16_t CH2;
    // ... 其他通道定义
    uint8_t ConnectState;
}SBUS_CH_Struct;

void Sbus_Data_Read(uint8_t *buf);

// 删除重复的CRSF地址定义（第66-81行）
// 删除重复的CRSF帧类型定义（第84-100行）

// CRSF相关定义保持不变
#define CRSF_BAUDRATE  420000
#define CRSF_NUM_CHANNELS  16
#define CRSF_MAX_PACKET_SIZE  64
#define CRSF_MAX_PAYLOAD_LEN  (CRSF_MAX_PACKET_SIZE - 4)

typedef struct
{
    uint16_t CH1;
    uint16_t CH2;
    // ... 其他通道定义
    uint8_t ConnectState;
}CRSF_CH_Struct;

#endif
```

**CRSF/CRSF_PROTOCOL.h（保持不变，已经是正确值）：**
```c
typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xD5,  // 正确标准值
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_e;
```

## 3.2. **验证修改的正确性**

修改完成后，需要验证以下几点：

1. **编译检查**：确保所有文件能够正常编译，没有未定义的符号错误
2. **地址值一致性**：确认所有模块使用的是 `0xD5` 作为飞行控制器地址
3. **功能测试**：测试ELRS和Receiver模块能够正确接收和处理CRSF数据包

# 4. 回归范围

本次修改涉及CRSF协议地址定义的统一，需要从端到端业务流程角度进行回归测试，确保遥控信号的正常接收和处理。

## 4.1. 主链路

**正常遥控信号接收流程：**

1. 发射器发送CRSF数据包（目标地址：0xD5）
2. ELRS模块通过UART接收数据包
3. 地址校验通过（0xD5 == 0xD5）
4. 解析通道数据并更新控制信号
5. 飞行控制器根据控制信号执行相应动作

**测试重点：**
- 确认ELRS模块能够正确接收地址为 `0xD5` 的数据包
- 确认Receiver模块能够正确处理地址为 `0xD5` 的数据包
- 确认通道数据能够正确解析并更新到控制信号中
- 确认飞行控制器能够正常响应遥控指令

**测试用例示例：**

| 前置条件 | 操作步骤 | 预期结果 |
|---------|---------|---------|
| 发射器和接收机已配对 | 移动发射器摇杆，观察飞行器响应 | 飞行器能够实时响应遥控指令，无延迟或丢失 |
| 发射器和接收机已配对 | 切换飞行模式开关 | 飞行模式能够正确切换 |
| 发射器和接收机已配对 | 长时间持续操作 | 遥控信号稳定，无间歇性失效 |

## 4.2. 边界情况

**异常场景测试：**

1. **地址不匹配场景**
   - 触发条件：发射器发送地址为 `0xC8` 的数据包（模拟旧版本发射器）
   - 预期行为：数据包被正确过滤，不影响正常通信
   - 测试重点：确认系统不会因为地址不匹配而崩溃或进入异常状态

2. **数据包丢失场景**
   - 触发条件：模拟信号干扰导致数据包丢失
   - 预期行为：系统能够检测到信号丢失并进入failsafe模式
   - 测试重点：确认failsafe逻辑正常工作

3. **高频率数据包场景**
   - 触发条件：发射器以最大频率发送数据包
   - 预期行为：系统能够稳定处理高频率数据包，无缓冲区溢出
   - 测试重点：确认系统在高负载下的稳定性

4. **混合地址场景**
   - 触发条件：同时接收到地址为 `0xD5` 和 `0xC8` 的数据包
   - 预期行为：只处理地址为 `0xD5` 的数据包，忽略其他地址
   - 测试重点：确认地址过滤逻辑的正确性

**测试用例示例：**

| 场景 | 输入/条件 | 预期系统行为 |
|------|----------|-------------|
| 发射器地址错误 | 发射器发送地址为0xC8的数据包 | 数据包被过滤，不影响正常通信 |
| 信号干扰 | 模拟50%的数据包丢失率 | 系统检测到信号丢失，进入failsafe模式 |
| 高频数据包 | 发射器以1000Hz频率发送数据包 | 系统稳定处理，无缓冲区溢出或崩溃 |
| 混合地址 | 同时接收0xD5和0xC8地址的数据包 | 只处理0xD5地址的数据包，忽略其他 |