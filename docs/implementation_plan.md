# 多芯片SPI通信系统实施方案

> **文档版本**：v3.1（INT信号时序优化版）
> **日期**：2025年
> **项目**：3×CYT2BL3 + 1×CYT4BB7 SPI通信系统
> **修订说明**：v3.1 更新INT信号握手机制（从机检测CS上升沿清除INT，状态机保护）

---

## 一、实施目标

### 1.1 当前阶段目标

**在当前工程(4bb7_2bl3_spi)中实现纯SPI通信功能**：
- 3个CYT2BL3作为SPI从机（信标检测）
- 1个CYT4BB7作为SPI主机
- 不包含mknm_car_new的运动控制功能
- 为将来集成到mknm_car_new预留接口

### 1.2 已确认的技术参数

| 参数 | 确认值 |
|------|--------|
| 物理连接 | 排线约5cm（测试），PCB直连（将来） |
| 通信频率 | 100Hz（可配置） |
| SPI时钟 | 8MHz（可配置，支持便捷切换） |
| 实施方案 | 单硬件SPI_0 + 3个GPIO CS + 3个INT信号 |
| 开发策略 | 先中断模式，后DMA优化 |
| 测试策略 | 先1主1从验证，后扩展3从机 |

---

## 二、系统架构设计

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        CYT4BB7 (主机)                            │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                    应用层                                   │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │  │
│  │  │ 信标特征    │  │ 数据融合    │  │ 状态管理    │        │  │
│  │  │ 接收管理    │  │ 处理模块    │  │ 模块        │        │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘        │  │
│  └───────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                    通信层                                   │  │
│  │  ┌─────────────────────────────────────────────────────┐  │  │
│  │  │      SPI主机管理模块 (SPI_0/SCB7 + 3xGPIO CS)       │  │  │
│  │  │                                                     │  │  │
│  │  │   事件驱动: INT_1/INT_2/INT_3 触发 → SPI通信        │  │  │
│  │  │                                                     │  │  │
│  │  └─────────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
              │            │            │
     ┌────────┴──┐   ┌─────┴────┐  ┌────┴─────┐
     │  2BL3_1   │   │  2BL3_2  │  │  2BL3_3  │
     │ SPI从机   │   │ SPI从机  │  │ SPI从机  │
     │ +摄像头   │   │ +摄像头  │  │ +摄像头  │
     │ +信标检测 │   │ +信标检测 │  │ +信标检测│
     │ +屏幕     │   │ +屏幕    │  │ +屏幕    │
     │ +菜单     │   │ +菜单    │  │ +菜单    │
     └───────────┘   └──────────┘  └──────────┘
```

### 2.2 模块划分

#### 2.2.1 CYT4BB7模块

| 模块 | 文件 | 功能 |
|------|------|------|
| SPI主机驱动 | `spi_master.c/h` | SPI主机+GPIO CS控制 |
| 通信协议 | `comm_protocol.c/h` | 数据包封装解析 |
| 从机管理 | `slave_manager.c/h` | 3个从机事件驱动管理 |
| 信标数据 | `beacon_data.h` | 数据结构定义 |

#### 2.2.2 CYT2BL3模块

| 模块 | 文件 | 功能 |
|------|------|------|
| SPI从机驱动 | `spi_slave.c/h` | SPI从机通信 |
| 通信协议 | `comm_protocol.c/h` | 数据包封装解析（与主机共用） |
| INT信号 | `int_signal.c/h` | 数据就绪信号控制 |

---

## 三、硬件设计

### 3.1 引脚分配

#### 3.1.1 CYT4BB7引脚分配（用户确认）

| 功能 | 引脚 | SCB | 说明 |
|------|------|-----|------|
| **SPI_0通信总线** | | SCB7 | |
| CLK | P02_2 | | 时钟输出（共享） |
| MOSI | P02_1 | | 数据输出（共享） |
| MISO | P02_0 | | 数据输入（共享） |
| **从机1 (单从机测试)** | | - | 软件控制 |
| CS_1 (2BL3_1) | P02_3 | | 片选1（SPI0硬件CS0） |
| INT_1 (2BL3_1) | P02_4 | | 数据就绪检测1 |
| **从机2** | | - | |
| CS_2 (2BL3_2) | P01_0 | | 片选2（GPIO） |
| INT_2 (2BL3_2) | P01_1 | | 数据就绪检测2 |
| **从机3** | | - | |
| CS_3 (2BL3_3) | P19_0 | | 片选3（GPIO） |
| INT_3 (2BL3_3) | P19_1 | | 数据就绪检测3 |

**冲突检查**：
- P02_3/P02_4：WiFi模块默认引脚，用户确认不使用WiFi ✅
- P01_0/P01_1：无冲突（仅示例代码） ✅
- P19_0/P19_1：DL1A/DL1B TOF传感器默认引脚，用户确认不使用TOF ✅

#### 3.1.2 CYT2BL3引脚分配（使用SCB0）

| 功能 | 引脚 | SCB | HSIOM | 说明 |
|------|------|-----|-------|------|
| **SPI从机通信** | | SCB0 | | **Deep Sleep模式(30)** |
| MISO | **P0_0** | | P0_0_SCB0_SPI_MISO | 数据输出（向主机发送） |
| MOSI | **P0_1** | | P0_1_SCB0_SPI_MOSI | 数据输入（从主机接收） |
| CLK | **P0_2** | | P0_2_SCB0_SPI_CLK | 时钟输入 |
| SS | **P0_3** | | P0_3_SCB0_SPI_SELECT0 | 硬件片选输入 |
| **INT信号** | | GPIO | |
| INT_OUT | **P18_6** | | | 数据就绪信号输出 |

**重要说明**:
- SCB2(P14端口)的P14_x引脚没有SPI_SELECT0的HSIOM定义，无法用于SPI从机模式
- SCB0是CYT2BL3上唯一具备完整SPI从机功能且未被占用的SCB
- SCB0使用Deep Sleep HSIOM模式(值=30)

### 3.2 接线图

```
CYT4BB7                              CYT2BL3_1 / 2BL3_2 / 2BL3_3
┌─────────────┐                      ┌─────────────┐
│             │                      │             │
│ P02_2(CLK) ├────┬────┬────────────►│ P0_2(CLK)  │
│ P02_1(MOSI)├────┼────┼────────────►│ P0_1(MOSI) │
│ P02_0(MISO)│◄───┴────┴────────────┤ P0_0(MISO) │
│             │                      │             │
│ P02_3(CS1) ├──────────────────────►│ P0_3(SS)   │  ← 2BL3_1 (单从机测试)
│ P01_0(CS2) ├──────────────────────►│ P0_3(SS)   │  ← 2BL3_2
│ P19_0(CS3) ├──────────────────────►│ P0_3(SS)   │  ← 2BL3_3
│             │                      │             │
│ P02_4(INT1)│◄─────────────────────┤ P18_6(INT) │  ← 2BL3_1
│ P01_1(INT2)│◄─────────────────────┤ P18_6(INT) │  ← 2BL3_2
│ P19_1(INT3)│◄─────────────────────┤ P18_6(INT) │  ← 2BL3_3
│             │                      │             │
│ GND        ├───────────────────────┤ GND        │
└─────────────┘                      └─────────────┘

注意：CLK、MOSI、MISO是共享总线，连接到所有3个从机
      每个从机有独立的CS线(连接到P0_3硬件SS)和INT线
      所有2BL3的INT信号均从P18_6输出
      2BL3使用SCB0的P0端口，HSIOM=30(Deep Sleep模式)
```

---

## 四、通信协议设计

### 4.1 数据帧格式

#### 4.1.1 通用帧结构

```
┌──────┬──────┬──────┬────────────┬──────┬──────┐
│ HEAD │ CMD  │ LEN  │   DATA     │ CRC  │ TAIL │
│ 2B   │ 1B   │ 2B   │  0-256B    │ 2B   │ 1B   │
└──────┴──────┴──────┴────────────┴──────┴──────┘

HEAD: 0x5A 0xA5 (帧头标识)
CMD:  命令字节
LEN:  数据长度(小端)
DATA: 有效数据
CRC:  CRC16校验
TAIL: 0xAA (帧尾标识)
```

#### 4.1.2 命令定义

| 命令 | 值 | 方向 | 说明 |
|------|-----|------|------|
| CMD_NOP | 0x00 | M→S | 空操作/握手 |
| CMD_GET_STATUS | 0x01 | M→S | 获取从机状态 |
| CMD_GET_BEACON | 0x02 | M→S | 获取信标数据 |
| CMD_SET_PARAM | 0x10 | M→S | 设置参数 |
| CMD_ACK | 0x80 | S→M | 确认响应 |
| CMD_DATA | 0x81 | S→M | 数据响应 |
| CMD_ERROR | 0xFF | S→M | 错误响应 |

### 4.2 信标数据结构（用户确认）

```c
// 信标检测结果数据结构 (6字节)
typedef struct {
    uint8_t  beacon_found;       // 是否检测到信标
    uint8_t  beacon_count;       // 检测到的信标数量
    int16_t  nearest_beacon_x;   // 最近信标 X 坐标
    int16_t  nearest_beacon_y;   // 最近信标 Y 坐标
} beacon_result_t;

// 完整数据帧: 14字节
// HEAD(2) + CMD(1) + LEN(2) + beacon_result_t(6) + CRC(2) + TAIL(1) = 14字节
```

### 4.3 事件驱动通信时序

**INT信号握手机制**：从机检测到CS上升沿（主机完成读取）后清除INT信号

```
从机(2BL3)                              主机(4BB7)
    │                                       │
    │  [1] 图像处理完成                       │
    │      数据预加载到TX FIFO               │
    │                                       │
    ├───── [2] INT引脚拉高 ─────────────────►│ 检测到INT上升沿
    │                                       │ 加入待处理队列
    │                                       │
    │◄──────────── [3] CS拉低 ─────────────┤ 选中从机
    │                                       │
    │◄════ [4] CMD_GET_BEACON请求 ══════════┤ 发送请求帧
    │                                       │
    │═════ [5] beacon_result响应 ═══════════►│ 返回数据帧
    │                                       │
    │◄──────────── [6] CS拉高 ─────────────┤ 释放从机
    │      (从机检测到CS上升沿)               │
    │                                       │
    │  [7] INT引脚拉低                        │ 处理数据
    │      等待下次数据就绪                    │
    │                                       │
```

**关键设计要点**：
1. **数据先于INT**：从机必须先将数据预加载到TX FIFO，然后才拉高INT
2. **CS触发清除**：从机检测到CS上升沿后主动清除INT，避免时序竞争
3. **状态机保护**：从机使用状态机确保数据准备完成后才通知主机

### 4.4 通信参数（可配置）

```c
// comm_config.h

// SPI时钟配置（便捷切换）
#define COMM_SPI_SPEED_LOW      (1 * 1000 * 1000)   // 1MHz (长线缆)
#define COMM_SPI_SPEED_MEDIUM   (4 * 1000 * 1000)   // 4MHz (排线)
#define COMM_SPI_SPEED_HIGH     (8 * 1000 * 1000)   // 8MHz (PCB直连)
#define COMM_SPI_SPEED          COMM_SPI_SPEED_MEDIUM  // 当前配置

// 通信频率配置（便捷更改）
#define COMM_FREQUENCY_HZ       (100)              // 100Hz
#define COMM_PERIOD_MS          (1000 / COMM_FREQUENCY_HZ)  // 10ms
```

---

## 五、软件实现方案

### 5.1 CYT4BB7软件架构

#### 5.1.1 目录结构

```
4bb7/4bb7_2bl3_spi/
├── code/
│   ├── spi_comm/
│   │   ├── spi_master.c          # SPI主机驱动(含GPIO CS)
│   │   ├── spi_master.h
│   │   ├── comm_protocol.c       # 通信协议实现
│   │   ├── comm_protocol.h
│   │   ├── slave_manager.c       # 从机事件管理
│   │   ├── slave_manager.h
│   │   ├── beacon_data.h         # 数据结构定义
│   │   └── comm_config.h         # 配置参数
│   └── test/
│       └── comm_test.c           # 通信测试代码
├── user/
│   ├── main_cm7_0.c              # 主程序
│   └── cm7_0_isr.c               # 中断处理
└── libraries/                     # 逐飞库
```

#### 5.1.2 核心代码框架

**comm_config.h:**
```c
#ifndef _COMM_CONFIG_H_
#define _COMM_CONFIG_H_

// ============== SPI配置（用户确认） ==============
#define COMM_SPI_INDEX          SPI_0           // 使用SPI_0
#define COMM_SPI_CLK_PIN        SPI0_CLK_P02_2
#define COMM_SPI_MOSI_PIN       SPI0_MOSI_P02_1
#define COMM_SPI_MISO_PIN       SPI0_MISO_P02_0

// SPI时钟速度（便捷切换）
#define COMM_SPI_SPEED_1M       (1 * 1000 * 1000)   // 1MHz
#define COMM_SPI_SPEED_4M       (4 * 1000 * 1000)   // 4MHz
#define COMM_SPI_SPEED_8M       (8 * 1000 * 1000)   // 8MHz
#define COMM_SPI_SPEED          COMM_SPI_SPEED_4M   // 当前使用4MHz

// ============== GPIO CS引脚定义（用户确认） ==============
#define SLAVE_CS_1_PIN          P02_3           // 从机1 CS (单从机测试)
#define SLAVE_CS_2_PIN          P01_0           // 从机2 CS
#define SLAVE_CS_3_PIN          P19_0           // 从机3 CS

// ============== GPIO INT引脚定义（用户确认） ==============
#define SLAVE_INT_1_PIN         P02_4           // 从机1 INT
#define SLAVE_INT_2_PIN         P01_1           // 从机2 INT
#define SLAVE_INT_3_PIN         P19_1           // 从机3 INT

// ============== 从机数量 ==============
#define SLAVE_COUNT             3

// ============== 通信参数 ==============
#define COMM_BUFFER_SIZE        32              // 通信缓冲区大小
#define COMM_FREQUENCY_HZ       100             // 通信频率(Hz)
#define COMM_PERIOD_MS          (1000 / COMM_FREQUENCY_HZ)  // 通信周期(ms)

// ============== 协议常量 ==============
#define FRAME_HEAD_1            0x5A
#define FRAME_HEAD_2            0xA5
#define FRAME_TAIL              0xAA

#endif
```

**beacon_data.h:**
```c
#ifndef _BEACON_DATA_H_
#define _BEACON_DATA_H_

#include "zf_common_typedef.h"

// 信标检测结果 (用户确认的数据结构)
typedef struct {
    uint8_t  beacon_found;       // 是否检测到信标
    uint8_t  beacon_count;       // 检测到的信标数量
    int16_t  nearest_beacon_x;   // 最近信标 X 坐标
    int16_t  nearest_beacon_y;   // 最近信标 Y 坐标
} beacon_result_t;               // 6字节

#endif
```

**slave_manager.h:**
```c
#ifndef _SLAVE_MANAGER_H_
#define _SLAVE_MANAGER_H_

#include "beacon_data.h"
#include "spi_master.h"

// 从机编号定义
typedef enum {
    SLAVE_1 = 0,
    SLAVE_2,
    SLAVE_3,
    SLAVE_NUM = SLAVE_COUNT
} slave_id_t;

// 从机状态
typedef struct {
    uint8_t  online;             // 在线标志
    uint8_t  data_ready;         // 数据就绪标志(INT触发)
    uint8_t  error_count;        // 错误计数
    uint32_t last_update_time;   // 最后更新时间
    beacon_result_t beacon;      // 最新信标数据
} slave_status_t;

// 初始化
void slave_manager_init(void);

// 事件驱动任务(检测INT信号并通信)
void slave_manager_event_task(void);

// INT信号中断回调
void slave_manager_int_callback(slave_id_t slave);

// 获取从机状态
slave_status_t* slave_manager_get_status(slave_id_t slave);

// 获取所有从机信标数据
void slave_manager_get_all_beacons(beacon_result_t beacons[SLAVE_NUM]);

// 检查是否所有从机在线
uint8_t slave_manager_all_online(void);

#endif
```

**slave_manager.c (核心事件驱动逻辑):**
```c
#include "slave_manager.h"
#include "comm_protocol.h"
#include "zf_common_clock.h"

static slave_status_t slave_status[SLAVE_NUM];
static uint8_t tx_buffer[COMM_BUFFER_SIZE];
static uint8_t rx_buffer[COMM_BUFFER_SIZE];
static volatile uint8_t int_pending[SLAVE_NUM] = {0};  // INT待处理标志

// CS引脚数组（用户确认）
static const gpio_pin_enum cs_pins[SLAVE_NUM] = {
    P02_3,  // 从机1 CS (单从机测试)
    P01_0,  // 从机2 CS
    P19_0   // 从机3 CS
};

// INT引脚数组（用户确认）
static const gpio_pin_enum int_pins[SLAVE_NUM] = {
    P02_4,  // 从机1 INT
    P01_1,  // 从机2 INT
    P19_1   // 从机3 INT
};

void slave_manager_init(void) {
    spi_master_init();

    // 初始化CS引脚为输出（高电平）
    for (int i = 0; i < SLAVE_NUM; i++) {
        gpio_init(cs_pins[i], GPO, GPIO_HIGH, GPO_PUSH_PULL);
    }

    // 初始化INT引脚为输入
    for (int i = 0; i < SLAVE_NUM; i++) {
        gpio_init(int_pins[i], GPI, GPIO_LOW, GPI_PULL_DOWN);
    }

    // 配置INT引脚上升沿中断（可选）
    // exti_init(P02_4, EXTI_TRIGGER_RISING);  // 从机1
    // exti_init(P01_1, EXTI_TRIGGER_RISING);  // 从机2
    // exti_init(P19_1, EXTI_TRIGGER_RISING);  // 从机3

    for (int i = 0; i < SLAVE_NUM; i++) {
        slave_status[i].online = 0;
        slave_status[i].data_ready = 0;
        slave_status[i].error_count = 0;
        slave_status[i].last_update_time = 0;
    }
}

// INT信号中断回调(在外部中断ISR中调用)
void slave_manager_int_callback(slave_id_t slave) {
    if (slave < SLAVE_NUM) {
        int_pending[slave] = 1;  // 标记有数据待处理
    }
}

// 处理单个从机通信
static void process_slave(slave_id_t slave) {
    // 1. 选择从机（拉低CS）
    gpio_low(cs_pins[slave]);

    // 2. 构造请求数据包
    uint16_t tx_len = comm_protocol_build_request(
        CMD_GET_BEACON, NULL, 0, tx_buffer);

    // 3. SPI传输
    spi_master_transfer(tx_buffer, rx_buffer, tx_len + 14);

    // 4. 取消选择（拉高CS）
    gpio_high(cs_pins[slave]);

    // 5. 解析响应
    beacon_result_t *beacon = &slave_status[slave].beacon;
    if (comm_protocol_parse_response(rx_buffer, beacon)) {
        slave_status[slave].online = 1;
        slave_status[slave].error_count = 0;
        slave_status[slave].last_update_time = system_getval_ms();
    } else {
        slave_status[slave].error_count++;
        if (slave_status[slave].error_count > 10) {
            slave_status[slave].online = 0;
        }
    }
}

// 事件驱动任务(在主循环中调用)
void slave_manager_event_task(void) {
    // 检查每个从机的INT标志
    for (int i = 0; i < SLAVE_NUM; i++) {
        if (int_pending[i]) {
            int_pending[i] = 0;  // 清除标志
            process_slave((slave_id_t)i);
        }
    }

    // 备用: 也可以轮询检测INT引脚电平
    // if (gpio_get_level(P02_4)) process_slave(SLAVE_1);
    // if (gpio_get_level(P01_1)) process_slave(SLAVE_2);
    // if (gpio_get_level(P19_1)) process_slave(SLAVE_3);
}
```

### 5.2 CYT2BL3软件架构

#### 5.2.1 目录结构

```
2bl3/4bb7_2bl3_spi/
├── code/
│   ├── spi_comm/
│   │   ├── spi_slave.c           # SPI从机驱动
│   │   ├── spi_slave.h
│   │   ├── comm_protocol.c       # 通信协议(与主机共用)
│   │   ├── comm_protocol.h
│   │   ├── int_signal.c          # INT信号控制
│   │   ├── int_signal.h
│   │   └── comm_config.h         # 配置参数
│   ├── display/
│   │   ├── display_system.c      # 显示系统
│   │   └── display_system.h
│   └── menu/
│       ├── menu_core.c           # 菜单核心(移植自mknm_car_new)
│       ├── menu_core.h
│       └── ...                   # 暂时删除参数菜单
├── user/
│   ├── main_cm4.c                # 主程序
│   └── cm4_isr.c                 # 中断处理
└── libraries/                     # 逐飞库
```

#### 5.2.2 SPI从机配置（使用SCB0）

**comm_config.h (2BL3):**
```c
#ifndef _COMM_CONFIG_H_
#define _COMM_CONFIG_H_

// ============== SPI从机配置（使用SCB0） ==============
#define SPI_SLAVE_SCB           SCB0            // 使用SCB0 (唯一具备完整SPI从机功能的SCB)

// 引脚定义 (P0端口, HSIOM=30 Deep Sleep模式)
#define SPI_SLAVE_MISO_PIN      P0_0            // 数据输出
#define SPI_SLAVE_MOSI_PIN      P0_1            // 数据输入
#define SPI_SLAVE_CLK_PIN       P0_2            // 时钟输入
#define SPI_SLAVE_SS_PIN        P0_3            // 硬件片选输入

// ============== INT信号配置 ==============
#define INT_SIGNAL_PIN          P18_6           // 数据就绪信号输出

// ============== 通信参数 ==============
#define COMM_BUFFER_SIZE        32              // 通信缓冲区大小

// ============== 协议常量 ==============
#define FRAME_HEAD_1            0xAA
#define FRAME_HEAD_2            0x55
#define FRAME_TAIL              0xED

#endif
```

**int_signal.h:**
```c
#ifndef _INT_SIGNAL_H_
#define _INT_SIGNAL_H_

#include "zf_common_typedef.h"

// 从机状态定义
typedef enum {
    SLAVE_STATE_IDLE,           // 空闲，等待图像处理完成
    SLAVE_STATE_DATA_READY,     // 数据已准备到TX FIFO，INT已拉高
    SLAVE_STATE_TRANSFERRING,   // 正在被主机读取（CS=低）
} slave_state_t;

// 初始化INT信号引脚
void int_signal_init(void);

// 通知主机有新数据就绪（需先调用spi_slave_prepare_tx准备数据）
void int_signal_set_ready(void);

// 清除数据就绪信号
void int_signal_clear(void);

// 状态机任务（需在主循环中调用，检测CS上升沿）
void int_signal_state_task(void);

// 获取当前状态
slave_state_t int_signal_get_state(void);

#endif
```

**int_signal.c:**
```c
#include "int_signal.h"
#include "comm_config.h"
#include "zf_driver_gpio.h"

static volatile slave_state_t slave_state = SLAVE_STATE_IDLE;
static uint8_t last_cs_state = 1;  // CS默认高电平（未选中）

void int_signal_init(void) {
    gpio_init(INT_SIGNAL_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(SPI_SLAVE_CS_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    slave_state = SLAVE_STATE_IDLE;
    last_cs_state = 1;
}

void int_signal_set_ready(void) {
    // 只有在空闲状态才能设置INT（确保数据已准备好）
    if (slave_state == SLAVE_STATE_IDLE) {
        gpio_high(INT_SIGNAL_PIN);  // 拉高INT信号
        slave_state = SLAVE_STATE_DATA_READY;
    }
}

void int_signal_clear(void) {
    gpio_low(INT_SIGNAL_PIN);   // 拉低INT信号
    slave_state = SLAVE_STATE_IDLE;
}

// 状态机任务 - 检测CS上升沿清除INT
void int_signal_state_task(void) {
    uint8_t curr_cs_state = gpio_get_level(SPI_SLAVE_CS_PIN);

    switch (slave_state) {
        case SLAVE_STATE_IDLE:
            // 空闲状态，等待数据准备完成后调用int_signal_set_ready()
            break;

        case SLAVE_STATE_DATA_READY:
            // 数据就绪，等待主机选中（CS下降沿）
            if (last_cs_state == 1 && curr_cs_state == 0) {
                slave_state = SLAVE_STATE_TRANSFERRING;
            }
            break;

        case SLAVE_STATE_TRANSFERRING:
            // 传输中，等待主机完成读取（CS上升沿）
            if (last_cs_state == 0 && curr_cs_state == 1) {
                int_signal_clear();  // CS拉高时清除INT
            }
            break;
    }

    last_cs_state = curr_cs_state;
}

slave_state_t int_signal_get_state(void) {
    return slave_state;
}
```

#### 5.2.3 SPI从机驱动关键实现

**spi_slave.c (核心初始化):**
```c
#include "spi_slave.h"
#include "comm_config.h"
#include "scb/cy_scb_spi.h"
#include "gpio/cy_gpio.h"
#include "sysclk/cy_sysclk.h"

static cy_stc_scb_spi_context_t spi_context;
static uint8_t tx_buffer[COMM_BUFFER_SIZE];
static uint8_t rx_buffer[COMM_BUFFER_SIZE];
static volatile uint8_t transfer_complete = 0;

// SPI从机配置
static const cy_stc_scb_spi_config_t spi_slave_config = {
    .spiMode            = CY_SCB_SPI_SLAVE,         // 从机模式!
    .subMode            = CY_SCB_SPI_MOTOROLA,
    .sclkMode           = CY_SCB_SPI_CPHA0_CPOL0,   // MODE0
    .oversample         = 0,                         // 从机不需要过采样
    .rxDataWidth        = 8,
    .txDataWidth        = 8,
    .enableMsbFirst     = true,
    .enableInputFilter  = true,
    .enableFreeRunSclk  = false,
    .enableMisoLateSample = false,
    .enableTransferSeperation = false,

    // FIFO配置
    .rxFifoTriggerLevel   = 0,
    .rxFifoIntEnableMask  = CY_SCB_SPI_RX_NOT_EMPTY,
    .txFifoTriggerLevel   = 0,
    .txFifoIntEnableMask  = 0,

    .enableSpiDoneInterrupt = true,
    .enableSpiBusErrorInterrupt = true,
};

void spi_slave_init(void) {
    cy_stc_gpio_pin_config_t pin_cfg = {0};

    // 1. 配置GPIO引脚 (P0端口, HSIOM=30 Deep Sleep模式)
    // MISO - 从机输出 (强驱动)
    pin_cfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pin_cfg.hsiom = P0_0_SCB0_SPI_MISO;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 0, &pin_cfg);

    // MOSI - 从机输入 (高阻)
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P0_1_SCB0_SPI_MOSI;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 1, &pin_cfg);

    // CLK - 从机输入 (高阻)
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P0_2_SCB0_SPI_CLK;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 2, &pin_cfg);

    // SS - 硬件片选输入 (使用SCB硬件功能)
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P0_3_SCB0_SPI_SELECT0;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 3, &pin_cfg);

    // 2. 配置时钟
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB0_CLOCK, CY_SYSCLK_DIV_8_BIT, 0);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 0, 0);  // 不分频
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 0);

    // 3. 初始化SCB0为SPI从机
    Cy_SCB_SPI_Init(SCB0, &spi_slave_config, &spi_context);
    Cy_SCB_SPI_SetActiveSlaveSelect(SCB0, CY_SCB_SPI_SLAVE_SELECT0);
    Cy_SCB_SPI_Enable(SCB0);

    // 4. 配置中断
    // ... (中断配置代码)
}

// 准备发送数据（在设置INT之前必须先调用此函数预加载数据）
void spi_slave_prepare_tx(const uint8_t *data, uint16_t len) {
    Cy_SCB_SPI_ClearTxFifo(SCB0);
    for (uint16_t i = 0; i < len; i++) {
        Cy_SCB_SPI_Write(SCB0, data[i]);
    }
}

// 检查是否被选中(SS低电平) - 使用硬件SS引脚P0_3
uint8_t spi_slave_is_selected(void) {
    return (Cy_GPIO_Read(GPIO_PRT0, 3) == 0);
}

// 获取接收到的数据
uint16_t spi_slave_get_rx_data(uint8_t *data, uint16_t max_len) {
    uint16_t count = 0;
    while (Cy_SCB_SPI_GetNumInRxFifo(SCB0) > 0 && count < max_len) {
        data[count++] = (uint8_t)Cy_SCB_SPI_Read(SCB0);
    }
    return count;
}
```

#### 5.2.4 从机数据通知使用示例

**正确的使用顺序（数据先于INT）**：
```c
// 在图像处理完成的回调中调用
void on_image_process_complete(beacon_result_t *result) {
    // 1. 先构建响应数据帧
    uint8_t response_frame[COMM_BUFFER_SIZE];
    uint16_t frame_len = comm_protocol_build_response(
        CMD_DATA, (uint8_t*)result, sizeof(beacon_result_t), response_frame);

    // 2. 将数据预加载到TX FIFO（必须在设置INT之前）
    spi_slave_prepare_tx(response_frame, frame_len);

    // 3. 数据准备完成后，通知主机（设置INT）
    int_signal_set_ready();
}

// 主循环中调用状态机任务
void main_loop(void) {
    while (1) {
        // 状态机任务：检测CS上升沿并清除INT
        int_signal_state_task();

        // 其他任务...
    }
}
```

---

## 六、开发阶段规划

### 6.1 第一阶段：基础通信验证（中断模式）

**目标**：1个4BB7 + 1个2BL3实现基础SPI通信

**任务**：
1. [ ] 4BB7 SPI主机驱动开发（单CS，中断模式）
2. [ ] 2BL3 SPI从机驱动开发（中断模式）
3. [ ] 通信协议基础实现
4. [ ] INT信号机制实现
5. [ ] 简单数据收发测试

**验收标准**：
- 主机能正确发送请求
- 从机能正确响应
- INT信号触发正常
- 数据无丢失、无错误

### 6.2 第二阶段：多从机扩展

**目标**：扩展到3个CS控制3个从机

**任务**：
1. [ ] 4BB7添加GPIO CS控制代码
2. [ ] 4BB7添加多INT检测
3. [ ] 从机管理模块开发
4. [ ] 事件驱动调度实现
5. [ ] 3从机通信测试

**验收标准**：
- 3个CS线能正确控制
- 3个INT信号能正确检测
- 事件驱动通信正常

### 6.3 第三阶段：DMA优化

**目标**：使用DMA提升通信效率

**任务**：
1. [ ] 4BB7 SPI主机DMA实现
2. [ ] 2BL3 SPI从机DMA实现
3. [ ] 性能测试和优化

**验收标准**：
- DMA传输稳定
- CPU占用率显著降低

### 6.4 第四阶段：功能完善

**目标**：完成2BL3端的显示和菜单功能

**任务**：
1. [ ] 移植IPS114屏幕驱动
2. [ ] 移植菜单系统框架（暂不含参数菜单）
3. [ ] 屏幕显示测试

**验收标准**：
- 屏幕显示正常
- 菜单操作正常

### 6.5 第五阶段：集成测试

**目标**：系统联调

**任务**：
1. [ ] 3个2BL3同时运行测试
2. [ ] 100Hz通信稳定性测试
3. [ ] 长时间稳定性测试
4. [ ] 异常处理测试
5. [ ] 性能优化

---

## 七、测试方案

### 7.1 单元测试

| 测试项 | 测试方法 | 预期结果 |
|--------|---------|---------|
| GPIO CS控制 | 示波器观察 | 3个CS独立可控 |
| GPIO INT检测 | 手动触发 | 能检测上升沿 |
| SPI主机发送 | 示波器观察波形 | 时序正确 |
| SPI从机接收 | 打印接收数据 | 数据正确 |
| CRC校验 | 故意破坏数据 | 能检测错误 |

### 7.2 集成测试

| 测试项 | 测试方法 | 预期结果 |
|--------|---------|---------|
| 单从机通信 | 持续通信1小时 | 无错误 |
| 事件驱动 | INT触发通信 | 响应及时 |
| 多从机轮询 | 3从机同时运行 | 各从机正常 |
| 100Hz测试 | 监测通信频率 | 稳定100Hz |

### 7.3 验收标准

| 测试项 | 验收标准 |
|--------|---------|
| 通信成功率 | >99.9% |
| 通信频率 | 100Hz±5% |
| 响应延迟 | <1ms |
| 稳定运行时间 | ≥24小时 |

---

## 附录

### A. 参考代码位置

| 功能 | 参考代码路径 |
|------|-------------|
| SPI主机(4BB7) | `mknm_car_new/libraries/zf_driver/zf_driver_spi.c` |
| SPI主机(2BL3) | `4bb7_2bl3_spi/2bl3/libraries/zf_driver/zf_driver_spi.c` |
| SCB SPI底层 | `libraries/sdk/common/src/drivers/scb/cy_scb_spi.c` |
| 菜单系统 | `mknm_car_new/mknm_car/code/menu/` |

### B. 官方参考资源

- [Infineon SPI Master with DMA Example](https://github.com/Infineon/mtb-t2g-lite-example-spi-master-dma)
- [AN225401 - How to use SCB in TRAVEO T2G](https://documentation.infineon.com/traveo/docs/qmr1680597505537)
- [CYT4BB Datasheet](https://www.infineon.com/cms/en/product/microcontroller/32-bit-traveo-t2g-arm-cortex-microcontroller/)
- [CYT2BL Datasheet](https://www.infineon.com/cms/en/product/microcontroller/32-bit-traveo-t2g-arm-cortex-microcontroller/)
