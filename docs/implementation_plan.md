# 多芯片SPI通信系统实施方案

> **文档版本**：v4.0（第一阶段验证完成版）
> **日期**：2025年12月
> **项目**：3×CYT2BL3 + 1×CYT4BB7 SPI通信系统
> **修订说明**：v4.0 第一阶段基础通信验证完成，更新为实际实现的代码和架构

---

## 一、实施目标

### 1.1 当前阶段目标

**在当前工程(4bb7_2bl3_spi)中实现纯SPI通信功能**：
- 3个CYT2BL3作为SPI从机（信标检测）
- 1个CYT4BB7作为SPI主机
- 不包含mknm_car_new的运动控制功能
- 为将来集成到mknm_car_new预留接口

### 1.2 已确认的技术参数

| 参数 | 确认值 | 实际状态 |
|------|--------|----------|
| 物理连接 | 排线约5cm（测试），PCB直连（将来） | ✅ 已验证 |
| 通信频率 | 100Hz（可配置） | ✅ 支持 |
| SPI时钟 | 1MHz（可配置，支持便捷切换） | ✅ 已验证 |
| 实施方案 | 单硬件SPI_0 + GPIO CS + INT信号 | ✅ 已实现 |
| 开发策略 | 先中断模式，后DMA优化 | ✅ 第一阶段完成 |
| 测试策略 | 先1主1从验证，后扩展3从机 | ✅ 1主1从验证成功 |

### 1.3 当前进度

| 阶段 | 状态 | 完成日期 |
|------|------|----------|
| **第一阶段：基础通信验证** | ✅ **已完成** | 2025-12-25 |
| 第二阶段：主机DMA优化 | ⏳ 待开始 | - |
| 第三阶段：从机DMA优化（可选） | ⏳ 待开始 | - |
| 第四阶段：多从机扩展 | ⏳ 待开始 | - |
| 第五阶段：功能完善 | ⏳ 待开始 | - |
| 第六阶段：集成测试 | ⏳ 待开始 | - |

**阶段调整说明**（v4.1）：
- 原方案：先多从机扩展，后DMA优化
- 新方案：**先DMA优化，后多从机扩展**
- 调整理由：
  1. DMA是底层变化，先稳定底层再扩展上层
  2. 单从机环境下调试DMA，变量最少，便于定位问题
  3. DMA接口确定后，多从机管理可直接基于异步API设计
  4. 避免"先写同步代码，后改异步"的重构成本

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
│  │  │      SPI主机管理模块 (SPI_0/SCB7 + GPIO CS)         │  │  │
│  │  │                                                     │  │  │
│  │  │   事件驱动: INT信号触发 → SPI通信                   │  │  │
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

### 2.2 实际模块划分

#### 2.2.1 CYT4BB7模块（实际实现）

| 模块 | 文件 | 功能 | 状态 |
|------|------|------|------|
| SPI主机通信 | `code/spi_comm.c/h` | SPI主机通信+协议解析 | ✅ 已实现 |
| 主程序 | `user/main_cm7_0.c` | 主循环和测试 | ✅ 已实现 |

#### 2.2.2 CYT2BL3模块（实际实现）

| 模块 | 文件 | 功能 | 状态 |
|------|------|------|------|
| SPI从机通信 | `code/spi_slave.c/h` | SPI从机通信+协议构建 | ✅ 已实现 |
| 主程序 | `user/main_cm4.c` | 主循环和测试数据 | ✅ 已实现 |

---

## 三、硬件设计

### 3.1 引脚分配

#### 3.1.1 CYT4BB7引脚分配（实际使用）

| 功能 | 引脚 | SCB | 说明 | 状态 |
|------|------|-----|------|------|
| **SPI_0通信总线** | | SCB7 | | ✅ |
| CLK | P02_2 | | 时钟输出（共享） | ✅ |
| MOSI | P02_1 | | 数据输出（共享） | ✅ |
| MISO | P02_0 | | 数据输入（共享） | ✅ |
| **从机1 (单从机测试)** | | - | 软件控制 | ✅ |
| CS_1 (2BL3_1) | P02_3 | | 片选1（GPIO软件控制） | ✅ |
| INT_1 (2BL3_1) | P02_4 | | 数据就绪检测1 | ✅ |
| **从机2** | | - | | ⏳ |
| CS_2 (2BL3_2) | P01_0 | | 片选2（GPIO） | ⏳ 待实现 |
| INT_2 (2BL3_2) | P01_1 | | 数据就绪检测2 | ⏳ 待实现 |
| **从机3** | | - | | ⏳ |
| CS_3 (2BL3_3) | P19_0 | | 片选3（GPIO） | ⏳ 待实现 |
| INT_3 (2BL3_3) | P19_1 | | 数据就绪检测3 | ⏳ 待实现 |

#### 3.1.2 CYT2BL3引脚分配（实际使用 - SCB0）

| 功能 | 引脚 | SCB | HSIOM | 说明 | 状态 |
|------|------|-----|-------|------|------|
| **SPI从机通信** | | SCB0 | | **Deep Sleep模式(30)** | ✅ |
| MISO | **P0_0** | | P0_0_SCB0_SPI_MISO | 数据输出（向主机发送） | ✅ |
| MOSI | **P0_1** | | P0_1_SCB0_SPI_MOSI | 数据输入（从主机接收） | ✅ |
| CLK | **P0_2** | | P0_2_SCB0_SPI_CLK | 时钟输入 | ✅ |
| SS | **P0_3** | | P0_3_SCB0_SPI_SELECT0 | 硬件片选输入 | ✅ |
| **INT信号** | | GPIO | | | ✅ |
| INT_OUT | **P18_6** | | | 数据就绪信号输出 | ✅ |

**重要说明**:
- SCB2(P14端口)的P14_x引脚没有SPI_SELECT0的HSIOM定义，无法用于SPI从机模式
- SCB0是CYT2BL3上唯一具备完整SPI从机功能且未被占用的SCB
- SCB0使用Deep Sleep HSIOM模式(值=30)
- ⚠️ SCB0与Debug UART冲突，需禁用debug_init()

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

## 四、通信协议设计（实际实现）

### 4.1 数据帧格式

#### 4.1.1 通用帧结构

```
┌──────┬──────┬──────┬────────────┬──────┬──────┐
│ HEAD │ CMD  │ LEN  │   DATA     │ CRC  │ TAIL │
│ 2B   │ 1B   │ 2B   │  0-32B     │ 2B   │ 1B   │
└──────┴──────┴──────┴────────────┴──────┴──────┘

HEAD: 0xAA 0x55 (帧头标识)
CMD:  命令字节
LEN:  数据长度(小端序，高字节在前)
DATA: 有效数据
CRC:  CRC16-Modbus校验 (计算范围: CMD+LEN+DATA)
TAIL: 0xED (帧尾标识)
```

#### 4.1.2 命令定义（实际实现）

| 命令 | 值 | 方向 | 说明 |
|------|-----|------|------|
| CMD_PING | 0x01 | M→S | 心跳测试 |
| CMD_GET_BEACON | 0x10 | M→S | 获取信标数据 |

### 4.2 信标数据结构（实际实现）

```c
// 来源: spi_comm.h / spi_slave.h
typedef struct
{
    int16 center_x;      // 灯塔中心X坐标 (2字节, 小端序)
    int16 center_y;      // 灯塔中心Y坐标 (2字节, 小端序)
    uint8 found;         // 是否找到灯塔 (1字节)
    uint8 confidence;    // 置信度(0-100) (1字节)
} beacon_result_t;       // 总计: 6字节 (BEACON_DATA_SIZE)

// 完整响应帧: 14字节 (BEACON_RESPONSE_LEN)
// HEAD(2) + CMD(1) + LEN(2) + beacon_result_t(6) + CRC(2) + TAIL(1) = 14字节
```

### 4.3 CRC16校验算法

```c
// 来源: spi_comm.c / spi_slave.c
// Modbus CRC16算法
static uint16 calc_crc16(const uint8 *data, uint16 len)
{
    uint16 crc = 0xFFFF;
    for (uint16 i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8 j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
```

### 4.4 事件驱动通信时序（实际实现）

**TX FIFO预加载策略**：从机在拉高INT之前预加载数据到TX FIFO

```
从机(2BL3)                              主机(4BB7)
    │                                       │
    │  [1] 数据更新完成                       │
    │      build_beacon_response()          │
    │      preload_tx_fifo()               │
    │      g_data_pending = 1              │
    │                                       │
    ├───── [2] INT引脚拉高 ─────────────────►│ gpio_get_level(INT)检测
    │      spi_slave_set_int(1)            │
    │                                       │
    │◄──────────── [3] CS拉低 ─────────────┤ gpio_low(CS)
    │      (SS下降沿检测)                    │ system_delay_us(5)
    │      清空RX FIFO                      │
    │                                       │
    │◄════════════ [4] SPI传输 ════════════┤ spi_transfer_8bit()
    │      TX FIFO数据自动发出              │
    │                                       │
    │════════════ [5] 响应数据 ═════════════►│ 接收到rx_buf
    │                                       │
    │◄──────────── [6] CS拉高 ─────────────┤ gpio_high(CS)
    │      (SS上升沿检测)                    │
    │                                       │
    │  [7] INT引脚拉低                        │ parse_response_frame()
    │      g_data_pending = 0              │ 解析beacon_result
    │      preload_tx_fifo()               │
    │                                       │
```

### 4.5 协议参数定义（实际实现）

```c
// 来源: spi_comm.h
#define FRAME_HEAD_1        0xAA
#define FRAME_HEAD_2        0x55
#define FRAME_TAIL          0xED

#define CMD_PING            0x01
#define CMD_GET_BEACON      0x10

#define MAX_DATA_SIZE       32
#define FRAME_OVERHEAD      8    // HEAD(2)+CMD(1)+LEN(2)+CRC(2)+TAIL(1)
#define MAX_FRAME_SIZE      (MAX_DATA_SIZE + FRAME_OVERHEAD)
#define BEACON_DATA_SIZE    6    // 灯塔数据大小
#define BEACON_RESPONSE_LEN 14   // 灯塔响应帧长度: 8+6

// 错误码定义
#define SPI_ERR_OK              0   // 成功
#define SPI_ERR_FRAME_SHORT     1   // 帧长度不足
#define SPI_ERR_INVALID_HEAD    2   // 帧头错误
#define SPI_ERR_PAYLOAD_LONG    3   // 数据长度超限
#define SPI_ERR_INVALID_TAIL    4   // 帧尾错误
#define SPI_ERR_CRC_MISMATCH    5   // CRC校验失败
#define SPI_ERR_INCOMPLETE      6   // 帧不完整
#define SPI_ERR_NULL_PTR        7   // 空指针参数
#define SPI_ERR_DATA_SIZE       10  // 数据大小不匹配

// CS延时参数
#define SPI_CS_SETUP_DELAY_US   5   // CS拉低后到传输开始的延时

// 状态打印间隔
#define STATUS_PRINT_INTERVAL   1000000
```

---

## 五、软件实现方案（实际代码）

### 5.1 CYT4BB7软件架构

#### 5.1.1 实际目录结构

```
4bb7/4bb7_2bl3_spi/
├── code/
│   ├── spi_comm.c          # SPI主机通信实现
│   ├── spi_comm.h          # SPI主机通信头文件
│   └── 本文件夹作用.txt
├── user/
│   ├── main_cm7_0.c        # 主程序
│   └── cm7_0_isr.c         # 中断处理
└── libraries/               # 逐飞库
```

#### 5.1.2 主要函数（实际实现）

**spi_comm.h 关键定义:**
```c
// 硬件配置
#define SPI_MASTER_CH       SPI_0
#define SPI_MASTER_BAUD     (1000000)  // 1MHz波特率
#define SPI_MASTER_CLK      SPI0_CLK_P02_2
#define SPI_MASTER_MOSI     SPI0_MOSI_P02_1
#define SPI_MASTER_MISO     SPI0_MISO_P02_0
#define SPI_CS_PIN          P02_3
#define SPI_INT_PIN         P02_4

// 函数声明
void spi_comm_init(void);              // 初始化SPI通信模块
uint8 spi_comm_data_ready(void);       // 检测INT信号
uint8 spi_comm_read_beacon(beacon_result_t *result);  // 读取灯塔数据
void spi_comm_test(void);              // 测试函数
```

**main_cm7_0.c 主循环:**
```c
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    // 初始化SPI主机
    spi_comm_init();
    printf("4BB7 SPI Master Ready\r\n");

    while(true)
    {
        // SPI通信测试
        spi_comm_test();
    }
}
```

### 5.2 CYT2BL3软件架构

#### 5.2.1 实际目录结构

```
2bl3/4bb7_2bl3_spi/
├── code/
│   ├── spi_slave.c         # SPI从机通信实现
│   ├── spi_slave.h         # SPI从机通信头文件
│   └── 本文件夹作用.txt
├── user/
│   ├── main_cm4.c          # 主程序
│   └── cm4_isr.c           # 中断处理
└── libraries/               # 逐飞库
```

#### 5.2.2 主要函数（实际实现）

**spi_slave.h 关键定义:**
```c
// 硬件配置
#define SPI_SLAVE_SCB       SCB0
#define SPI_SLAVE_MISO      P0_0
#define SPI_SLAVE_MOSI      P0_1
#define SPI_SLAVE_CLK       P0_2
#define SPI_SLAVE_SS        P0_3
#define SPI_SLAVE_INT       P18_6

// 函数声明
void spi_slave_init(void);                            // 初始化SPI从机
void spi_slave_set_int(uint8 level);                  // 设置INT信号
void spi_slave_update_beacon(const beacon_result_t *result);  // 更新灯塔数据
void spi_slave_task(void);                            // 从机任务处理
```

**main_cm4.c 主循环:**
```c
// SCB0资源冲突解决方案
#define ENABLE_DEBUG_UART   0   // 禁用debug串口，SPI从机独占SCB0

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);

#if ENABLE_DEBUG_UART
    debug_init();  // 与SPI从机冲突！
#endif

    spi_slave_init();

    // 设置初始测试数据
    beacon_result_t test_data = {
        .center_x = 160,
        .center_y = 120,
        .found = 1,
        .confidence = 85
    };
    spi_slave_update_beacon(&test_data);

    while(true)
    {
        spi_slave_task();  // SS边沿检测和INT管理

        // 周期性更新测试数据
        // ...
    }
}
```

#### 5.2.3 SPI从机核心实现

**引脚初始化 (spi_slave.c):**
```c
static void spi_slave_init_pins(void)
{
    cy_stc_gpio_pin_config_t pin_cfg = {0};

    // MISO (P0_0) - 从机输出 (强驱动)
    pin_cfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pin_cfg.hsiom = P0_0_SCB0_SPI_MISO;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 0, &pin_cfg);

    // MOSI (P0_1) - 从机输入 (高阻)
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P0_1_SCB0_SPI_MOSI;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 1, &pin_cfg);

    // CLK (P0_2) - 从机输入 (高阻)
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P0_2_SCB0_SPI_CLK;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 2, &pin_cfg);

    // SS (P0_3) - 硬件片选输入 (使用SCB硬件功能)
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P0_3_SCB0_SPI_SELECT0;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 3, &pin_cfg);

    // INT (P18_6) - GPIO输出
    gpio_init(SPI_SLAVE_INT, GPO, GPIO_LOW, GPO_PUSH_PULL);
}
```

**TX FIFO预加载策略 (spi_slave.c):**
```c
static void preload_tx_fifo(void)
{
    Cy_SCB_SPI_ClearTxFifo(SPI_SLAVE_SCB);
    for (uint8 i = 0; i < BEACON_RESPONSE_LEN; i++)
    {
        Cy_SCB_WriteTxFifo(SPI_SLAVE_SCB, g_tx_buffer[i]);
    }
}

void spi_slave_update_beacon(const beacon_result_t *result)
{
    if (result == NULL) return;

    g_beacon_data = *result;
    build_beacon_response(g_tx_buffer);  // 构建响应帧
    preload_tx_fifo();                   // 预加载到TX FIFO
    g_data_pending = 1;
    spi_slave_set_int(1);                // 拉高INT通知主机
}
```

**SS边沿检测 (spi_slave.c):**
```c
void spi_slave_task(void)
{
    static uint8 ss_last = 1;
    uint8 ss_now = Cy_GPIO_Read(GPIO_PRT0, 3);

    // 检测SS下降沿 (主机开始通信)
    if (ss_last == 1 && ss_now == 0)
    {
        Cy_SCB_SPI_ClearRxFifo(SPI_SLAVE_SCB);
    }

    // 检测SS上升沿 (传输完成)
    if (ss_last == 0 && ss_now == 1)
    {
        if (g_data_pending)
        {
            g_data_pending = 0;
            spi_slave_set_int(0);    // 拉低INT
            preload_tx_fifo();       // 重新预加载
        }
    }

    ss_last = ss_now;
}
```

---

## 六、开发阶段规划

### 6.1 第一阶段：基础通信验证（中断模式）✅ **已完成**

**目标**：1个4BB7 + 1个2BL3实现基础SPI通信

**任务**：
- [x] 4BB7 SPI主机驱动开发（单CS，中断模式）
- [x] 2BL3 SPI从机驱动开发（SCB0，中断模式）
- [x] 通信协议基础实现（帧头帧尾、CRC16）
- [x] INT信号机制实现（TX FIFO预加载策略）
- [x] 简单数据收发测试

**验收结果**：
- ✅ 主机能正确发送请求
- ✅ 从机能正确响应（数据：x=160,y=120,found=1,conf=85）
- ✅ INT信号触发正常
- ✅ 数据变化规律符合预期（x+10 mod 320, y+5 mod 240）
- ✅ CRC校验正确

**解决的问题**：
1. **SCB0资源冲突**：Debug UART与SPI从机均使用SCB0，通过ENABLE_DEBUG_UART宏解决
2. **TX FIFO竞态**：SS下降沿时TX FIFO可能未准备好，通过预加载策略解决
3. **帧头错误(err=2)**：硬件接线问题，检查SS/CS连接后解决

### 6.2 第二阶段：主机DMA优化 ⏳ 待开始

**目标**：4BB7主机端使用PDMA传输，从机保持中断模式

**设计理由**：
- 先在单从机环境下验证DMA，变量最少，便于调试
- DMA是底层变化，先稳定底层再扩展上层
- 主机端DMA配置与从机数量无关，验证后可直接复用到多从机场景
- 从机端对DMA完全透明，无需任何修改

**当前代码分析**（需要改造的部分）：
```c
// 当前 spi_comm.c:162 - 阻塞式传输
spi_transfer_8bit(SPI_MASTER_CH, tx_buf, rx_buf, transfer_len);
// ↑ CPU在此等待整个传输完成（约112μs @ 1MHz）

// DMA优化后 - 非阻塞式传输
spi_transfer_dma(tx_buf, rx_buf, transfer_len, on_complete_callback);
// ↑ CPU启动DMA后立即返回，完成后通过中断通知
```

**任务**：
- [ ] 配置PDMA通道（TX: 1个通道，RX: 1个通道）
- [ ] 配置DMA描述符（源地址、目的地址、传输长度）
- [ ] 实现DMA传输启动函数 `spi_transfer_dma()`
- [ ] 实现DMA完成中断处理 `DMA_Complete_ISR()`
- [ ] 修改 `spi_comm_read_beacon()` 为异步模式（回调机制）
- [ ] 修改 `spi_comm_test()` 适配异步API
- [ ] 中断模式与DMA模式对比测试（CPU占用率、稳定性）

**涉及的CYT4BB7资源**：
| 资源 | 配置 | 说明 |
|------|------|------|
| PDMA通道 | 2个（TX+RX） | SCB7的DMA触发信号 |
| DMA描述符 | 2个 | 存放在RAM中 |
| 中断 | DMA完成中断 | 替代原有轮询 |

**API变化设计**：
```c
// 新增异步接口
typedef void (*spi_callback_t)(uint8 error_code, beacon_result_t *result);
void spi_comm_read_beacon_async(spi_callback_t callback);

// 保留同步接口（内部使用标志位等待）
uint8 spi_comm_read_beacon(beacon_result_t *result);  // 兼容现有代码
```

**验收标准**：
- DMA传输数据正确（CRC校验通过）
- DMA完成中断正常触发
- CPU占用率降低（传输期间可执行其他代码）
- 与中断模式性能对比数据

### 6.3 第三阶段：从机DMA优化（可选）⏳ 待开始

**目标**：2BL3从机端使用DMA传输

**设计理由**：
- 从机端DMA收益相对较小（每次仅14字节，约112μs）
- 从机端有特殊挑战：SS信号同步问题
- 建议先评估主机DMA收益后，再决定是否实施

**当前代码分析**（需要改造的部分）：
```c
// 当前 spi_slave.c:148-156 - CPU逐字节加载TX FIFO
static void preload_tx_fifo(void)
{
    Cy_SCB_SPI_ClearTxFifo(SPI_SLAVE_SCB);
    for (uint8 i = 0; i < BEACON_RESPONSE_LEN; i++)
    {
        Cy_SCB_WriteTxFifo(SPI_SLAVE_SCB, g_tx_buffer[i]);
    }
}

// DMA优化后 - DMA自动加载
static void preload_tx_fifo_dma(void)
{
    Cy_PDMA_Channel_SetDescriptor(TX_DMA_CH, &tx_descriptor);
    Cy_PDMA_Channel_Enable(TX_DMA_CH);
    // DMA自动将g_tx_buffer搬运到TX FIFO
}
```

**从机DMA的特殊挑战**：
```
问题：SS信号由主机控制，从机无法预知传输开始时刻

方案A：TX预加载 + RX用DMA
  - TX仍使用现有的FIFO预加载策略（简单可靠）
  - RX使用DMA接收主机请求
  - 收益：仅节省RX的CPU时间

方案B：SS下降沿触发DMA
  - 配置SS引脚中断（下降沿）
  - 中断中启动TX/RX DMA
  - 风险：中断延迟可能导致第一个字节丢失

推荐：方案A（TX预加载 + RX DMA），风险最低
```

**任务**：
- [ ] 评估主机DMA优化后的整体性能
- [ ] 决定是否需要从机DMA优化
- [ ] （如需要）配置DW通道（TX: 1个，RX: 1个）
- [ ] （如需要）实现RX DMA接收
- [ ] （如需要）测试SS信号同步

**验收标准**：
- 从机CPU占用率评估
- 与纯中断模式性能对比
- SS信号同步无丢字节

### 6.4 第四阶段：多从机扩展 ⏳ 待开始

**目标**：扩展到3个CS控制3个从机

**设计理由**：
- DMA接口已稳定，多从机管理直接基于DMA接口设计
- 避免"先写同步代码，后改异步"的重构成本
- 每个阶段只改变一个维度（此阶段只增加从机数量）

**当前代码分析**（需要扩展的部分）：
```c
// 当前 spi_comm.h - 单从机配置
#define SPI_CS_PIN          P02_3
#define SPI_INT_PIN         P02_4

// 扩展后 - 多从机配置
#define SLAVE_COUNT         3
#define SPI_CS_PINS         {P02_3, P01_0, P19_0}
#define SPI_INT_PINS        {P02_4, P01_1, P19_1}
```

**任务**：
- [ ] 定义从机ID枚举 `slave_id_t`
- [ ] 添加GPIO CS引脚数组（P02_3, P01_0, P19_0）
- [ ] 添加GPIO INT引脚数组（P02_4, P01_1, P19_1）
- [ ] 实现从机管理模块 `slave_manager.c/h`
- [ ] 实现事件驱动调度（INT触发 → 加入队列 → 依次通信）
- [ ] 实现 `slave_manager_process_all()` 轮询函数
- [ ] 3从机通信测试

**从机管理模块设计**：
```c
// slave_manager.h
typedef enum { SLAVE_1 = 0, SLAVE_2, SLAVE_3 } slave_id_t;

typedef struct {
    uint8 online;
    uint8 data_ready;
    beacon_result_t beacon;
} slave_status_t;

void slave_manager_init(void);
void slave_manager_task(void);  // 主循环调用
slave_status_t* slave_manager_get_status(slave_id_t id);
```

**验收标准**：
- 3个CS线能独立控制
- 3个INT信号能正确检测
- 事件驱动通信正常
- 各从机数据独立无干扰

### 6.5 第五阶段：功能完善 ⏳ 待开始

**目标**：完成2BL3端的显示和菜单功能

**任务**：
- [ ] 移植IPS114屏幕驱动
- [ ] 移植菜单系统框架（暂不含参数菜单）
- [ ] 屏幕显示测试
- [ ] 恢复debug串口功能（迁移到其他SCB）

**验收标准**：
- 屏幕显示正常
- 菜单操作正常
- debug串口与SPI从机共存

### 6.6 第六阶段：集成测试 ⏳ 待开始

**目标**：系统联调

**任务**：
- [ ] 3个2BL3同时运行测试
- [ ] 100Hz通信稳定性测试
- [ ] 长时间稳定性测试（≥24小时）
- [ ] 异常处理测试（从机掉线、通信错误恢复）
- [ ] 性能优化（根据测试结果调整）

**验收标准**：
- 通信成功率 >99.9%
- 通信频率稳定 100Hz±5%
- 响应延迟 <1ms
- 连续运行 ≥24小时无故障

---

## 七、测试方案

### 7.1 第一阶段测试结果

| 测试项 | 测试方法 | 结果 |
|--------|---------|------|
| SPI主机发送 | 串口打印 | ✅ 通过 |
| SPI从机接收 | 逻辑分析仪 | ✅ 通过 |
| INT信号触发 | 串口打印INT=0/1 | ✅ 通过 |
| CRC校验 | 对比计算值 | ✅ 通过 |
| 数据解析 | 串口打印beacon | ✅ 通过 |

**测试输出样例**:
```
4BB7 SPI Master Ready
Beacon: x=160, y=120, found=1, conf=85
Beacon: x=170, y=125, found=1, conf=90
Beacon: x=180, y=130, found=1, conf=80
...
```

### 7.2 待完成的测试

| 测试项 | 测试方法 | 状态 |
|--------|---------|------|
| 单从机长时间运行 | 持续通信1小时 | ⏳ 待测 |
| 多从机轮询 | 3从机同时运行 | ⏳ 待测 |
| 100Hz频率测试 | 监测通信频率 | ⏳ 待测 |
| DMA传输稳定性 | DMA模式运行 | ⏳ 待测 |

### 7.3 验收标准

| 测试项 | 验收标准 | 当前状态 |
|--------|---------|----------|
| 通信成功率 | >99.9% | ⏳ 待测 |
| 通信频率 | 100Hz±5% | ⏳ 待测 |
| 响应延迟 | <1ms | ⏳ 待测 |
| 稳定运行时间 | ≥24小时 | ⏳ 待测 |

---

## 八、已知问题与解决方案

### 8.1 已解决的问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| SCB0资源冲突 | Debug UART和SPI从机均使用SCB0 | 添加ENABLE_DEBUG_UART宏，禁用debug串口 |
| TX FIFO竞态 | SS下降沿时TX数据未准备好 | 采用预加载策略，数据更新后立即加载TX FIFO |
| 帧头错误(err=2) | 硬件SS/CS连接问题 | 检查并修复硬件接线 |
| INT信号未清除 | 缺少SS上升沿检测 | 在spi_slave_task()中检测SS上升沿并清除INT |

### 8.2 待解决的问题

| 问题 | 影响 | 计划解决方案 |
|------|------|-------------|
| 2BL3无调试串口 | 调试困难 | 将debug串口迁移到其他SCB（如SCB5/P7端口） |
| 多从机支持 | 功能不完整 | 第二阶段实现 |

---

## 附录

### A. 实际代码文件清单

| 功能 | 文件路径 | 行数 |
|------|---------|------|
| 主机通信模块 | `4bb7/4bb7_2bl3_spi/code/spi_comm.c` | ~213行 |
| 主机通信头文件 | `4bb7/4bb7_2bl3_spi/code/spi_comm.h` | ~96行 |
| 从机通信模块 | `2bl3/4bb7_2bl3_spi/code/spi_slave.c` | ~256行 |
| 从机通信头文件 | `2bl3/4bb7_2bl3_spi/code/spi_slave.h` | ~73行 |
| 主机主程序 | `4bb7/4bb7_2bl3_spi/user/main_cm7_0.c` | - |
| 从机主程序 | `2bl3/4bb7_2bl3_spi/user/main_cm4.c` | ~78行 |

### B. 参考代码位置

| 功能 | 参考代码路径 |
|------|-------------|
| SPI主机(4BB7) | `mknm_car_new/libraries/zf_driver/zf_driver_spi.c` |
| SPI主机(2BL3) | `4bb7_2bl3_spi/2bl3/libraries/zf_driver/zf_driver_spi.c` |
| SCB SPI底层 | `libraries/sdk/common/src/drivers/scb/cy_scb_spi.c` |
| 菜单系统 | `mknm_car_new/mknm_car/code/menu/` |

### C. 官方参考资源

- [Infineon SPI Master with DMA Example](https://github.com/Infineon/mtb-t2g-lite-example-spi-master-dma)
- [AN225401 - How to use SCB in TRAVEO T2G](https://documentation.infineon.com/traveo/docs/qmr1680597505537)
- [CYT4BB Datasheet](https://www.infineon.com/cms/en/product/microcontroller/32-bit-traveo-t2g-arm-cortex-microcontroller/)
- [CYT2BL Datasheet](https://www.infineon.com/cms/en/product/microcontroller/32-bit-traveo-t2g-arm-cortex-microcontroller/)

### D. 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2024-12 | 初版实施计划 |
| v2.0 | 2024-12 | 添加INT信号机制 |
| v3.0 | 2024-12 | 确认SCB0为SPI从机 |
| v3.1 | 2024-12 | INT信号时序优化 |
| **v4.0** | **2025-12** | **第一阶段验证完成，更新为实际实现的代码和架构** |
