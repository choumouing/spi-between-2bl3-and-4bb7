/*********************************************************************************************************************
 * @file    spi_comm.h
 * @brief   SPI通信模块 - 主机端(4BB7)头文件 - DMA优化版
 * @details 实现与CYT2BL3从机的SPI通信，使用DW1 PDMA
 ********************************************************************************************************************/

#ifndef _SPI_COMM_H_
#define _SPI_COMM_H_

#include "zf_common_typedef.h"

//==================================================== 硬件配置 ====================================================
// SPI主机配置 (4BB7使用SCB7, 对应SPI_0)
#define SPI_MASTER_CH       SPI_0
#define SPI_MASTER_BAUD     (1000000)           // 1MHz波特率
#define SPI_MASTER_CLK      SPI0_CLK_P02_2
#define SPI_MASTER_MOSI     SPI0_MOSI_P02_1
#define SPI_MASTER_MISO     SPI0_MISO_P02_0

// CS引脚 (GPIO软件控制)
#define SPI_CS_PIN          P02_3

// INT引脚 (检测从机数据就绪, 高电平有效)
#define SPI_INT_PIN         P02_4

//==================================================== 协议定义 ====================================================
// NOTE: 以下协议定义需与从机端(spi_slave.h)保持一致
// 帧结构: HEAD(2) + CMD(1) + LEN(2) + DATA(n) + CRC(2) + TAIL(1)
#define FRAME_HEAD_1        0xAA
#define FRAME_HEAD_2        0x55
#define FRAME_TAIL          0xED

// 命令定义
#define CMD_PING            0x01                // 心跳测试
#define CMD_GET_BEACON      0x10                // 获取灯塔数据

// 帧长度限制
#define MAX_DATA_SIZE       32                  // 最大数据长度
#define FRAME_OVERHEAD      8                   // 帧开销: HEAD(2)+CMD(1)+LEN(2)+CRC(2)+TAIL(1)
#define MAX_FRAME_SIZE      (MAX_DATA_SIZE + FRAME_OVERHEAD)
#define BEACON_DATA_SIZE    6                   // 灯塔数据大小 (不依赖编译器对齐)
#define BEACON_RESPONSE_LEN 14                  // 灯塔响应帧长度: 8+6

// 错误码定义
#define SPI_ERR_OK              0               // 成功
#define SPI_ERR_FRAME_SHORT     1               // 帧长度不足
#define SPI_ERR_INVALID_HEAD    2               // 帧头错误
#define SPI_ERR_PAYLOAD_LONG    3               // 数据长度超限
#define SPI_ERR_INVALID_TAIL    4               // 帧尾错误
#define SPI_ERR_CRC_MISMATCH    5               // CRC校验失败
#define SPI_ERR_INCOMPLETE      6               // 帧不完整
#define SPI_ERR_NULL_PTR        7               // 空指针参数
#define SPI_ERR_TIMEOUT         8               // DMA传输超时
#define SPI_ERR_DMA_INIT        9               // DMA初始化失败
#define SPI_ERR_DATA_SIZE       10              // 数据大小不匹配

// CS延时参数 (微秒)
#define SPI_CS_SETUP_DELAY_US   5               // CS拉低后到传输开始的延时

// DMA超时计数
#define DMA_TIMEOUT_COUNT       1000000         // DMA传输超时计数

// 状态打印间隔 (循环次数)
#define STATUS_PRINT_INTERVAL   1000000

//==================================================== 数据结构 ====================================================
// 灯塔识别结果
typedef struct
{
    int16 center_x;                             // 灯塔中心X坐标
    int16 center_y;                             // 灯塔中心Y坐标
    uint8 found;                                // 是否找到灯塔
    uint8 confidence;                           // 置信度(0-100)
} beacon_result_t;

//==================================================== 函数声明 ====================================================
/**
 * @brief  初始化SPI通信模块
 */
void spi_comm_init(void);

/**
 * @brief  检测从机数据是否就绪 (INT信号)
 * @return 1-就绪, 0-未就绪
 */
uint8 spi_comm_data_ready(void);

/**
 * @brief  读取灯塔数据
 * @param  result 灯塔数据输出指针
 * @return 0-成功, 非0-失败
 */
uint8 spi_comm_read_beacon(beacon_result_t *result);

/**
 * @brief  SPI通信测试函数 (主循环调用)
 */
void spi_comm_test(void);

#endif // _SPI_COMM_H_
