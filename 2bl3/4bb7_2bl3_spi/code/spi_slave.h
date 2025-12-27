/*********************************************************************************************************************
 * @file    spi_slave.h
 * @brief   SPI从机通信模块 - 从机端(2BL3)头文件
 * @details 实现与CYT4BB7主机的SPI从机通信
 ********************************************************************************************************************/

#ifndef _SPI_SLAVE_H_
#define _SPI_SLAVE_H_

#include "zf_common_typedef.h"

//==================================================== 硬件配置 ====================================================
// SPI从机配置 (2BL3使用SCB0, 唯一具备完整SPI从机功能的SCB)
// 注意: SCB2的P14端口没有SPI_SELECT0的HSIOM定义，无法作为SPI从机使用硬件SS
#define SPI_SLAVE_SCB       SCB0

// 引脚定义 (P0端口, HSIOM=30 Deep Sleep模式)
#define SPI_SLAVE_MISO      P0_0        // 从机输出 (主机接收)
#define SPI_SLAVE_MOSI      P0_1        // 从机接收 (主机输出)
#define SPI_SLAVE_CLK       P0_2        // 时钟
#define SPI_SLAVE_SS        P0_3        // 硬件片选输入 (SCB0_SPI_SELECT0)
#define SPI_SLAVE_INT       P18_6       // 数据就绪信号 (GPIO输出)

//==================================================== 协议定义 (与主机一致) ====================================================
// NOTE: 以下协议定义需与主机端(spi_comm.h)保持一致
#define FRAME_HEAD_1        0xAA
#define FRAME_HEAD_2        0x55
#define FRAME_TAIL          0xED
#define CMD_GET_BEACON      0x10

#define MAX_DATA_SIZE       32
#define FRAME_OVERHEAD      8
#define MAX_FRAME_SIZE      (MAX_DATA_SIZE + FRAME_OVERHEAD)
#define BEACON_DATA_SIZE    6                   // 灯塔数据大小 (不依赖编译器对齐)
#define BEACON_RESPONSE_LEN 14                  // 灯塔响应帧长度: 8+6

//==================================================== 数据结构 ====================================================
// 灯塔识别结果
typedef struct
{
    int16 center_x;
    int16 center_y;
    uint8 found;
    uint8 confidence;
} beacon_result_t;

//==================================================== 函数声明 ====================================================
/**
 * @brief  初始化SPI从机通信模块
 */
void spi_slave_init(void);

/**
 * @brief  设置INT信号 (通知主机数据就绪)
 * @param  level 1-高电平(就绪), 0-低电平(空闲)
 */
void spi_slave_set_int(uint8 level);

/**
 * @brief  更新灯塔数据 (供上层调用)
 * @param  result 灯塔数据指针
 */
void spi_slave_update_beacon(const beacon_result_t *result);

/**
 * @brief  SPI从机任务处理 (主循环调用)
 */
void spi_slave_task(void);

#endif // _SPI_SLAVE_H_
