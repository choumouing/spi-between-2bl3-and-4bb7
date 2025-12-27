/*********************************************************************************************************************
 * @file    spi_comm.h
 * @brief   SPI通信模块 - 主机端(4BB7)头文件
 * @details 实现与3个CYT2BL3从机的SPI通信
 *          - 单SPI总线 + GPIO软件CS控制
 *          - 事件驱动：INT信号触发通信
 *          - FIFO批量填充传输方式
 ********************************************************************************************************************/

#ifndef _SPI_COMM_H_
#define _SPI_COMM_H_

#include "zf_common_typedef.h"

//==================================================== 从机配置 ====================================================
// 从机数量
#define SLAVE_COUNT         3

// 从机ID定义
typedef enum
{
    SLAVE_1 = 0,    // 从机1
    SLAVE_2 = 1,    // 从机2
    SLAVE_3 = 2,    // 从机3
} slave_id_t;

//==================================================== 硬件配置 ====================================================
// SPI主机配置 (4BB7使用SCB7, 对应SPI_0)
#define SPI_MASTER_CH       SPI_0
#define SPI_MASTER_BAUD     (1000000)           // 1MHz波特率
#define SPI_MASTER_CLK      SPI0_CLK_P02_2
#define SPI_MASTER_MOSI     SPI0_MOSI_P02_1
#define SPI_MASTER_MISO     SPI0_MISO_P02_0

// 从机1引脚 (P02端口)
#define SPI_CS1_PIN         P02_3
#define SPI_INT1_PIN        P02_4

// 从机2引脚 (P01端口)
#define SPI_CS2_PIN         P01_0
#define SPI_INT2_PIN        P01_1

// 从机3引脚 (P19端口)
#define SPI_CS3_PIN         P19_0
#define SPI_INT3_PIN        P19_1

//==================================================== 协议定义 ====================================================
// NOTE: 以下协议定义需与从机端(spi_slave.h)保持一致
// 帧结构: HEAD(2) + CMD(1) + LEN(2) + DATA(n) + CRC(2) + TAIL(1)
#define FRAME_HEAD_1        0xAA
#define FRAME_HEAD_2        0x55
#define FRAME_TAIL          0xED

// 命令定义 (仅定义当前实现的命令，遵循YAGNI原则)

#define CMD_GET_BEACON      0x10                // 获取灯塔数据

// 帧长度限制
#define MAX_DATA_SIZE       32                  // 最大数据长度
#define FRAME_OVERHEAD      8                   // 帧开销: HEAD(2)+CMD(1)+LEN(2)+CRC(2)+TAIL(1)
#define MAX_FRAME_SIZE      (MAX_DATA_SIZE + FRAME_OVERHEAD)
#define BEACON_DATA_SIZE    6                   // 灯塔数据大小
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
#define SPI_ERR_TIMEOUT         8               // 传输超时
#define SPI_ERR_DATA_SIZE       9               // 数据大小不匹配
#define SPI_ERR_INVALID_SLAVE   10              // 无效的从机ID

// CS延时参数 (微秒)
#define SPI_CS_SETUP_DELAY_US   10               // CS拉低后到传输开始的延时

//==================================================== 数据结构 ====================================================
// 灯塔识别结果
typedef struct
{
    int16 center_x;                             // 灯塔中心X坐标
    int16 center_y;                             // 灯塔中心Y坐标
    uint8 found;                                // 是否找到灯塔
    uint8 confidence;                           // 置信度(0-100)
} beacon_result_t;

// 从机状态
typedef struct
{
    uint8 online;                               // 是否在线 (有成功通信过)
    beacon_result_t beacon;                     // 灯塔数据
} slave_status_t;

//==================================================== 函数声明 ====================================================
/**
 * @brief  初始化SPI通信模块 (包括所有从机的CS/INT引脚)
 */
void spi_comm_init(void);

/**
 * @brief  检测指定从机的INT信号
 * @param  id 从机ID (SLAVE_1, SLAVE_2, SLAVE_3)
 * @return 1-数据就绪, 0-未就绪
 */
uint8 spi_comm_data_ready(slave_id_t id);

/**
 * @brief  从指定从机读取灯塔数据
 * @param  id     从机ID
 * @param  result 灯塔数据输出指针
 * @return 0-成功, 非0-失败
 */
uint8 spi_comm_read_beacon(slave_id_t id, beacon_result_t *result);

/**
 * @brief  获取指定从机的状态
 * @param  id 从机ID
 * @return 从机状态指针 (只读)
 */
const slave_status_t* spi_comm_get_status(slave_id_t id);

/**
 * @brief  从机管理任务 (主循环调用)
 * @note   轮询所有从机INT信号，有就绪的从机则读取数据
 */
void spi_comm_task(void);

#endif // _SPI_COMM_H_
