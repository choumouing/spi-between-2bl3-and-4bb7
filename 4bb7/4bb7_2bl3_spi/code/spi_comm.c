/*********************************************************************************************************************
 * @file    spi_comm.c
 * @brief   SPI通信模块 - 主机端(4BB7)
 * @details 采用FIFO批量填充方式实现高效SPI传输
 *          - 利用SCB7的256字节FIFO深度，一次性填充所有TX数据
 *          - SPI硬件自动完成全双工传输
 *          - 传输完成后批量读取RX数据
 *
 *          设计依据：
 *          - 单次传输数据量14字节，远小于FIFO深度(256字节)
 *          - @1MHz传输仅需112μs，DMA收益可忽略
 *          - 遵循KISS/YAGNI原则，简化设计
 ********************************************************************************************************************/

#include "spi_comm.h"
#include "zf_common_headfile.h"
#include "scb/cy_scb_spi.h"
#include <string.h>

//==================================================== 超时配置 ====================================================
// 超时计数 (约10ms @ 250MHz，循环开销约10-20周期)
#define TRANSFER_TIMEOUT_COUNT  100000

//==================================================== 内部函数 ====================================================

/**
 * @brief  计算CRC16校验值 (Modbus CRC16)
 * @param  data 数据指针
 * @param  len  数据长度
 * @return CRC16校验值
 *
 * @note   算法: 多项式0xA001, 初始值0xFFFF
 */
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

/**
 * @brief  构建请求帧
 * @param  cmd     命令字节
 * @param  buffer  输出缓冲区
 * @return 帧长度 (固定8字节)
 *
 * @note   帧结构: HEAD(2) + CMD(1) + LEN(2) + CRC(2) + TAIL(1) = 8字节
 */
static uint8 build_request_frame(uint8 cmd, uint8 *buffer)
{
    uint16 crc;

    buffer[0] = FRAME_HEAD_1;
    buffer[1] = FRAME_HEAD_2;
    buffer[2] = cmd;
    buffer[3] = 0;  // LEN高字节
    buffer[4] = 0;  // LEN低字节 (请求帧无数据)

    // CRC计算范围: CMD + LEN (3字节)
    crc = calc_crc16(&buffer[2], 3);
    buffer[5] = (uint8)(crc & 0xFF);        // CRC低字节
    buffer[6] = (uint8)(crc >> 8);          // CRC高字节
    buffer[7] = FRAME_TAIL;

    return 8;
}

/**
 * @brief  解析响应帧
 * @param  rx_buf    接收缓冲区
 * @param  len       接收长度
 * @param  data_out  数据输出缓冲区
 * @param  data_len  数据长度输出
 * @return 错误码
 *
 * @note   帧结构: HEAD(2) + CMD(1) + LEN(2) + DATA(n) + CRC(2) + TAIL(1)
 *         索引:   [0-1]    [2]      [3-4]    [5~5+n-1]  [5+n~6+n]  [7+n]
 */
static uint8 parse_response_frame(const uint8 *rx_buf, uint8 len, uint8 *data_out, uint8 *data_len)
{
    uint16 crc_calc, crc_recv;
    uint16 payload_len;

    // 检查最小帧长度
    if (len < FRAME_OVERHEAD)
        return SPI_ERR_FRAME_SHORT;

    // 检查帧头
    if (rx_buf[0] != FRAME_HEAD_1 || rx_buf[1] != FRAME_HEAD_2)
        return SPI_ERR_INVALID_HEAD;

    // 提取数据长度 (大端序: 高字节在前)
    payload_len = ((uint16)rx_buf[3] << 8) | rx_buf[4];
    if (payload_len > MAX_DATA_SIZE)
        return SPI_ERR_PAYLOAD_LONG;

    // 检查完整帧长度
    if (len < FRAME_OVERHEAD + payload_len)
        return SPI_ERR_INCOMPLETE;

    // 检查帧尾 (索引: 5 + payload_len + 2 = 7 + payload_len)
    if (rx_buf[7 + payload_len] != FRAME_TAIL)
        return SPI_ERR_INVALID_TAIL;

    // CRC校验 (计算范围: CMD + LEN + DATA)
    crc_calc = calc_crc16(&rx_buf[2], 3 + payload_len);
    crc_recv = rx_buf[5 + payload_len] | ((uint16)rx_buf[6 + payload_len] << 8);
    if (crc_calc != crc_recv)
        return SPI_ERR_CRC_MISMATCH;

    // 复制数据
    *data_len = (uint8)payload_len;
    if (payload_len > 0)
    {
        memcpy(data_out, &rx_buf[5], payload_len);
    }

    return SPI_ERR_OK;
}

/**
 * @brief  执行SPI全双工传输 (FIFO批量填充方式)
 * @param  tx_data  发送数据指针
 * @param  rx_data  接收数据指针
 * @param  len      传输长度
 * @return 0-成功, 非0-错误码
 *
 * @note   传输策略:
 *         1. 清空RX/TX FIFO
 *         2. 批量填充TX FIFO (利用256字节深度)
 *         3. 等待SPI硬件完成传输 (全双工，自动接收)
 *         4. 批量读取RX FIFO
 *
 *         性能分析 (@1MHz, 14字节):
 *         - 传输时间: 14 × 8 / 1MHz = 112μs
 *         - CPU等待时间: ~112μs (可接受)
 */
static uint8 spi_transfer_batch(const uint8 *tx_data, uint8 *rx_data, uint8 len)
{
    uint32 timeout;

    // 参数校验
    if (len == 0 || len > MAX_FRAME_SIZE)
        return SPI_ERR_DATA_SIZE;

    // 清空SPI FIFO
    Cy_SCB_SPI_ClearRxFifo(SCB7);
    Cy_SCB_SPI_ClearTxFifo(SCB7);

    // 批量填充TX FIFO
    // SCB7 FIFO深度256字节，14字节数据一次性填入
    for (uint8 i = 0; i < len; i++)
    {
        Cy_SCB_WriteTxFifo(SCB7, tx_data[i]);
    }

    // 等待SPI传输完成 (TX FIFO清空且移位寄存器空闲)
    timeout = TRANSFER_TIMEOUT_COUNT;
    while (!Cy_SCB_IsTxComplete(SCB7))
    {
        if (--timeout == 0)
            return SPI_ERR_TIMEOUT;
    }

    // 等待所有RX数据接收完毕
    // SPI全双工: 发送N字节必然接收N字节
    timeout = TRANSFER_TIMEOUT_COUNT;
    while (Cy_SCB_SPI_GetNumInRxFifo(SCB7) < len)
    {
        if (--timeout == 0)
            return SPI_ERR_TIMEOUT;
    }

    // 批量读取RX数据
    for (uint8 i = 0; i < len; i++)
    {
        rx_data[i] = (uint8)Cy_SCB_SPI_Read(SCB7);
    }

    return SPI_ERR_OK;
}

//==================================================== 接口实现 ====================================================

void spi_comm_init(void)
{
    // 初始化SPI主机 (CS使用GPIO软件控制)
    spi_init(SPI_MASTER_CH, SPI_MODE0, SPI_MASTER_BAUD,
             SPI_MASTER_CLK, SPI_MASTER_MOSI, SPI_MASTER_MISO, SPI_CS_NULL);

    // 初始化CS引脚 (输出高电平，空闲状态)
    gpio_init(SPI_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // 初始化INT检测引脚 (输入，检测从机数据就绪)
    gpio_init(SPI_INT_PIN, GPI, GPIO_LOW, GPI_PULL_DOWN);
}

uint8 spi_comm_data_ready(void)
{
    return gpio_get_level(SPI_INT_PIN);
}

uint8 spi_comm_read_beacon(beacon_result_t *result)
{
    uint8 tx_buf[MAX_FRAME_SIZE];
    uint8 rx_buf[MAX_FRAME_SIZE];
    uint8 data_buf[MAX_DATA_SIZE];
    uint8 data_len;
    uint8 tx_len;
    uint8 transfer_len;
    uint8 ret;

    // 空指针检查
    if (result == NULL)
        return SPI_ERR_NULL_PTR;

    // 构建请求帧
    tx_len = build_request_frame(CMD_GET_BEACON, tx_buf);

    // 计算传输长度 (取请求帧和响应帧的较大值)
    transfer_len = (tx_len > BEACON_RESPONSE_LEN) ? tx_len : BEACON_RESPONSE_LEN;

    // 填充发送缓冲区 (补齐到传输长度，0xFF为空闲字节)
    memset(&tx_buf[tx_len], 0xFF, transfer_len - tx_len);

    // 拉低CS，选中从机
    gpio_low(SPI_CS_PIN);

    // CS建立时间延时 (确保从机准备好)
    system_delay_us(SPI_CS_SETUP_DELAY_US);

    // 执行SPI全双工传输
    ret = spi_transfer_batch(tx_buf, rx_buf, transfer_len);

    // 拉高CS，释放从机
    gpio_high(SPI_CS_PIN);

    // 检查传输结果
    if (ret != SPI_ERR_OK)
        return ret;

    // 解析响应帧
    ret = parse_response_frame(rx_buf, transfer_len, data_buf, &data_len);
    if (ret != SPI_ERR_OK)
        return ret;

    // 验证数据长度
    if (data_len != BEACON_DATA_SIZE)
        return SPI_ERR_DATA_SIZE;

    // 解析灯塔数据 (小端序)
    result->center_x   = (int16)((uint16)data_buf[0] | ((uint16)data_buf[1] << 8));
    result->center_y   = (int16)((uint16)data_buf[2] | ((uint16)data_buf[3] << 8));
    result->found      = data_buf[4];
    result->confidence = data_buf[5];

    return SPI_ERR_OK;
}

void spi_comm_test(void)
{
    static uint32 loop_count = 0;
    beacon_result_t beacon;

    // 检测INT信号
    if (spi_comm_data_ready())
    {
        uint8 ret = spi_comm_read_beacon(&beacon);
        if (ret == SPI_ERR_OK)
        {
            printf("Beacon: x=%d, y=%d, found=%d, conf=%d\r\n",
                   beacon.center_x, beacon.center_y, beacon.found, beacon.confidence);
        }
        else
        {
            printf("Read err=%d\r\n", ret);
        }
    }

    // 周期性打印状态
    loop_count++;
    if (loop_count >= STATUS_PRINT_INTERVAL)
    {
        loop_count = 0;
        printf("INT=%d\r\n", spi_comm_data_ready());
    }
}
