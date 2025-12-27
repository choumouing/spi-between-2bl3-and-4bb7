/*********************************************************************************************************************
 * @file    spi_comm.c
 * @brief   SPI通信模块 - 主机端(4BB7)
 * @details 采用FIFO批量填充方式实现高效SPI传输，支持3个从机
 *          - 利用SCB7的256字节FIFO深度，一次性填充所有TX数据
 *          - SPI硬件自动完成全双工传输
 *          - 传输完成后批量读取RX数据
 *          - 事件驱动：检测各从机INT信号，有就绪的就通信
 ********************************************************************************************************************/

#include "spi_comm.h"
#include "zf_common_headfile.h"
#include "scb/cy_scb_spi.h"
#include <string.h>

//==================================================== 超时配置 ====================================================
// 超时计数 (约10ms @ 250MHz，循环开销约10-20周期)
#define TRANSFER_TIMEOUT_COUNT  100000

//==================================================== 引脚配置表 ====================================================
// 各从机的CS引脚
static const gpio_pin_enum g_cs_pins[SLAVE_COUNT] = {
    SPI_CS1_PIN,    // 从机1: P02_3
    SPI_CS2_PIN,    // 从机2: P01_0
    SPI_CS3_PIN     // 从机3: P19_0
};

// 各从机的INT引脚
static const gpio_pin_enum g_int_pins[SLAVE_COUNT] = {
    SPI_INT1_PIN,   // 从机1: P02_4
    SPI_INT2_PIN,   // 从机2: P01_1
    SPI_INT3_PIN    // 从机3: P19_1
};

//==================================================== 内部变量 ====================================================
// 各从机状态
static slave_status_t g_slave_status[SLAVE_COUNT];

//==================================================== 内部函数 ====================================================

/**
 * @brief  计算CRC16校验值 (Modbus CRC16)
 * @param  data 数据指针
 * @param  len  数据长度
 * @return CRC16校验值
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
    buffer[5] = (uint8)(crc & 0xFF);
    buffer[6] = (uint8)(crc >> 8);
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
 */
static uint8 parse_response_frame(const uint8 *rx_buf, uint8 len, uint8 *data_out, uint8 *data_len)
{
    uint16 crc_calc, crc_recv;
    uint16 payload_len;

    if (len < FRAME_OVERHEAD)
        return SPI_ERR_FRAME_SHORT;

    if (rx_buf[0] != FRAME_HEAD_1 || rx_buf[1] != FRAME_HEAD_2)
        return SPI_ERR_INVALID_HEAD;

    // 提取数据长度 (大端序: 高字节在前)
    payload_len = ((uint16)rx_buf[3] << 8) | rx_buf[4];
    if (payload_len > MAX_DATA_SIZE)
        return SPI_ERR_PAYLOAD_LONG;

    if (len < FRAME_OVERHEAD + payload_len)
        return SPI_ERR_INCOMPLETE;

    if (rx_buf[7 + payload_len] != FRAME_TAIL)
        return SPI_ERR_INVALID_TAIL;

    // CRC校验
    crc_calc = calc_crc16(&rx_buf[2], 3 + payload_len);
    crc_recv = rx_buf[5 + payload_len] | ((uint16)rx_buf[6 + payload_len] << 8);
    if (crc_calc != crc_recv)
        return SPI_ERR_CRC_MISMATCH;

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
 */
static uint8 spi_transfer_batch(const uint8 *tx_data, uint8 *rx_data, uint8 len)
{
    uint32 timeout;

    if (len == 0 || len > MAX_FRAME_SIZE)
        return SPI_ERR_DATA_SIZE;

    // 清空SPI FIFO
    Cy_SCB_SPI_ClearRxFifo(SCB7);
    Cy_SCB_SPI_ClearTxFifo(SCB7);

    // 批量填充TX FIFO
    for (uint8 i = 0; i < len; i++)
    {
        Cy_SCB_WriteTxFifo(SCB7, tx_data[i]);
    }

    // 等待SPI传输完成
    timeout = TRANSFER_TIMEOUT_COUNT;
    while (!Cy_SCB_IsTxComplete(SCB7))
    {
        if (--timeout == 0)
            return SPI_ERR_TIMEOUT;
    }

    // 等待所有RX数据接收完毕
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
    // 初始化SPI主机 (不使用硬件CS，使用GPIO软件控制)
    spi_init(SPI_MASTER_CH, SPI_MODE0, SPI_MASTER_BAUD,
             SPI_MASTER_CLK, SPI_MASTER_MOSI, SPI_MASTER_MISO, SPI_CS_NULL);

    // 初始化所有从机的CS引脚 (输出高电平，空闲状态)
    for (uint8 i = 0; i < SLAVE_COUNT; i++)
    {
        gpio_init(g_cs_pins[i], GPO, GPIO_HIGH, GPO_PUSH_PULL);
    }

    // 初始化所有从机的INT检测引脚 (输入，下拉)
    for (uint8 i = 0; i < SLAVE_COUNT; i++)
    {
        gpio_init(g_int_pins[i], GPI, GPIO_LOW, GPI_PULL_DOWN);
    }

    // 初始化从机状态
    memset(g_slave_status, 0, sizeof(g_slave_status));
}

uint8 spi_comm_data_ready(slave_id_t id)
{
    if (id >= SLAVE_COUNT)
        return 0;

    return gpio_get_level(g_int_pins[id]);
}

uint8 spi_comm_read_beacon(slave_id_t id, beacon_result_t *result)
{
    uint8 tx_buf[MAX_FRAME_SIZE];
    uint8 rx_buf[MAX_FRAME_SIZE];
    uint8 data_buf[MAX_DATA_SIZE];
    uint8 data_len;
    uint8 tx_len;
    uint8 transfer_len;
    uint8 ret;
    gpio_pin_enum cs_pin;

    // 参数检查
    if (id >= SLAVE_COUNT)
        return SPI_ERR_INVALID_SLAVE;
    if (result == NULL)
        return SPI_ERR_NULL_PTR;

    cs_pin = g_cs_pins[id];

    // 构建请求帧
    tx_len = build_request_frame(CMD_GET_BEACON, tx_buf);

    // 计算传输长度
    transfer_len = (tx_len > BEACON_RESPONSE_LEN) ? tx_len : BEACON_RESPONSE_LEN;

    // 填充发送缓冲区
    memset(&tx_buf[tx_len], 0xFF, transfer_len - tx_len);

    // 拉低CS，选中从机
    gpio_low(cs_pin);
    system_delay_us(SPI_CS_SETUP_DELAY_US);

    // 执行SPI传输
    ret = spi_transfer_batch(tx_buf, rx_buf, transfer_len);

    // 拉高CS，释放从机
    gpio_high(cs_pin);

    if (ret != SPI_ERR_OK)
        return ret;

    // 解析响应帧
    ret = parse_response_frame(rx_buf, transfer_len, data_buf, &data_len);
    if (ret != SPI_ERR_OK)
        return ret;

    if (data_len != BEACON_DATA_SIZE)
        return SPI_ERR_DATA_SIZE;

    // 解析灯塔数据 (小端序)
    result->center_x   = (int16)((uint16)data_buf[0] | ((uint16)data_buf[1] << 8));
    result->center_y   = (int16)((uint16)data_buf[2] | ((uint16)data_buf[3] << 8));
    result->found      = data_buf[4];
    result->confidence = data_buf[5];

    return SPI_ERR_OK;
}

const slave_status_t* spi_comm_get_status(slave_id_t id)
{
    if (id >= SLAVE_COUNT)
        return NULL;

    return &g_slave_status[id];
}

void spi_comm_task(void)
{
    beacon_result_t beacon;

    // 轮询所有从机，检测INT信号
    for (uint8 i = 0; i < SLAVE_COUNT; i++)
    {
        if (spi_comm_data_ready((slave_id_t)i))
        {
            uint8 ret = spi_comm_read_beacon((slave_id_t)i, &beacon);
            if (ret == SPI_ERR_OK)
            {
                // 更新从机状态
                g_slave_status[i].online = 1;
                g_slave_status[i].beacon = beacon;

                printf("S%d: x=%d, y=%d, found=%d, conf=%d\r\n",
                       i + 1, beacon.center_x, beacon.center_y,
                       beacon.found, beacon.confidence);
            }
            else
            {
                printf("S%d: err=%d\r\n", i + 1, ret);
            }
        }
    }
}
