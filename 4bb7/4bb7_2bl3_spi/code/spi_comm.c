/*********************************************************************************************************************
 * @file    spi_comm.c
 * @brief   SPI通信模块 - 主机端(4BB7)
 * @details 第一阶段：基础通信验证（中断模式）
 ********************************************************************************************************************/

#include "spi_comm.h"
#include "zf_common_headfile.h"

//==================================================== 内部函数 ====================================================

/**
 * @brief  计算CRC16校验值 (Modbus CRC16)
 * @param  data 数据指针
 * @param  len  数据长度
 * @return CRC16值
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
 * @param  cmd      命令字
 * @param  buffer   输出缓冲区
 * @return 帧长度
 */
static uint8 build_request_frame(uint8 cmd, uint8 *buffer)
{
    uint16 crc;

    buffer[0] = FRAME_HEAD_1;
    buffer[1] = FRAME_HEAD_2;
    buffer[2] = cmd;
    buffer[3] = 0;  // LEN高字节
    buffer[4] = 0;  // LEN低字节 (请求帧无数据)

    crc = calc_crc16(&buffer[2], 3);
    buffer[5] = (uint8)(crc & 0xFF);
    buffer[6] = (uint8)(crc >> 8);
    buffer[7] = FRAME_TAIL;

    return 8;
}

/**
 * @brief  解析响应帧
 * @param  rx_buf   接收缓冲区
 * @param  len      接收长度
 * @param  data_out 输出数据指针
 * @param  data_len 输出数据长度
 * @return 0-成功, 非0-失败 (参见 SPI_ERR_* 定义)
 */
static uint8 parse_response_frame(const uint8 *rx_buf, uint8 len, uint8 *data_out, uint8 *data_len)
{
    uint16 crc_calc, crc_recv;
    uint16 payload_len;

    // 检查最小长度
    if (len < FRAME_OVERHEAD)
        return SPI_ERR_FRAME_SHORT;

    // 检查帧头
    if (rx_buf[0] != FRAME_HEAD_1 || rx_buf[1] != FRAME_HEAD_2)
        return SPI_ERR_INVALID_HEAD;

    // 获取数据长度
    payload_len = ((uint16)rx_buf[3] << 8) | rx_buf[4];
    if (payload_len > MAX_DATA_SIZE)
        return SPI_ERR_PAYLOAD_LONG;

    // 检查完整帧长度
    if (len < FRAME_OVERHEAD + payload_len)
        return SPI_ERR_INCOMPLETE;

    // 检查帧尾
    if (rx_buf[5 + payload_len + 2] != FRAME_TAIL)
        return SPI_ERR_INVALID_TAIL;

    // 校验CRC
    crc_calc = calc_crc16(&rx_buf[2], 3 + payload_len);
    crc_recv = rx_buf[5 + payload_len] | ((uint16)rx_buf[6 + payload_len] << 8);
    if (crc_calc != crc_recv)
        return SPI_ERR_CRC_MISMATCH;

    // 复制数据
    *data_len = (uint8)payload_len;
    for (uint8 i = 0; i < payload_len; i++)
    {
        data_out[i] = rx_buf[5 + i];
    }

    return SPI_ERR_OK;
}

//==================================================== 接口实现 ====================================================

void spi_comm_init(void)
{
    // 初始化SPI主机 (CS使用GPIO软件控制)
    spi_init(SPI_MASTER_CH, SPI_MODE0, SPI_MASTER_BAUD,
             SPI_MASTER_CLK, SPI_MASTER_MOSI, SPI_MASTER_MISO, SPI_CS_NULL);

    // 初始化CS引脚 (输出高电平，不选中)
    gpio_init(SPI_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // 初始化INT检测引脚 (输入，检测从机拉高表示数据就绪)
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
    uint8 ret;

    // 空指针检查
    if (result == NULL)
        return SPI_ERR_NULL_PTR;

    // 构建请求帧
    tx_len = build_request_frame(CMD_GET_BEACON, tx_buf);

    // 响应帧长度
    uint8 rx_len = BEACON_RESPONSE_LEN;
    uint8 transfer_len = (tx_len > rx_len) ? tx_len : rx_len;

    // 填充发送缓冲区
    for (uint8 i = tx_len; i < transfer_len; i++)
    {
        tx_buf[i] = 0xFF;
    }

    // 拉低CS，选中从机
    gpio_low(SPI_CS_PIN);

    // CS建立时间延时，确保从机有时间准备TX FIFO
    system_delay_us(SPI_CS_SETUP_DELAY_US);

    // 全双工传输
    spi_transfer_8bit(SPI_MASTER_CH, tx_buf, rx_buf, transfer_len);

    // 拉高CS，释放从机
    gpio_high(SPI_CS_PIN);

    // 解析响应帧
    ret = parse_response_frame(rx_buf, transfer_len, data_buf, &data_len);
    if (ret != SPI_ERR_OK)
        return ret;

    // 检查数据长度 (使用显式常量，不依赖编译器对齐)
    if (data_len != BEACON_DATA_SIZE)
        return SPI_ERR_DATA_SIZE;

    // 复制灯塔数据
    result->center_x   = (int16)(data_buf[0] | (data_buf[1] << 8));
    result->center_y   = (int16)(data_buf[2] | (data_buf[3] << 8));
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
