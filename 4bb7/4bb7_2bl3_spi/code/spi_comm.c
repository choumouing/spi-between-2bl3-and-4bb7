/*********************************************************************************************************************
 * @file    spi_comm.c
 * @brief   SPI通信模块 - 主机端(4BB7)
 * @details 第二阶段：主机DMA优化
 *          - TX传输使用PDMA进行批量写入TX FIFO
 *          - RX接收使用轮询读取（数据量小，效率更高）
 *          - 添加超时保护机制
 ********************************************************************************************************************/

#include "spi_comm.h"
#include "zf_common_headfile.h"
#include "dma/cy_pdma.h"
#include "scb/cy_scb_spi.h"
#include <string.h>

//==================================================== DMA配置 ====================================================
// DMA通道分配 (使用DW0通道0用于TX)
#define DMA_TX_CH           0

// SCB7寄存器地址 (SPI_0对应SCB7)
#define SCB7_TX_FIFO_WR     ((volatile uint32_t*)&SCB7->unTX_FIFO_WR.u32Register)

// 超时计数 (约10ms @ 250MHz，循环开销约10-20周期)
#define DMA_TIMEOUT_COUNT   100000

//==================================================== DMA变量 ====================================================
// DMA描述符 (必须4字节对齐)
CY_ALIGN(4) static cy_stc_pdma_descr_t g_tx_descr;

// DMA缓冲区 (使用volatile防止编译器优化)
static volatile uint8 g_tx_buf[MAX_FRAME_SIZE];

//==================================================== 内部函数 ====================================================

/**
 * @brief  计算CRC16校验值 (Modbus CRC16)
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
 * @brief  初始化TX DMA描述符
 * @note   配置为1D传输，软件触发后一次性将数据写入TX FIFO
 */
static void init_dma_descriptor(void)
{
    cy_stc_pdma_descr_config_t tx_cfg = {
        .deact          = CY_PDMA_RETDIG_IM,            // 立即重触发模式
        .intrType       = CY_PDMA_INTR_DESCR_CMPLT,     // 描述符完成时产生中断
        .trigoutType    = CY_PDMA_TRIGOUT_DESCR_CMPLT,  // 描述符完成时触发输出
        .chStateAtCmplt = CY_PDMA_CH_DISABLED,          // 传输完成后禁用通道
        .triginType     = CY_PDMA_TRIGIN_DESCR,         // 触发整个描述符传输
        .dataSize       = CY_PDMA_BYTE,                 // 字节传输
        .srcTxfrSize    = CY_PDMA_TXFR_SIZE_DATA_SIZE,  // 源传输大小=数据大小
        .destTxfrSize   = CY_PDMA_TXFR_SIZE_DATA_SIZE,  // 目标传输大小=数据大小
        .descrType      = CY_PDMA_1D_TRANSFER,          // 1D传输模式
        .srcAddr        = (void*)g_tx_buf,              // 源地址：RAM缓冲区
        .destAddr       = (void*)SCB7_TX_FIFO_WR,       // 目标地址：TX FIFO
        .srcXincr       = 1,                            // 源地址每次递增1
        .destXincr      = 0,                            // 目标地址固定（FIFO寄存器）
        .xCount         = BEACON_RESPONSE_LEN,          // 初始传输字节数
        .srcYincr       = 0,
        .destYincr      = 0,
        .yCount         = 0,
        .descrNext      = NULL                          // 无链接描述符
    };
    Cy_PDMA_Descr_Init(&g_tx_descr, &tx_cfg);
}

/**
 * @brief  初始化DMA通道
 */
static void init_dma_channel(void)
{
    cy_stc_pdma_chnl_config_t tx_chnl_cfg = {
        .PDMA_Descriptor = &g_tx_descr,
        .preemptable     = 0,                           // 不可抢占
        .priority        = 0,                           // 最高优先级
        .enable          = 0,                           // 初始禁用，传输时启用
    };
    Cy_PDMA_Chnl_Init(DW0, DMA_TX_CH, &tx_chnl_cfg);

    // 使能DW0模块
    Cy_PDMA_Enable(DW0);
}

/**
 * @brief  使用DMA执行SPI全双工传输
 * @param  tx_data  发送数据指针
 * @param  rx_data  接收数据指针
 * @param  len      传输长度
 * @return 0-成功, 非0-超时
 *
 * @note   传输策略:
 *         - TX: 使用DMA将数据批量写入TX FIFO
 *         - RX: SPI全双工特性，传输完成后从RX FIFO读取
 *         - 对于14字节的小数据量，此方案简单高效
 */
static uint8 spi_transfer_dma(const uint8 *tx_data, uint8 *rx_data, uint8 len)
{
    uint32 timeout;

    // 参数校验
    if (len == 0 || len > MAX_FRAME_SIZE)
        return SPI_ERR_DATA_SIZE;

    // 复制发送数据到DMA缓冲区
    memcpy((void*)g_tx_buf, tx_data, len);

    // 更新描述符传输长度 (X_COUNT = N-1)
    g_tx_descr.unPDMA_DESCR_X_CTL.stcField.u8X_COUNT = (uint8)(len - 1);

    // 清空SPI FIFO
    Cy_SCB_SPI_ClearRxFifo(SCB7);
    Cy_SCB_SPI_ClearTxFifo(SCB7);

    // 重新设置描述符指针
    Cy_PDMA_Chnl_SetDescr(DW0, DMA_TX_CH, &g_tx_descr);

    // 清除DMA中断状态
    Cy_PDMA_Chnl_ClearInterrupt(DW0, DMA_TX_CH);

    // 启用DMA通道
    Cy_PDMA_Chnl_Enable(DW0, DMA_TX_CH);

    // 触发TX DMA传输
#if defined(CPUSS_SW_TR_PRESENT) && (CPUSS_SW_TR_PRESENT == 1)
    // 使用软件触发启动DMA
    Cy_PDMA_Chnl_SetSwTrigger(DW0, DMA_TX_CH);
#else
    // 备用方案：直接写入TX FIFO (无DMA硬件触发支持时)
    for (uint8 i = 0; i < len; i++)
    {
        Cy_SCB_WriteTxFifo(SCB7, g_tx_buf[i]);
    }
#endif

    // 等待TX DMA完成 (带超时保护)
    timeout = DMA_TIMEOUT_COUNT;
    while (!Cy_PDMA_Chnl_GetInterruptStatus(DW0, DMA_TX_CH))
    {
        if (--timeout == 0)
        {
            Cy_PDMA_Chnl_Disable(DW0, DMA_TX_CH);
            return SPI_ERR_TIMEOUT;
        }
    }

    // 等待SPI传输完成 (TX FIFO清空且移位寄存器空闲)
    timeout = DMA_TIMEOUT_COUNT;
    while (!Cy_SCB_IsTxComplete(SCB7))
    {
        if (--timeout == 0)
        {
            Cy_PDMA_Chnl_Disable(DW0, DMA_TX_CH);
            return SPI_ERR_TIMEOUT;
        }
    }

    // 等待所有RX数据接收完毕
    timeout = DMA_TIMEOUT_COUNT;
    while (Cy_SCB_SPI_GetNumInRxFifo(SCB7) < len)
    {
        if (--timeout == 0)
        {
            Cy_PDMA_Chnl_Disable(DW0, DMA_TX_CH);
            return SPI_ERR_TIMEOUT;
        }
    }

    // 从RX FIFO读取接收数据
    for (uint8 i = 0; i < len; i++)
    {
        rx_data[i] = (uint8)Cy_SCB_SPI_Read(SCB7);
    }

    // 禁用DMA通道
    Cy_PDMA_Chnl_Disable(DW0, DMA_TX_CH);

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

    // 初始化DMA
    init_dma_descriptor();
    init_dma_channel();
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

    // 填充发送缓冲区 (补齐到传输长度)
    memset(&tx_buf[tx_len], 0xFF, transfer_len - tx_len);

    // 拉低CS，选中从机
    gpio_low(SPI_CS_PIN);

    // CS建立时间延时 (确保从机准备好)
    system_delay_us(SPI_CS_SETUP_DELAY_US);

    // 执行SPI全双工传输
    ret = spi_transfer_dma(tx_buf, rx_buf, transfer_len);

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
