/*********************************************************************************************************************
 * @file    spi_comm.c
 * @brief   SPI通信模块 - 主机端(4BB7) - DMA优化版
 * @details 第二阶段：使用DW1 PDMA实现SPI主机端传输
 *
 * DMA配置说明（基于cyt4bb_config.h官方定义）：
 *   - SCB7的TX/RX触发连接到DW1（非DW0）
 *   - TX: TRIG_OUT_1TO1_2_SCB_TX_TO_PDMA17 → DW1通道30
 *   - RX: TRIG_OUT_1TO1_2_SCB_RX_TO_PDMA17 → DW1通道31
 *   - 使用1-to-1触发器，无需复杂的TrigMux路由
 ********************************************************************************************************************/

#include "spi_comm.h"
#include "zf_common_headfile.h"
#include "scb/cy_scb_spi.h"
#include "dma/cy_pdma.h"
#include "trigmux/cy_trigmux.h"

//==================================================== DMA配置 ====================================================
// DMA硬件资源定义（基于cyt4bb_config.h官方定义）
#define DMA_HW                  DW1             // SCB7使用DW1（非DW0）
#define DMA_TX_CH               30              // dw1_tr_in[30] 对应SCB7 TX
#define DMA_RX_CH               31              // dw1_tr_in[31] 对应SCB7 RX

// SCB7 FIFO寄存器地址（基于cyreg_scb.h）
#define SCB7_TX_FIFO_WR_ADDR    0x40670240UL
#define SCB7_RX_FIFO_RD_ADDR    0x40670340UL

// DMA描述符（必须4字节对齐）
CY_ALIGN(4) static cy_stc_pdma_descr_t g_dma_tx_descr;
CY_ALIGN(4) static cy_stc_pdma_descr_t g_dma_rx_descr;

// DMA初始化标志
static uint8 g_dma_initialized = 0;

//==================================================== DMA内部函数 ====================================================

/**
 * @brief  初始化SPI DMA模块
 * @note   配置DW1的TX/RX通道，连接1-to-1触发器
 */
static void spi_dma_init(void)
{
    if (g_dma_initialized)
        return;

    // 1. 使能DW1控制器
    Cy_PDMA_Enable(DMA_HW);

    // 2. 配置1-to-1触发器（基于cyt4bb_config.h定义）
    // SCB7 TX请求 → DW1通道30
    Cy_TrigMux_Connect1To1(TRIG_OUT_1TO1_2_SCB_TX_TO_PDMA17,
                           CY_TR_MUX_TR_INV_DISABLE,
                           TRIGGER_TYPE_LEVEL,
                           0);  // 不在调试模式下冻结

    // SCB7 RX请求 → DW1通道31
    Cy_TrigMux_Connect1To1(TRIG_OUT_1TO1_2_SCB_RX_TO_PDMA17,
                           CY_TR_MUX_TR_INV_DISABLE,
                           TRIGGER_TYPE_LEVEL,
                           0);

    g_dma_initialized = 1;
}

/**
 * @brief  配置并启动DMA传输
 * @param  tx_data  发送数据缓冲区
 * @param  rx_data  接收数据缓冲区
 * @param  len      传输长度（字节）
 * @return 0-成功，非0-失败
 */
static uint8 dma_transfer(const uint8 *tx_data, uint8 *rx_data, uint8 len)
{
    cy_stc_pdma_descr_config_t descr_cfg;
    cy_stc_pdma_chnl_config_t  chnl_cfg;
    uint32 timeout;

    // 清空RX FIFO，避免读取到残留数据
    Cy_SCB_SPI_ClearRxFifo(SCB7);

    // ===== 配置TX DMA描述符 =====
    descr_cfg.deact          = CY_PDMA_TRIG_DEACT_NO_WAIT;   // 不等待触发去激活（脉冲触发）
    descr_cfg.intrType       = CY_PDMA_INTR_DESCR_CMPLT;   // 描述符完成时产生中断
    descr_cfg.trigoutType    = CY_PDMA_TRIGOUT_DESCR_CMPLT;// 完成时输出触发
    descr_cfg.triginType     = CY_PDMA_TRIGIN_1ELEMENT;    // 每个触发传输1个元素
    descr_cfg.dataSize       = CY_PDMA_BYTE;               // 8位数据宽度
    descr_cfg.srcTxfrSize    = CY_PDMA_TXFR_SIZE_DATA_SIZE;// 源传输大小=数据大小
    descr_cfg.destTxfrSize   = CY_PDMA_TXFR_SIZE_WORD;     // 目的传输大小=字（FIFO要求）
    descr_cfg.descrType      = CY_PDMA_1D_TRANSFER;        // 1D传输
    descr_cfg.srcAddr        = (void *)tx_data;            // 源：内存缓冲区
    descr_cfg.destAddr       = (void *)SCB7_TX_FIFO_WR_ADDR;// 目的：TX FIFO
    descr_cfg.srcXincr       = 1;                          // 源地址递增
    descr_cfg.destXincr      = 0;                          // 目的地址不变（固定FIFO）
    descr_cfg.xCount         = len;                        // 传输元素数量
    descr_cfg.descrNext      = NULL;                       // 无下一描述符
    descr_cfg.chStateAtCmplt = CY_PDMA_CH_DISABLED;        // 完成后禁用通道

    Cy_PDMA_Descr_Init(&g_dma_tx_descr, &descr_cfg);

    // ===== 配置RX DMA描述符 =====
    descr_cfg.srcTxfrSize    = CY_PDMA_TXFR_SIZE_WORD;     // 源传输大小=字（FIFO）
    descr_cfg.destTxfrSize   = CY_PDMA_TXFR_SIZE_DATA_SIZE;// 目的传输大小=数据大小
    descr_cfg.srcAddr        = (void *)SCB7_RX_FIFO_RD_ADDR;// 源：RX FIFO
    descr_cfg.destAddr       = (void *)rx_data;            // 目的：内存缓冲区
    descr_cfg.srcXincr       = 0;                          // 源地址不变（固定FIFO）
    descr_cfg.destXincr      = 1;                          // 目的地址递增

    Cy_PDMA_Descr_Init(&g_dma_rx_descr, &descr_cfg);

    // ===== 配置并使能DMA通道 =====
    chnl_cfg.PDMA_Descriptor = &g_dma_tx_descr;
    chnl_cfg.preemptable     = 0;
    chnl_cfg.priority        = 0;
    chnl_cfg.enable          = 1;

    // 清除中断状态
    Cy_PDMA_Chnl_ClearInterrupt(DMA_HW, DMA_TX_CH);
    Cy_PDMA_Chnl_ClearInterrupt(DMA_HW, DMA_RX_CH);

    // 初始化TX通道
    Cy_PDMA_Chnl_Init(DMA_HW, DMA_TX_CH, &chnl_cfg);
    Cy_PDMA_Chnl_SetInterruptMask(DMA_HW, DMA_TX_CH);

    // 初始化RX通道
    chnl_cfg.PDMA_Descriptor = &g_dma_rx_descr;
    Cy_PDMA_Chnl_Init(DMA_HW, DMA_RX_CH, &chnl_cfg);
    Cy_PDMA_Chnl_SetInterruptMask(DMA_HW, DMA_RX_CH);

    // ===== 等待传输完成 =====
    // RX DMA完成意味着所有数据都已传输（因为SPI是全双工的）
    timeout = DMA_TIMEOUT_COUNT;
    while (Cy_PDMA_Chnl_GetInterruptStatusMasked(DMA_HW, DMA_RX_CH) == 0)
    {
        if (--timeout == 0)
        {
            // 超时，禁用通道
            Cy_PDMA_Chnl_Disable(DMA_HW, DMA_TX_CH);
            Cy_PDMA_Chnl_Disable(DMA_HW, DMA_RX_CH);
            return SPI_ERR_TIMEOUT;
        }
    }

    // 清除中断状态
    Cy_PDMA_Chnl_ClearInterrupt(DMA_HW, DMA_TX_CH);
    Cy_PDMA_Chnl_ClearInterrupt(DMA_HW, DMA_RX_CH);

    return SPI_ERR_OK;
}

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

    // 初始化DMA (DW1 PDMA + 1-to-1触发器)
    spi_dma_init();
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

    // 全双工DMA传输 (替代原spi_transfer_8bit)
    ret = dma_transfer(tx_buf, rx_buf, transfer_len);
    if (ret != SPI_ERR_OK)
    {
        gpio_high(SPI_CS_PIN);
        return ret;
    }

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
