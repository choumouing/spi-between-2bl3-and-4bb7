/*********************************************************************************************************************
 * @file    spi_slave.c
 * @brief   SPI从机通信模块 - 从机端(2BL3)
 * @details 第一阶段：基础通信验证（中断模式）
 ********************************************************************************************************************/

#include "spi_slave.h"
#include "zf_common_headfile.h"
#include "scb/cy_scb_spi.h"
#include "sysclk/cy_sysclk.h"
#include "gpio/cy_gpio.h"

//==================================================== 内部变量 ====================================================
static beacon_result_t g_beacon_data;                   // 灯塔数据缓存
static uint8 g_tx_buffer[MAX_FRAME_SIZE];               // 发送缓冲区
static volatile uint8 g_data_pending = 0;               // 数据待发送标志 (1=有新数据待发送)

//==================================================== 内部函数 ====================================================

/**
 * @brief  计算CRC16校验值 (Modbus CRC16)
 * @note   此函数需与主机端(spi_comm.c)保持一致
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
 * @brief  构建响应帧 (GET_BEACON命令)
 * @param  buffer 输出缓冲区
 * @return 帧长度
 */
static uint8 build_beacon_response(uint8 *buffer)
{
    uint16 crc;
    uint8 *p = &buffer[5];  // 数据区起始

    buffer[0] = FRAME_HEAD_1;
    buffer[1] = FRAME_HEAD_2;
    buffer[2] = CMD_GET_BEACON;
    buffer[3] = 0;                  // LEN高字节
    buffer[4] = BEACON_DATA_SIZE;   // LEN低字节 = 6

    // 填充数据 (小端序)
    *p++ = (uint8)(g_beacon_data.center_x & 0xFF);
    *p++ = (uint8)(g_beacon_data.center_x >> 8);
    *p++ = (uint8)(g_beacon_data.center_y & 0xFF);
    *p++ = (uint8)(g_beacon_data.center_y >> 8);
    *p++ = g_beacon_data.found;
    *p++ = g_beacon_data.confidence;

    // CRC计算范围: CMD + LEN(2) + DATA(6) = 9字节
    crc = calc_crc16(&buffer[2], 3 + BEACON_DATA_SIZE);
    *p++ = (uint8)(crc & 0xFF);
    *p++ = (uint8)(crc >> 8);
    *p++ = FRAME_TAIL;

    return BEACON_RESPONSE_LEN;
}

/**
 * @brief  初始化SPI从机引脚
 * @note   使用SCB0 (P0端口), HSIOM=30 (Deep Sleep模式)
 *         SCB0是CYT2BL3上唯一具备完整SPI从机功能(含硬件SS)的SCB
 */
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

    // SS (P0_3) - 硬件片选输入 (高阻, 使用SCB硬件功能)
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P0_3_SCB0_SPI_SELECT0;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 3, &pin_cfg);

    // INT (P18_6) - GPIO输出 (使用逐飞库, 初始低电平)
    gpio_init(SPI_SLAVE_INT, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/**
 * @brief  初始化SPI从机模块 (SCB0)
 */
static void spi_slave_init_scb(void)
{
    cy_stc_scb_spi_config_t spi_cfg = {0};

    // 配置时钟分频器 (从机模式下分频器不影响速率，但需要使能)
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB0_CLOCK, CY_SYSCLK_DIV_8_BIT, 0);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 0, 0);  // 不分频
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 0);

    // 配置SPI从机模式
    spi_cfg.spiMode = CY_SCB_SPI_SLAVE;
    spi_cfg.subMode = CY_SCB_SPI_MOTOROLA;
    spi_cfg.sclkMode = CY_SCB_SPI_CPHA0_CPOL0;  // MODE0
    spi_cfg.oversample = 0;                      // 从机模式忽略此参数
    spi_cfg.rxDataWidth = 8;
    spi_cfg.txDataWidth = 8;
    spi_cfg.enableMsbFirst = true;
    spi_cfg.enableInputFilter = false;
    spi_cfg.enableFreeRunSclk = false;
    spi_cfg.enableMisoLateSample = false;
    spi_cfg.enableTransferSeperation = false;
    spi_cfg.ssPolarity0 = false;                 // SS低电平有效
    spi_cfg.enableWakeFromSleep = false;
    spi_cfg.rxFifoTriggerLevel = 0;
    spi_cfg.rxFifoIntEnableMask = 0;
    spi_cfg.txFifoTriggerLevel = 0;
    spi_cfg.txFifoIntEnableMask = 0;
    spi_cfg.masterSlaveIntEnableMask = 0;

    Cy_SCB_SPI_Init(SPI_SLAVE_SCB, &spi_cfg, NULL);
    Cy_SCB_SPI_SetActiveSlaveSelect(SPI_SLAVE_SCB, CY_SCB_SPI_SLAVE_SELECT0);
    Cy_SCB_SPI_Enable(SPI_SLAVE_SCB);
}

/**
 * @brief  预加载TX FIFO数据
 * @note   在数据更新后立即调用，确保主机随时可以读取
 */
static void preload_tx_fifo(void)
{
    // 清空TX FIFO后重新加载
    Cy_SCB_SPI_ClearTxFifo(SPI_SLAVE_SCB);
    for (uint8 i = 0; i < BEACON_RESPONSE_LEN; i++)
    {
        Cy_SCB_WriteTxFifo(SPI_SLAVE_SCB, g_tx_buffer[i]);
    }
}

//==================================================== 接口实现 ====================================================

void spi_slave_init(void)
{
    // 初始化引脚
    spi_slave_init_pins();

    // 初始化SPI从机模块
    spi_slave_init_scb();

    // 初始化默认灯塔数据
    g_beacon_data.center_x = 0;
    g_beacon_data.center_y = 0;
    g_beacon_data.found = 0;
    g_beacon_data.confidence = 0;

    // 预先构建响应帧
    build_beacon_response(g_tx_buffer);

    // 预加载初始数据到TX FIFO，确保主机随时可以读取
    preload_tx_fifo();

    // 初始状态：无待发送数据
    g_data_pending = 0;
}

void spi_slave_set_int(uint8 level)
{
    if (level)
        gpio_high(SPI_SLAVE_INT);
    else
        gpio_low(SPI_SLAVE_INT);
}

/**
 * @brief  更新灯塔数据 (供上层调用)
 * @param  result 灯塔数据指针
 * @note   采用预加载策略：数据更新后立即加载到TX FIFO，避免SS下降沿时的竞态条件。
 *         调用约束：不可在中断中调用，应在主循环中顺序调用。
 */
void spi_slave_update_beacon(const beacon_result_t *result)
{
    // 空指针检查
    if (result == NULL)
        return;

    // 更新数据
    g_beacon_data = *result;

    // 重新构建响应帧
    build_beacon_response(g_tx_buffer);

    // 预加载数据到TX FIFO (关键：先准备数据，再通知主机)
    preload_tx_fifo();

    // 设置数据待发送标志
    g_data_pending = 1;

    // 拉高INT通知主机
    spi_slave_set_int(1);
}

void spi_slave_task(void)
{
    static uint8 ss_last = 1;
    // 读取硬件SS引脚状态 (P0_3)
    uint8 ss_now = Cy_GPIO_Read(GPIO_PRT0, 3);

    // 检测SS下降沿 (主机开始通信)
    if (ss_last == 1 && ss_now == 0)
    {
        // SS被拉低，主机开始读取
        // 清空RX FIFO准备接收主机请求
        Cy_SCB_SPI_ClearRxFifo(SPI_SLAVE_SCB);

        // 注意：TX FIFO数据已在update_beacon()中预加载，无需再次加载
        // 这种预加载策略避免了SS下降沿时的竞态条件
    }

    // 检测SS上升沿 (传输完成)
    if (ss_last == 0 && ss_now == 1)
    {
        // 传输完成
        if (g_data_pending)
        {
            // 数据已被读取，清除待发送标志
            g_data_pending = 0;

            // 拉低INT信号
            spi_slave_set_int(0);

            // 传输完成后重新预加载TX FIFO，为下次传输做准备
            preload_tx_fifo();
        }
    }

    ss_last = ss_now;
}
