/*********************************************************************************************************************
* CYT2BL3 Opensourec Library （本工程基于逐飞科技官方开源库）
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "spi_slave.h"

//==================================================== 临时配置 ====================================================
// SPI功能验证开关
// 原因: Debug串口(UART_0)使用SCB0和P0_0/P0_1，与SPI从机(SCB0, P0_0~P0_3)冲突
// 设为0: 禁用debug串口，SPI从机独占SCB0
// 设为1: 启用debug串口，但SPI从机将无法正常工作
// 待解决方案: 将debug串口迁移到其他SCB/引脚后，可改回1
#define ENABLE_DEBUG_UART   0

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);

#if ENABLE_DEBUG_UART
    debug_init();
#endif

    // SPI从机初始化 (使用SCB0, P0_0~P0_3)
    spi_slave_init();

#if ENABLE_DEBUG_UART
    printf("2BL3 SPI Slave Ready\r\n");
#endif

    // 设置初始测试数据并通知主机
    beacon_result_t test_data = {
        .center_x = 160,
        .center_y = 120,
        .found = 1,
        .confidence = 85
    };
    spi_slave_update_beacon(&test_data);

    uint32 loop_count = 0;

    while(true)
    {
        // SPI从机任务处理
        spi_slave_task();

        // 周期性更新测试数据 (约每1000000次循环更新一次)
        loop_count++;
        if (loop_count >= 1000000)
        {
            loop_count = 0;

            // 模拟数据变化
            test_data.center_x = (test_data.center_x + 10) % 320;
            test_data.center_y = (test_data.center_y + 5) % 240;
            test_data.confidence = 80 + (test_data.center_x % 20);

            spi_slave_update_beacon(&test_data);

#if ENABLE_DEBUG_UART
            printf("Update: x=%d, y=%d\r\n", test_data.center_x, test_data.center_y);
#endif
        }
    }
}
