/*********************************************************************************************************************
* CYT4BB Opensourec Library （本工程基于逐飞科技官方开源库）
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "spi_comm.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化
    debug_init();                       // 调试串口信息初始化

    // SPI通信初始化
    spi_comm_init();
    printf("4BB7 SPI Master Ready\r\n");

    while(true)
    {
        // SPI通信测试
        spi_comm_test();
    }
}
