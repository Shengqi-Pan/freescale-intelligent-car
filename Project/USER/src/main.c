/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MDK FOR C51 V9.60
 * @Target core     STC8H8K64S4
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-06-01
 ********************************************************************************************************************/

#include "headfile.h"
#include "icm20602.h"
#include "car_info.h"
#include "kalman.h"

//board.h文件中FOSC的值设置为0,则程序自动识别系统频率

/*board.h文件中FOSC的值设置不为0，则系统频率为FOSC的值，
在使用stc-isp工具下载程序的时候需要将IRC频率设置为FOSC的值*/

/*在board_init中,已经将P54引脚设置为复位，
如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可*/

// 保存车辆当前的信息
extern CarInfo car_info;

void send_icm(void)
{
    uint8 high = icm_acc_x >> 8;
    uint8 low = (uint8)icm_acc_x;
    uart_putchar(UART_1, high);
    uart_putchar(UART_1, low);
}

void main()
{
    // 保存读取来的信息
    float angle = 0;
    Omega omega = {0, 0};

    DisableGlobalIRQ(); //  关闭总中断
    board_init(); //  初始化寄存器
    EnableGlobalIRQ(); //  开启总中断
    icm20602_init_simspi(); // icm20602初始化, 引脚查看宏定义
    uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, 115200, TIM_1);  // 串口1初始化，波特率115200，发送引脚TX P31 接收引脚RX P30

    while(1)
    {
        // send_icm();                   // 发送六个数据，顺序见子函数
        angle = get_angle_from_icm();
        omega = get_omega_from_icm();
        kalman(angle, omega);
        delay_ms(100);
    }
}