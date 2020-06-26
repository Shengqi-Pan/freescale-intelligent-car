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
#include "motor.h"
#include "encoder.h"
#include "control.h"

//board.h文件中FOSC的值设置为0,则程序自动识别系统频率

/*board.h文件中FOSC的值设置不为0，则系统频率为FOSC的值，
在使用stc-isp工具下载程序的时候需要将IRC频率设置为FOSC的值*/

/*在board_init中,已经将P54引脚设置为复位，
如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可*/

// 保存车辆当前的信息
float angle = 0;
Omega omega = {0, 0};

void main()
{
    DisableGlobalIRQ(); //  关闭总中断
    board_init(); //  初始化寄存器
    pit_timer_ms(TIM_0, 1); // 使用TIMER作为周期中断，时间1ms一次
    icm20602_init_simspi(); // icm20602初始化, 引脚查看宏定义
    // uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, 115200, TIM_1);  // 串口1初始化，波特率115200，发送引脚TX P31 接收引脚RX P30
    seekfree_wireless_init();  // 无线串口初始化
    motor_init();  // 电机初始化
    l_init();  //ad初始化
    //encoder_init();
    delay_ms(10);

    EnableGlobalIRQ(); //  开启总中断

    while(1)
    {   
        /*用于测试电感检测值*/
        /*static uint16 ref[4] = {0, 0, 0, 0};
        uint8 to_send;
        int i;
        induc_test();
        for(i=0;i<4;i++)
        {
            if(ref[i] < ad_test[i])
            {
                ref[i] = ad_test[i];
                to_send = (uint8)(i & 0xff);
                uart_putchar(WIRELESS_UART, to_send);
                to_send = (uint8)((ref[i] >> 8) & 0xff);
                uart_putchar(WIRELESS_UART, to_send);
                to_send = (uint8)(ref[i] & 0xff);
                uart_putchar(WIRELESS_UART, to_send); 
            }
        }*/
        // 上位机查看角度和角速度等
        // data_conversion((int16)angle, (int16)omega.y,
        //                 (int16)car_info.angle, (int16)car_info.omega.y,
        //                 virtual_scope_data);
        // 上位机查看四个电感的值
        //induc_test();
        /*data_conversion(ad_test[0], ad_test[1],
                        ad_test[2], ad_test[3],
                        virtual_scope_data);*/
        /*data_conversion(car_info.speed.left, car_info.speed.right,
                        0, 0,
                        virtual_scope_data);*/
        //uart_putbuff(WIRELESS_UART, virtual_scope_data, sizeof(virtual_scope_data));
    }
}