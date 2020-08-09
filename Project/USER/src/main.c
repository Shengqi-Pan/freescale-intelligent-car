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
void main()
{
    DisableGlobalIRQ();  //  关闭总中断
    board_init();  // 初始化寄存器
    pit_timer_us(TIM_1, 1500);  // 使用TIMER作为周期中断，时间1.5ms一次
    icm20602_init_simspi();  // icm20602初始化, 引脚查看宏定义
    seekfree_wireless_init();  // 无线串口初始化
    motor_init();  // 电机初始化
    l_init();  // ad初始化
    // adc_init(ADC_P15, ADC_SYSclk_DIV_2);
    encoder_init();  // 编码器初始化
    ccd_init();  // 线性ccd初始化
    delay_ms(10);
    EnableGlobalIRQ(); //  开启总中断
    while(1)
    {
        // 上位机示波器查看
        // data_conversion(ad[0], car_info.speed.left, car_info.speed.right, car_info.speed.average, virtual_scope_data);
        // uart_putbuff(WIRELESS_UART, virtual_scope_data, sizeof(virtual_scope_data));
        // seekfree_wireless_send_buff(virtual_scope_data, sizeof(virtual_scope_data));
        // ccd_send_data(WIRELESS_UART, ccd_data);
    }
}