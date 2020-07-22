/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8H8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
#include "headfile.h"

//UART1中断
void UART1_Isr() interrupt 4
{
    uint8 res;
    static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //程序自动下载
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2中断
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
    {
        UART2_CLEAR_TX_FLAG;
        busy[2] = 0;
    }
    if(UART2_GET_RX_FLAG)
    {
        UART2_CLEAR_RX_FLAG;
        //接收数据寄存器为：S2BUF

    }
}


//UART3中断
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
    {
        UART3_CLEAR_TX_FLAG;
        busy[3] = 0;
    }
    if(UART3_GET_RX_FLAG)
    {
        UART3_CLEAR_RX_FLAG;
        //接收数据寄存器为：S3BUF

    }
}


//UART4中断
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;
		//S4BUF;

	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//点亮LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //清除中断标志
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //清除中断标志
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //清除中断标志
}

void TM0_Isr() interrupt 1
{
    
}
void TM1_Isr() interrupt 3
{
    static float angle;
    static Omega omega;
    static float stand_duty;  //控直立的占空比
    static int16 speed_set = SPEED_STRAIGHT;  // 给定速度1000mm/s
    static float angle_set = 22;  // 给定角度,车辆平衡角为23.87，要前进可以多给一些 23
    static float angle_bias = 0;  // 用于控直立的偏移角
    static int16 turn_duty = 0; //控转向的占空比    
    //--------------下面存一些定时间隔---------------//
    static uint16 encoder_read_cnt = 0;  // 编码器读取间隔
    static uint16 turn_control_cnt = 0;  // 转向控制间隔
    static uint16 ring_out_cnt = 0;  // 出环屏蔽时间
    static uint16 ramp_trans_cnt = 0;  // 用于上坡后状态转移延时
    static uint16 ccd_collect_cnt = 0;  // ccd采集间隔

    static uint8 proceed_dir = 0;   //用于指示方向
    static uint8 start_distance_flag = 0;
    // 读取角度和角速度并卡尔曼滤波
    angle = get_angle_from_icm();
    omega = get_omega_from_icm();
    if (angle - car_info.angle > 13)
    {
        angle = car_info.angle + 13;
    }
    else if (angle - car_info.angle < -13)
    {
        angle = car_info.angle - 13;
    }
    test[2] = (int16)(100 * angle); 
    kalman(angle, omega.y);
    // 测试 angle_test += omega.y;
    // 控直立
    stand_duty = angle_control(car_info.angle, car_info.omega.y, angle_set + angle_bias);
    if (++turn_control_cnt == 2)
    {
        turn_control_cnt = 0;
        turn_duty = direction_control();  // 控转向
    }
    motor_output(stand_duty, turn_duty);
    if (++encoder_read_cnt == 4)
    {
        encoder_read_cnt = 0;
        // 读速度, 6ms一次
        car_info.speed = get_speed(6);
        angle_bias = speed_control(car_info.speed.average, speed_set);
    }

    if (++ccd_collect_cnt == 20)  // 30ms采集一次ccd
    {
        ccd_collect();
    }

    // 本质就是一个状态机，根据车辆当前判到的状态进行不同的控制
    switch(car_info.state)
    {
        // 起步
        case TAKE_OFF:
            if(car_info.angle > 20)
            {
                car_info.state = STRAIGHT_AHEAD;
            }
            break;
            switch(take_off_state)
            {
                case STAND_UP:
                    if(start_distance_flag == 0)
                    {
                        start_distance_calc();
                        start_distance_flag = 1;
                    }
                    if(car_info.angle > 20)
                    {
                        take_off_state = GO_STRAIGHT;
                        speed_set = 300;
                    }
                    break;
                case GO_STRAIGHT:
                    if(car_info.distance > 50)
                    {
                        stop_distance_calc();
                        if(proceed_dir == 0)
                        {
                            take_off_state = TURN_LEFT;
                            start_turn_angle_calc();
                        }
                        else
                        {
                            take_off_state = TURN_RIGHT;
                            start_turn_angle_calc();
                        }        
                    }
                    break;
                case TURN_LEFT:
                    if(car_info.turn_angle < -70)
                    {
                        car_info.state = STRAIGHT_AHEAD;
                        stop_turn_angle_calc();
                    }
                    break;
                case TURN_RIGHT:
                    if(car_info.turn_angle > 70)
                    {
                        car_info.state = STRAIGHT_AHEAD;
                        stop_turn_angle_calc();
                    }
                    break;
                default: break;             
            }
            break;
        // 直道
        case STRAIGHT_AHEAD:
            // 状态转移条件:
            // 轮胎差速不是非常大，入弯
            if (car_info.speed.left_right_diff >= 300 && car_info.speed.left_right_diff <= 600)
                car_info.state = INTO_TURN;
            // 轮胎差速很大，弯中
            if (car_info.speed.left_right_diff > 600)
                car_info.state = IN_TURN;
            speed_set = SPEED_STRAIGHT;
            // 判圆环
            if(is_ring())
            {
                LED = 0;
                car_info.state = RING;
                ring_state = RING_TRUE;
                start_distance_calc();
                // motor_stop();
                // if(ring_dir == RIGHT)
                // {
                //     while(1);
                // }
                // else if(ring_dir == LEFT)
                // {
                //     LED = 0;
                //     while(1); 
                // }
            }
            if(is_ramp())
            {
                LED = 0;
                motor_stop();
                while(1);
                car_info.state = RAMP_UP; 
            }
            break;
        case INTO_TURN:
            if(is_ring())
            {
                LED = 0;
                car_info.state = RING;
                ring_state = RING_TRUE;
                start_distance_calc();
            }
            // 轮胎差速很大，弯中
            if (car_info.speed.left_right_diff > 600)
                car_info.state = IN_TURN;
            // 轮胎差速小，直道
            if (car_info.speed.left_right_diff < 300)
                car_info.state = STRAIGHT_AHEAD;
            speed_set = SPEED_CURL;
            if(is_ramp())
            {
                LED = 0;
                motor_stop();
                while(1);
                car_info.state = RAMP_UP; 
            }
            break;
        case IN_TURN:
            if(is_ring())
            {
                LED = 0;
                car_info.state = RING;
                ring_state = RING_TRUE;
                start_distance_calc();
            }
            // 轮胎差速小，直道
            if (car_info.speed.left_right_diff < 300)
            {
                car_info.state = STRAIGHT_AHEAD;
            }
            // 轮胎差速不是非常大，入弯
            if (car_info.speed.left_right_diff >= 300 && car_info.speed.left_right_diff <= 600)
                car_info.state = INTO_TURN;
            speed_set = SPEED_CURL;
            if(is_ramp())
            {
                LED = 0;
                motor_stop();
                while(1);
                car_info.state = RAMP_UP; 
            }
            break;
        case RAMP_UP:
            if(++ramp_trans_cnt >= 300)
            {
                car_info.state = RAMP_DOWN;
                ramp_trans_cnt = 0;
            }
            break;
        case RAMP_DOWN:
            if(++ramp_trans_cnt >= 1800)
            {
                ramp_trans_cnt = 0;
                car_info.state = STRAIGHT_AHEAD;
            }
            break;
        case RING:
            //LED = 0;
            switch(ring_state)
            {
                case RING_TRUE:
                // 判断出前瞻切点，通过距离来判断轮子经过切点
                    if(is_motor_tangent())
                    {
                        LED = 1;
                        ring_state = RING_INTO;
                        stop_distance_calc();
                        start_turn_angle_calc();
                    }
                    break;
                case RING_INTO:
                // 用竖电感进环并在转过50度时移交控制权给横电感
                    if(car_info.turn_angle > 50 || car_info.turn_angle < -50)
                    {
                        LED = 0;
                        ring_state = RING_IN;
                    }
                    break;
                case RING_IN:
                // 用横电感过环，等到270度削弱deviation至30%
                    if(car_info.turn_angle > 270 || car_info.turn_angle < -270)
                    {
                        LED = 1;
                        ring_state = RING_OUT;
                        car_info.turn_angle = 0;
                    }
                    break;
                case RING_OUT:
                    if(++ring_out_cnt > 800)
                    {
                        ring_out_cnt = 0;
                        ring_dir = NOT_A_RING;
                        ring_state = NOT_A_RING;
                        car_info.state = STRAIGHT_AHEAD;
                        // motor_stop();
                        // while(1)
                        // {
                        //     P52 = !P52;
                        //     delay_ms(100);
                        // }
                    }
                    break;
                default:
                    break;
            }
            speed_set = SPEED_CURL;
            break;

        case STOP:
            break;
        default:
            break;
    }
    test[1] = speed_set;
}
void TM2_Isr() interrupt 12
{
    TIM2_CLEAR_FLAG;  //清除中断标志
    
}
void TM3_Isr() interrupt 19
{
    TIM3_CLEAR_FLAG; //清除中断标志

}

void TM4_Isr() interrupt 20
{
    // ccd_collect();
    TIM4_CLEAR_FLAG; //清除中断标志
}

//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;