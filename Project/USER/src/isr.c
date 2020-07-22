/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8H8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
#include "headfile.h"

//UART1�ж�
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
        //�����Զ�����
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

//UART2�ж�
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
        //�������ݼĴ���Ϊ��S2BUF

    }
}


//UART3�ж�
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
        //�������ݼĴ���Ϊ��S3BUF

    }
}


//UART4�ж�
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
	LED = 0;	//����LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //����жϱ�־
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //����жϱ�־
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //����жϱ�־
}

void TM0_Isr() interrupt 1
{
    
}
void TM1_Isr() interrupt 3
{
    static float angle;
    static Omega omega;
    static float stand_duty;  //��ֱ����ռ�ձ�
    static int16 speed_set = SPEED_STRAIGHT;  // �����ٶ�1000mm/s
    static float angle_set = 22;  // �����Ƕ�,����ƽ���Ϊ23.87��Ҫǰ�����Զ��һЩ 23
    static float angle_bias = 0;  // ���ڿ�ֱ����ƫ�ƽ�
    static int16 turn_duty = 0; //��ת���ռ�ձ�    
    //--------------�����һЩ��ʱ���---------------//
    static uint16 encoder_read_cnt = 0;  // ��������ȡ���
    static uint16 turn_control_cnt = 0;  // ת����Ƽ��
    static uint16 ring_out_cnt = 0;  // ��������ʱ��
    static uint16 ramp_trans_cnt = 0;  // �������º�״̬ת����ʱ
    static uint16 ccd_collect_cnt = 0;  // ccd�ɼ����

    static uint8 proceed_dir = 0;   //����ָʾ����
    static uint8 start_distance_flag = 0;
    // ��ȡ�ǶȺͽ��ٶȲ��������˲�
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
    // ���� angle_test += omega.y;
    // ��ֱ��
    stand_duty = angle_control(car_info.angle, car_info.omega.y, angle_set + angle_bias);
    if (++turn_control_cnt == 2)
    {
        turn_control_cnt = 0;
        turn_duty = direction_control();  // ��ת��
    }
    motor_output(stand_duty, turn_duty);
    if (++encoder_read_cnt == 4)
    {
        encoder_read_cnt = 0;
        // ���ٶ�, 6msһ��
        car_info.speed = get_speed(6);
        angle_bias = speed_control(car_info.speed.average, speed_set);
    }

    if (++ccd_collect_cnt == 20)  // 30ms�ɼ�һ��ccd
    {
        ccd_collect();
    }

    // ���ʾ���һ��״̬�������ݳ�����ǰ�е���״̬���в�ͬ�Ŀ���
    switch(car_info.state)
    {
        // ��
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
        // ֱ��
        case STRAIGHT_AHEAD:
            // ״̬ת������:
            // ��̥���ٲ��Ƿǳ�������
            if (car_info.speed.left_right_diff >= 300 && car_info.speed.left_right_diff <= 600)
                car_info.state = INTO_TURN;
            // ��̥���ٺܴ�����
            if (car_info.speed.left_right_diff > 600)
                car_info.state = IN_TURN;
            speed_set = SPEED_STRAIGHT;
            // ��Բ��
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
            // ��̥���ٺܴ�����
            if (car_info.speed.left_right_diff > 600)
                car_info.state = IN_TURN;
            // ��̥����С��ֱ��
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
            // ��̥����С��ֱ��
            if (car_info.speed.left_right_diff < 300)
            {
                car_info.state = STRAIGHT_AHEAD;
            }
            // ��̥���ٲ��Ƿǳ�������
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
                // �жϳ�ǰհ�е㣬ͨ���������ж����Ӿ����е�
                    if(is_motor_tangent())
                    {
                        LED = 1;
                        ring_state = RING_INTO;
                        stop_distance_calc();
                        start_turn_angle_calc();
                    }
                    break;
                case RING_INTO:
                // ������н�������ת��50��ʱ�ƽ�����Ȩ������
                    if(car_info.turn_angle > 50 || car_info.turn_angle < -50)
                    {
                        LED = 0;
                        ring_state = RING_IN;
                    }
                    break;
                case RING_IN:
                // �ú��й������ȵ�270������deviation��30%
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
    TIM2_CLEAR_FLAG;  //����жϱ�־
    
}
void TM3_Isr() interrupt 19
{
    TIM3_CLEAR_FLAG; //����жϱ�־

}

void TM4_Isr() interrupt 20
{
    // ccd_collect();
    TIM4_CLEAR_FLAG; //����жϱ�־
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