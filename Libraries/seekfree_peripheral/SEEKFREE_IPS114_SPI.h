/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-11-15
 * @note		
					���߶��壺
					------------------------------------ 
					1.14��IPSģ��ܽ�       ��Ƭ���ܽ�
					SCL                 	�鿴SEEKFREE_IPS114_SPI.h�ļ��ڵ�IPS114_SCL		�궨��     Ӳ��SPI���Ų��������л�
					SDA                 	�鿴SEEKFREE_IPS114_SPI.h�ļ��ڵ�IPS114_SDA		�궨��     Ӳ��SPI���Ų��������л�
					RES                 	�鿴SEEKFREE_IPS114_SPI.h�ļ��ڵ�IPS114_REST_PIN�궨��    
					DC                  	�鿴SEEKFREE_IPS114_SPI.h�ļ��ڵ�IPS114_DC_PIN	�궨��  
					CS                  	�鿴SEEKFREE_IPS114_SPI.h�ļ��ڵ�IPS114_CS		�궨��     Ӳ��SPI���Ų��������л�
					BL  					�鿴SEEKFREE_IPS114_SPI.h�ļ��ڵ�IPS114_BL_PIN	�궨��
					
					��Դ����
					VCC 3.3V��Դ
					GND ��Դ��
					���ֱ���135*240
					------------------------------------ 
 ********************************************************************************************************************/
 


#ifndef _SEEKFREE_IPS114_H
#define _SEEKFREE_IPS114_H

#include "common.h"
#include "board.h"

//PERIPHERAL_PIN_SWITCH����Ϊ0��ʹ��48��������������
//PERIPHERAL_PIN_SWITCH����Ϊ1��ʹ��64��������������
//PERIPHERAL_PIN_SWITCH��ֵ������board.h�ļ����޸�
#if (PERIPHERAL_PIN_SWITCH == 0)

	#define IPS114_SPIN_PIN     SPI_2           //����ʹ�õ�SPI��
	#define	IPS114_SCL_PIN 		SPI2_SCLK_P25	//����SPI_SCK����
	#define	IPS114_SDA_PIN		SPI2_MOSI_P23   //����SPI_MOSI����
	#define	IPS114_SDA_IN_PIN	SPI2_MISO_P24   //����SPI_MISO����  IPS��Ļû��MISO���ţ�����������Ȼ��Ҫ���壬��spi�ĳ�ʼ��ʱ��Ҫʹ��
	#define IPS114_REST_PIN  	P32
	#define IPS114_DC_PIN 	 	P33   	//Һ������λ���Ŷ���
	#define IPS114_CS_PIN    	P35   	//����SPI_CS����
	#define IPS114_BL_PIN    	P36     //Һ���������Ŷ���  
	
#elif (PERIPHERAL_PIN_SWITCH == 1)

	#define IPS114_SPIN_PIN     SPI_2           //����ʹ�õ�SPI��
	#define	IPS114_SCL_PIN 		SPI2_SCLK_P25	//����SPI_SCK����
	#define	IPS114_SDA_PIN		SPI2_MOSI_P23   //����SPI_MOSI����
	#define	IPS114_SDA_IN_PIN	SPI2_MISO_P24   //����SPI_MISO����  IPS��Ļû��MISO���ţ�����������Ȼ��Ҫ���壬��spi�ĳ�ʼ��ʱ��Ҫʹ��
	#define IPS114_REST_PIN  	P20
	#define IPS114_DC_PIN 	 	P21   	//Һ������λ���Ŷ���
	#define IPS114_CS_PIN    	P22   	//����SPI_CS����
	#define IPS114_BL_PIN    	P27     //Һ���������Ŷ��� 
	
#endif



#define IPS114_REST(x)      (IPS114_REST_PIN = x)
#define IPS114_DC(x)        (IPS114_DC_PIN = x)
#define IPS114_CS(x)        (IPS114_CS_PIN = x)
#define IPS114_BL(x)        (IPS114_BL_PIN = x)


//-------������ɫ��SEEKFREE_FONT.h�ļ��ж���----------
//#define RED          	    0xF800	//��ɫ
//#define BLUE         	    0x001F  //��ɫ
//#define YELLOW       	    0xFFE0	//��ɫ
//#define GREEN        	    0x07E0	//��ɫ
//#define WHITE        	    0xFFFF	//��ɫ
//#define BLACK        	    0x0000	//��ɫ 
//#define GRAY  			0X8430 	//��ɫ
//#define BROWN 			0XBC40 	//��ɫ
//#define PURPLE    		0XF81F	//��ɫ
//#define PINK    		    0XFE19	//��ɫ


//����д�ֱʵ���ɫ
#define IPS114_PENCOLOR    RED

//���屳����ɫ
#define IPS114_BGCOLOR     WHITE



#define IPS114_W   135
#define IPS114_H   240

//������ʾ����
//0 ����ģʽ
//1 ����ģʽ  ��ת180
//2 ����ģʽ
//3 ����ģʽ  ��ת180
#define IPS114_DISPLAY_DIR 3

#if (0==IPS114_DISPLAY_DIR || 1==IPS114_DISPLAY_DIR)
#define	IPS114_X_MAX	IPS114_W	//Һ��X������
#define IPS114_Y_MAX	IPS114_H    //Һ��Y������
     
#elif (2==IPS114_DISPLAY_DIR || 3==IPS114_DISPLAY_DIR)
#define	IPS114_X_MAX	IPS114_H	//Һ��X������
#define IPS114_Y_MAX	IPS114_W    //Һ��Y������
     
#else
#error "IPS114_DISPLAY_DIR �������"
     
#endif


void ips114_init(void);
void ips114_clear(uint16 color);
void ips114_drawpoint(uint16 x,uint16 y,uint16 color);
void ips114_showchar(uint16 x,uint16 y,const int8 dat);
void ips114_showstr(uint16 x,uint16 y,const int8 dat[]);
void ips114_showint8(uint16 x,uint16 y,int8 dat);
void ips114_showuint8(uint16 x,uint16 y,uint8 dat);
void ips114_showint16(uint16 x,uint16 y,int16 dat);
void ips114_showuint16(uint16 x,uint16 y,uint16 dat);
void ips114_showint32(uint16 x,uint16 y,int32 dat,uint8 num);
void ips114_showfloat(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum);



#endif