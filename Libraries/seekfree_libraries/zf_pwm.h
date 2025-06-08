/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		pwm
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/

#ifndef __ZF_PWM_H
#define __ZF_PWM_H
#include "common.h"


#define PWM_DUTY_MAX 10000



typedef enum
{
	//PWMA��PWMB�����鲻ͬ��PWM
	
	//������PWMAͨ����
	//ͬһ��PWM��ͬһʱ�̣�ֻ����ͬһ��PWM�����
	//����:PWMA_CH1P_P10 �� PWMA_CH1N_P11����һ�������
	PWMA_CH1P_P10 = 0x00,PWMA_CH1N_P11,
	PWMA_CH1P_P20,		 PWMA_CH1N_P21,
	PWMA_CH1P_P60,		 PWMA_CH1N_P61,

	PWMA_CH2P_P12 = 0x10,//���������� USB �ں˵�Դ��ѹ��
	PWMA_CH2N_P13,          
	PWMA_CH2P_P22,		 PWMA_CH2N_P23,
	PWMA_CH2P_P62,		 PWMA_CH2N_P63,

	PWMA_CH3P_P14 = 0x20,PWMA_CH3N_P15,
	PWMA_CH3P_P24,		 PWMA_CH3N_P25,
	PWMA_CH3P_P64,		 PWMA_CH3N_P65,

	PWMA_CH4P_P16 = 0x30,PWMA_CH4N_P17,
	PWMA_CH4P_P26,		 PWMA_CH4N_P27,
	PWMA_CH4P_P66,		 PWMA_CH4N_P67,
	PWMA_CH4P_P34,		 PWMA_CH4N_P33,
	
	//������PWMBͨ����
	//ͬһ��PWM��ͬһʱ�̣�ֻ����ͬһ��PWM�����
	//����:PWMB_CH1_P20 �� PWMB_CH1_P17 ����ͬʱ��� 
	//���ǲ�ͬ��ͨ������ͬһʱ�������
	//����:PWMB_CH1_P20 �� PWMB_CH2_P21����ͬʱ���
	PWMB_CH1_P20 = 0x40,
	PWMB_CH1_P17,
	PWMB_CH1_P00,
	PWMB_CH1_P74,

	PWMB_CH2_P21 = 0x50,
	PWMB_CH2_P54,		//������Ϊ��λ����
	PWMB_CH2_P01,
	PWMB_CH2_P75,

	PWMB_CH3_P22 = 0x60,
	PWMB_CH3_P33,
	PWMB_CH3_P02,
	PWMB_CH3_P76,

	PWMB_CH4_P23 = 0x70,
	PWMB_CH4_P34,
	PWMB_CH4_P03,
	PWMB_CH4_P77,

}PWMCH_enum;


void pwm_init(PWMCH_enum pwmch,uint32 freq, uint32 duty);
void pwm_duty(PWMCH_enum pwmch, uint32 duty);
void pwm_freq(PWMCH_enum pwmch, uint32 freq, uint32 duty);


#endif
