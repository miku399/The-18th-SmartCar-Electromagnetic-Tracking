/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"


/*
 *�����ں�Ƶ�ʵ��趨�����Բ鿴board.h�ļ�
 *��board_init��,�Ѿ���P54��������Ϊ��λ
 *�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */
 
void main()
{
    WTST = 0;               //���ó������ȴ���������ֵΪ0�ɽ�CPUִ�г�����ٶ�����Ϊ���
    
	DisableGlobalIRQ();		//�ر����ж�
	
    //sys_clk��ѡֵ:35000000��30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600��
    //����ϵͳƵ�ʣ���Ƶ����Ҫ��STC-ISP����е� <�����û��������е�IRCƵ��>ѡ���Ƶ��һ�¡�
    //���Ƶ�����ò��ԣ����ᵼ�´��ڵ����ݲ�����,PWM�Ĺ����������ȵȡ�
    sys_clk = 35000000;     //����ϵͳƵ��Ϊ35MHz
    
	board_init();			//��ʼ���Ĵ���
	//�˴���д�û�����(���磺�����ʼ�������)
	iap_init();
	ips114_init();    
	dl1a_init();
	icm20602_init();
	Pid_Inc_Init();
	PIDInit();
//	imu660ra_init();			//���������ǳ�ʼ��
	
	adc_init(ADC_P00, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P0.0ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
  adc_init(ADC_P01, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P0.1ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
	adc_init(ADC_P05, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P0.5ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
  adc_init(ADC_P06, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P0.6ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
	
	ctimer_count_init(SPEEDL_PLUSE);	//��ʼ����ʱ��0��Ϊ�ⲿ����
	ctimer_count_init(SPEEDR_PLUSE);	//��ʼ����ʱ��3��Ϊ�ⲿ����
	//���ж������
	EnableGlobalIRQ();		//�������ж�

	  pit_timer_ms(TIM_1, 4);
		pit_timer_ms(TIM_2, 4);
		pit_timer_ms(TIM_4, 4);
	 
  pwm_init(PWMA_CH1P_P60, 17000, 0); //��ʼ��PWM1  ʹ��P60����  ��ʼ��Ƶ��Ϊ17Khz
	pwm_init(PWMA_CH2P_P62, 17000, 0); //��ʼ��PWM2  ʹ��P62����  ��ʼ��Ƶ��Ϊ17Khz
	
	pwm_init(PWMA_CH3P_P64, 17000, 0); //��ʼ��PWM3  ʹ��P64����  ��ʼ��Ƶ��Ϊ17Khz
	pwm_init(PWMA_CH4P_P66, 17000, 0); //��ʼ��PWM4  ʹ��P66����  ��ʼ��Ƶ��Ϊ17Khz
	
	pwm_init(PWMB_CH4_P77,  17000, 0); //��ʼ��PWM3   ʹ��P77����  ��ʼ��Ƶ��Ϊ17Khz
	pwm_init(PWMB_CH3_P33,  17000, 0);  //��ʼ��PWM4  ʹ��P33��     ��ʼ��Ƶ��Ϊ17Khz   ��ѹ����ʹ����ˢǿ�ŵ��

    while(1)
	{			
		UI_Disp();
	  keyScan();
		}
	}
//}

