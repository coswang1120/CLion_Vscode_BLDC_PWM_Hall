//############################################################
// FILE:  ADC_int.h
// Created on: 2017��1��9��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#ifndef __ADC_INT_H
#define __ADC_INT_H

#include "stm32f10x.h"

typedef struct {
	     int32_t   BUS_CurrF ;     // ĸ�ߵ��� DC Bus  Current
	     int32_t   BUS_Curr ;     // ĸ�ߵ��� DC Bus  Current
	     int32_t   PhaseU_Curr;   // U����� Phase U Current
	     int32_t   PhaseV_Curr;   // V�����Phase V Current
	     int32_t   BUS_Voltage ;  //ĸ�ߵ�ѹDC Bus  Voltage
	     int32_t   RP_speed_Voltage ;   // ��λ����ѹ RP1_Voltage
	     int32_t   OffsetBUS_Curr ;     // ĸ�ߵ���ƫִֵ DC Bus  Current
	     int32_t   OffsetPhaseU_Curr;   // U�����ƫִֵ  Phase U Current
	     int32_t   OffsetPhaseV_Curr;   // V�����ƫִֵ Phase V Current
	     int32_t   Coeff_filterK1;   // һ�׵�ͨ�˲���ϵ��1
		 int32_t   Coeff_filterK2;   // һ�׵�ͨ�˲���ϵ��2
} ADCSamp;

#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,0,1000,24}   // ��ʼ������

void  Offset_CurrentReading(void);  //ƫִ��ѹ��ȡ
void  DMA_Configuration(void);      //DMA����
void  ADC1_Configuration(void);     //ADC1����
void  ADC_Sample(void );            //��������
void  Protection_software(void);//
#endif  // __ADC_INT_H

