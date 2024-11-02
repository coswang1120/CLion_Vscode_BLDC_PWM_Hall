//############################################################
// FILE:  ThreeHall.h
// Created on: 2017��1��15��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#ifndef __THREEHALL_H
#define __THREEHALL_H
#include "stm32f10x.h"
#include "IQ_math.h"

typedef struct {
    uint8_t HallUVW[3];          // ��ȡ���������Ķ�Ӧ״̬
    uint8_t Hall_State;          // ��ǰ����״̬
    uint8_t OldHall_State;       // ��ʷ����״̬
    int32_t step_angle_error;    // �����Ƕ����
    int32_t step_angle;          // �����Ƕ�
    int32_t step_angleFitter;    // �����Ƕ� �˲�ֵ
    uint16_t Speed_count;        // �ٶȼ���ֵ
    uint16_t Speed_countFitter;  // �ٶȼ����˲�ֵ
    uint16_t Speed_count_old;    // �ٶȼ�����ʷֵ
    uint32_t speed_coeff;        // �ٶ�ϵ��
    uint8_t Poles;               // ���������
    uint8_t Move_State;          // �����ת״̬
    uint16_t Speed_RPM;          // �����ת�ٶ�
    uint16_t Speed_RPMF;         // �����ת�˲��ٶ�
} Hall;

#define  Hall_DEFAULTS {0,0,0,0,0,0,0,0,0,0,4,0,0,0} // ������ʼ��

// ��ͬ�������˳��ͬ��Ҫ�����޸�  ���������ʵ�ַ�ת


#define   Hall_num1   0x6     //��ˢ�����ת�����任˳��
#define   Hall_num2   0x4
#define   Hall_num3   0x5
#define   Hall_num4   0x1
#define   Hall_num5   0x3
#define   Hall_num6   0x2


/*
#define   Hall_num1   0x3     //��ˢ�����ת�����任˳��
#define   Hall_num2   0x1
#define   Hall_num3   0x5
#define   Hall_num4   0x4
#define   Hall_num5   0x6
#define   Hall_num6   0x2
*/
 	
void ThreeHallPara_init(void);  //������������ʼ��
void ThreeHall_huanxkz(void);   //�����������л�PWM����MOS

#endif /* __THREEHALL_H*/
//===========================================================================
// End of file.
//===========================================================================
