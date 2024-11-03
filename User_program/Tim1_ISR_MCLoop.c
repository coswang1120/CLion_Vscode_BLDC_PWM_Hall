//############################################################
// FILE: Tim1_ISR_MCLoop.c
// Created on: 2017��1��15��
// Author: XQ
// summary: Tim1_ISR_MCLoop
// ��ʱ��1������ƣ� �жϻ�·�ջ�����
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2018/7/12
//�汾��V17_3
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#include "Tim1_ISR_MCLoop.h"
#include "ADC_int.h"
//#include "Tim1_PWM.h"
//#include "GPIO_int.h"
#include "ThreeHall.h"
#include "Task_function.h"
#include "PI_Cale.h"
#include "Timer.h"
#include "printf_uart.h"
//#include "Tim4_Encoder_PWMDAC.h"
//#include "stdio.h"
extern   PI_Control   pi_ICurr;
extern   PI_Control   pi_spd ;
extern   ADCSamp    ADCSampPare;
extern   uint16_t   PWM_DUTY[3] ;
extern   Hall       Hall_Three;
extern   logic      logicContr;
extern   TaskTime   TaskTimePare;
extern   Test       TestPare;

extern  uint16_t  DUTY;
extern  u16 spdcmd;

uint16_t  FilK1=328;
uint16_t  FilK2=696;
// һ�����ֵ�ͨ�˲���   328+696=1024  a=696/1024     (696/1024) /(2*3.14*0.02) = 54hz  cutoff freq.
// https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search

void TIM1_UP_IRQHandler(void)  // ����ADC�жϲ����͵����·����   83.333us
{
    // �˳�������ж�һֱִ��

    ADC_Sample();  // ����ĸ�ߵ�����ѹ�ɼ�  PWM�жϽ����ȶ�ȡAD������λ�û���

    ThreeHall_huanxkz();  // �����������л�PWM����MOS   //  control comand

    if (logicContr.drive_car == 0)  //
    {
        Offset_CurrentReading();  // ��������ƫִ��ѹ��ȡ
    }

    TaskTimePare.PWMZD_count++;

    if (TaskTimePare.PWMZD_count > 24)  // 2msʱ������ ,[ ����24*83.333us=2msʱ������ 12K����Ƶ�� ]
    {
        TaskTimePare.PWMZD_count = 0;

        knob_control();  // ת�ѵ�λ�����ٵĿ���

        // ͨѶ���Բ������
        // ���Բ������������Ϊ������Բ���������ѹ����ת�ٵȲ������Ա�ͨѶ�����������
        TestPare.fact_BUS_Voil = ADCSampPare.BUS_Voltage;
        TestPare.fact_BUS_Curr = ADCSampPare.BUS_CurrF;

        TestPare.Speed_fact = pi_spd.Fbk; // Hall_Three.Speed_RPMF;    //  0--4000rpm

        TestPare.Speed_target = pi_spd.Ref;  //  pi_spd.Ref =  ADCSampPare.RP_speed_Voltage-300;   // ��λ��ת���ź�

        //---------------�ٶȻ�------------------------//    speed
        // resistometer controller
        pi_spd.Fbk = Hall_Three.Speed_RPMF;    //  0--4000rpm
        PI_Controller((p_PI_Control)&pi_spd);  // �ٶȻ�PI����
        pi_spd.OutF = _IQ10mpy(FilK1, pi_spd.OutF) + _IQ10mpy(FilK2, pi_spd.Out);
        //---------------------------------------------//

        //---------------ĸ�ߵ�����-----------------------------//
        pi_ICurr.Ref = ADCSampPare.RP_speed_Voltage / 20 + 300;  // 0.8A+1A=1.8A ���Ŵ���� ��20��С��Ӳ�����ܳ����۶�˿3A

        pi_ICurr.Fbk = ADCSampPare.BUS_CurrF;  // 0.10A��ϵͳ����

        PI_Controller((p_PI_Control)&pi_ICurr);  // ������PI����
        pi_ICurr.OutF = _IQ10mpy(FilK1, pi_ICurr.OutF) + _IQ10mpy(FilK2, pi_ICurr.Out);
        //-------------------------------------------------//

        if (logicContr.Control_Mode == 1) {
            DUTY = 2 * pi_spd.Ref;  // �ٶ� ��ѹ���� PWM���    max
                                    // (pi_spd.Ref)=2500
                                    // DUTY = 2*spdcmd;
            // printf("%d \r\n",DUTY);

            // 	  if(DUTY>=2500){   //�������ת��
            //   DUTY=2500;
            // 		}

        } else if (logicContr.Control_Mode ==2)  // �������ٶȻ��ջ���ĸ�ߵ����ջ�������ĸ�ߵ�������
        {
            //---------------������ �ٶȻ�
            // ����������·Ӧ��------------------------//
            if (pi_ICurr.OutF > pi_spd.OutF) {
                DUTY = pi_spd.OutF;  // �����ٶȱջ�
                // pi_ICurr.ui = pi_spd.OutF;
                // pi_ICurr.i1 = pi_spd.OutF;
            } else {
                DUTY = pi_ICurr.OutF;  // ����ĸ�ߵ������ƻ�·����
                // pi_spd.ui = pi_ICurr.OutF;
                // pi_spd.i1 = pi_ICurr.OutF;
            }
            //----------------------------------------------------//
        }
    }

    Protection_software();  // �������ֱ�ӹرչ���
                            // 600��3A���ң��ɵ����޸ģ�Ӳ����׼�����3A

    TIM_SetCompare3(TIM4, Hall_Three.Speed_RPMF);  // DACͨ��1   ( demo )
    TIM_SetCompare4(TIM4, DUTY);  // DACͨ��2    you can use microwave to see figure    ( demo )

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
}

//===========================================================================
// No more.
//===========================================================================
