//############################################################
// FILE: ThreeHall.c
// Created on: 2017��1��18��
// Author: XQ
// summary: ThreeHall
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################
#include "ThreeHall.h"
#include "Tim1_PWM.h"
#include "Task_function.h"

extern   Hall   Hall_Three;
extern  logic   logicContr;

uint16_t   HallK1=355; 		//�˲�ϵ��K1: 0.05/(T+0.05);
uint16_t   HallK2=669 ;  	//�˲�ϵ�� K2: T/(T+0.05);

void ThreeHallPara_init(void) {
    Hall_Three.Poles = 4;  // 12k����Ƶ��
    Hall_Three.speed_coeff =  120000 / Hall_Three.Poles;  // 93750���� һ�λ����������һ���ٶ�

    //  rpm=speed_coeff
    //  [=60/(Hall_Three.Speed_count*0.00008333*6(6�λ���)*4(4�Լ���))] /count

    // һ�λ���60�ȵ�Ƕ�������ʱ�� pwm_count ��Hall_Three.Speed_count
    // 90950���ڿ������1000RPMʱ��궨���� �û�����ʾ��������ʱ��
    // һ������ʾ����ʱ��    120*1/0.0145/8=1034 rpm
    // һ������ʾ����ʱ��14.5ms����60*(1/0.015)/p(������) = RPM =1000RPM
    // ��90950��λ�����ñ�����Keil���ߵ����޸Ĵ���ֵ
    // Hall_Three.Speed_count*0.0000833(һ��pwm80us) һ�λ�����������ʱ��
    // һ�λ�����1/6����  4�Լ��� 1/24Ȧ һ��еȦ����ʱ���24
    // Hall_Three.Speed_count*0.00008333*24(һ��pwm80us)
    // ����ÿ�����Ȧ1/(Hall_Three.Speed_count*0.00008333*24)
    // ����ÿ���Ӷ���Ȧ 1/(Hall_Three.Speed_count*0.0000833*24)*(1/60)
    // =60/(Hall_Three.Speed_count*0.0000833*24)
    //=60/(Hall_Three.Speed_count*0.00008333*6(6�λ���)*4(4�Լ���))
    // ���ת��rpm=120000/(4*Hall_Three.Speed_count)
}

// CCW  41    16   63   32   25     54
// CW   32    25   54   41   16     63


void ThreeHall_huanxkz(void)  // һ��PWM����ִ��һ��
{
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1)  // C   judge which phase on
        Hall_Three.HallUVW[0] = 1;
    else
        Hall_Three.HallUVW[0] = 0;
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == 1)  // B
        Hall_Three.HallUVW[1] = 1;
    else
        Hall_Three.HallUVW[1] = 0;
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 1)  // A
        Hall_Three.HallUVW[2] = 1;
    else
        Hall_Three.HallUVW[2] = 0;
    Hall_Three.Hall_State = Hall_Three.HallUVW[0] +
                            (Hall_Three.HallUVW[1] << 1) +
                            (Hall_Three.HallUVW[2] << 2);
    if (logicContr.Start_order == 1) {
        if (Hall_Three.Hall_State !=
            Hall_Three.OldHall_State)  // now state != old state
        {
            switch (Hall_Three.Hall_State) {
                case Hall_num1:  // 0x6
                {
                    if (logicContr.Run_mode == 1) MOS_Q41PWM();
                    if (logicContr.Run_mode == 2) MOS_Q32PWM();

                } break;
                case Hall_num2:  // 0x4
                {
                    if (logicContr.Run_mode == 1) MOS_Q16PWM();
                    if (logicContr.Run_mode == 2) MOS_Q25PWM();

                } break;
                case Hall_num3:  // 0x5
                {
                    if (logicContr.Run_mode == 1) MOS_Q63PWM();
                    if (logicContr.Run_mode == 2) MOS_Q54PWM();

                } break;
                case Hall_num4:  // 0x1
                {
                    if (logicContr.Run_mode == 1) MOS_Q32PWM();
                    if (logicContr.Run_mode == 2) MOS_Q41PWM();

                } break;
                case Hall_num5:  // 0x3
                {
                    if (logicContr.Run_mode == 1) MOS_Q25PWM();
                    if (logicContr.Run_mode == 2) MOS_Q16PWM();

                } break;
                case Hall_num6:  // 0x2
                {
                    if (logicContr.Run_mode == 1) MOS_Q54PWM();
                    if (logicContr.Run_mode == 2) MOS_Q63PWM();

                } break;
                default: {
                    Stop_Motor();
                    Hall_Three.Speed_RPM = 0;
                } break;
            }
            // һ�����ֵ�ͨ�Ѷ�������״̬֮��ʱ��ļ���ƽ���˲���HallK1+
            // HallK2=1024����y=a*y0+(1-a)*x;		 //  calculate   speed

            Hall_Three.Speed_countFitter =
                _IQ10mpy(HallK2, Hall_Three.Speed_countFitter) +
                _IQ10mpy(HallK1, Hall_Three.Speed_count);
            Hall_Three.Speed_RPM =
                Hall_Three.speed_coeff / Hall_Three.Speed_countFitter;
            Hall_Three.Speed_RPMF = _IQ10mpy(HallK2, Hall_Three.Speed_RPMF) +
                                    _IQ10mpy(HallK1, Hall_Three.Speed_RPM);
            Hall_Three.Speed_count = 0;
        }

        else if (Hall_Three.Hall_State == Hall_Three.OldHall_State) {
            Hall_Three.Speed_count++;
            if (Hall_Three.Speed_count >= 8000) {
                Hall_Three.Speed_count = 0;
                Hall_Three.Speed_RPMF = 0;

                Stop_Motor();
				

                Hall_Three.step_angleFitter = 0;  // 5     %1      3      %5      1    %3      5
                Hall_Three.Move_State = 0;

                switch (Hall_Three.Hall_State) {
                    case Hall_num1:  // 0x6
                    {
                        if (logicContr.Run_mode == 1) MOS_Q41PWM();
                        if (logicContr.Run_mode == 2) MOS_Q32PWM();

                    } break;
                    case Hall_num2:  // 0x4
                    {
                        if (logicContr.Run_mode == 1) MOS_Q16PWM();
                        if (logicContr.Run_mode == 2) MOS_Q25PWM();

                    } break;
                    case Hall_num3:  // 0x5
                    {
                        if (logicContr.Run_mode == 1) MOS_Q63PWM();
                        if (logicContr.Run_mode == 2) MOS_Q54PWM();

                    } break;
                    case Hall_num4:  // 0x1
                    {
                        if (logicContr.Run_mode == 1) MOS_Q32PWM();
                        if (logicContr.Run_mode == 2) MOS_Q41PWM();

                    } break;
                    case Hall_num5:  // 0x3
                    {
                        if (logicContr.Run_mode == 1) MOS_Q25PWM();
                        if (logicContr.Run_mode == 2) MOS_Q16PWM();

                    } break;
                    case Hall_num6:  // 0x2
                    {
                        if (logicContr.Run_mode == 1) MOS_Q54PWM();
                        if (logicContr.Run_mode == 2) MOS_Q63PWM();

                    } break;
                    default: {
                        Stop_Motor();
                        Hall_Three.Speed_RPM = 0;
                    } break;
                }
            }
        }
}Hall_Three.OldHall_State=Hall_Three.Hall_State ;
}


//===========================================================================
// No more.
//===========================================================================

