 //############################################################
// FILE:  PI_Cale.c
// Created on: 2017��1��4��
// Author: XQ
// summary: PI_Cale
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#include "PI_Cale.h"

extern  PI_Control   pi_spd ;
extern  PI_Control   pi_ICurr ;


void  PI_Controller(p_PI_Control  pV) // ֻҪ ݔ��  pV->Ref  pV->Fbk  pV->Out  pV->v1
{
    /* proportional term */
	pV->up = pV->Ref - pV->Fbk;

	/* integral term */
	pV->ui = (pV->Out == pV->v1)?(_IQmpy(pV->Ki, pV->up)+ pV->i1) : pV->i1; // integral term 
	pV->i1 = pV->ui;                                                        // history integral term 

	/* control output */ // _IQmpy ����λ2��^15�η� ����Kp/32768
	pV->v1 = _IQmpy(pV->Kp, (pV->up )) + pV->i1;
	pV->Out=IQsat( pV->v1, pV->Umax,  pV->Umin); //�������
}

void PI_Pare_init(void) {
    //_IQmpy ����λ2��^15�η� ����Kp/32768   Ki/32768
    pi_spd.Kp = 12000;   // _
    pi_spd.Ki = 1060;    //    T* ��·���� /0.2
    pi_spd.Umax = 5950;  //
    pi_spd.Umin = 0;

    pi_ICurr.Kp = 9000;    // _
    pi_ICurr.Ki = 880;     //    T* ��·���� /0.2
    pi_ICurr.Umax = 5950;  //
    pi_ICurr.Umin = 0;
}

//===========================================================================
// No more.
//===========================================================================

