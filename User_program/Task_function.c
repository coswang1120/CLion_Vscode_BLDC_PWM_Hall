//############################################################
// FILE: Task_function.c
// Created on: 2017��1��18��
// Author: XQ
// summary: Task_function
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#include "Task_function.h"
#include "Timer.h"
#include "PI_Cale.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"

extern ADCSamp        ADCSampPare;
extern  TaskTime       TaskTimePare;
extern  PI_Control     pi_spd  ;
extern  logic          logicContr;
extern  u16 spdcmd;
extern    logic       logicContr;
void knob_control(void)
{
//  ��λ������0.3V���ҿ�ʼ����PWM�����ٶȺ͵�ѹ 
	
	
	if (logicContr.Control_Mode==2){
	
	 if (( ADCSampPare.RP_speed_Voltage<= 350)  ) // ��λ����תһ��λ�ÿ�ʼ�����ٶ�������Ƶ��
     {
	// �������������Ҫ��ʼ������������PI��·����	
		logicContr.drive_car=0;
			 
			 
		logicContr.Start_order=0;
		logicContr.Qiehuan_count=0;

		pi_spd.Fbk=0;
		pi_spd.Ref=0;
		pi_spd.up=0;
	    pi_spd.ui=0;
	    pi_spd.v1=0;
	    pi_spd.i1=0;
	    pi_spd.Out=0;
	    pi_spd.OutF=0;
 
	}
	else
	{
	    logicContr.drive_car=1;
 	    pi_spd.Ref =  ADCSampPare.RP_speed_Voltage-300;   // ��λ��ת���ź�
 	    if( pi_spd.Ref>=2500)   //�������ת��
 	    pi_spd.Ref=2500;
	}

     if ((logicContr.olddrive_car==0) &&( logicContr.drive_car==1 ))
     {
		logicContr.Start_order=1;
     }
     else if ((logicContr.olddrive_car==1) &&( logicContr.drive_car==1 ))
	   { 
	   }
     else     // (logicContr.olddrive_car==1) &&( logicContr.drive_car==0 )        (logicContr.olddrive_car==0) &&( logicContr.drive_car==0 )
   	 {
		  Stop_Motor();
		  logicContr.Start_order=0;
	   }

		 
	 }
   else{    //  logicContr.Control_Mode==1
	 
				 if (( ADCSampPare.RP_speed_Voltage<= 350)  ) // ��λ����תһ��λ�ÿ�ʼ�����ٶ�������Ƶ��
     {
	// �������������Ҫ��ʼ������������PI��·����	
		 logicContr.drive_car=0;
			 
			 
		 logicContr.Start_order=0;
		 logicContr.Qiehuan_count=0;

		 pi_spd.Fbk=0;
		 pi_spd.Ref=0;
	   pi_spd.up=0;
	   pi_spd.ui=0;
	   pi_spd.v1=0;
	   pi_spd.i1=0;
	   pi_spd.Out=0;
	   pi_spd.OutF=0;
 
	}
	else
	{
	    logicContr.drive_car=1;
 	    pi_spd.Ref =  ADCSampPare.RP_speed_Voltage-300;   // ��λ��ת���ź�
 	    if( pi_spd.Ref>=2500)   //�������ת��
 	    pi_spd.Ref=2500;
	}

     if ((logicContr.olddrive_car==0) &&( logicContr.drive_car==1 ))
     {
   	  logicContr.Start_order=1;
     }
     else if ((logicContr.olddrive_car==1) &&( logicContr.drive_car==1 ))
	   { 
	   }
     else     // (logicContr.olddrive_car==1) &&( logicContr.drive_car==0 )        (logicContr.olddrive_car==0) &&( logicContr.drive_car==0 )
   	 {
		  Stop_Motor();
		  logicContr.Start_order=0;
	   }

				 
				 }
	logicContr.olddrive_car=logicContr.drive_car;
 
}


//===========================================================================
// No more.
//===========================================================================
