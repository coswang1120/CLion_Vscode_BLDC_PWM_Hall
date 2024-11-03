//############################################################
// FILE: Task_function.c
// Created on: 2017年1月18日
// Author: XQ
// summary: Task_function
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// 版权所有，盗版必究
// DSP/STM32电机控制开发板
// 硕历电子
// 网址: https://shuolidianzi.taobao.com
// Author-QQ: 616264123
// 电机控制QQ群：314306105
// 控制最外控制  ADCSampPare.RP_speed_Voltage  pi_spd.Ref 最大2500
//############################################################
#include "Task_function.h"
#include "Timer.h"
#include "PI_Cale.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"
extern ADCSamp ADCSampPare;
extern TaskTime TaskTimePare;
extern PI_Control pi_spd;
extern logic logicContr;
extern u16 spdcmd;
extern u8 switchRX;

void InitializeParameters() {
    logicContr.drive_car = 0;
    logicContr.Start_order = 0;
    logicContr.Qiehuan_count = 0;
    pi_spd.Fbk = 0;
    pi_spd.Ref = 0;
    pi_spd.up = 0;
    pi_spd.ui = 0;
    pi_spd.v1 = 0;
    pi_spd.i1 = 0;
    pi_spd.Out = 0;
    pi_spd.OutF = 0;
}

void knob_control(void) {
    if (ADCSampPare.RP_speed_Voltage <= 350) {
        InitializeParameters();
    } else {
        logicContr.drive_car = 1;
        pi_spd.Ref = ADCSampPare.RP_speed_Voltage - 300;
        if (pi_spd.Ref >= 2500)
            pi_spd.Ref = 2500;
    }

    if (logicContr.olddrive_car == 0 && logicContr.drive_car == 1) {
        logicContr.Start_order = 1;
    } else if (logicContr.olddrive_car == 1 && logicContr.drive_car == 1) {
        // 保留空条件逻辑
    } else { // logicContr.olddrive_car 为 1 && logicContr.drive_car 为 0 或 logicContr.olddrive_car 为 0 && logicContr.drive_car 为 0
        Stop_Motor();
        logicContr.Start_order = 0;
    }

    logicContr.olddrive_car = logicContr.drive_car;
}