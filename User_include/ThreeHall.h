//############################################################
// FILE:  ThreeHall.h
// Created on: 2017年1月15日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#ifndef __THREEHALL_H
#define __THREEHALL_H
#include "stm32f10x.h"
//#include "IQ_math.h"

typedef struct {
    uint8_t HallUVW[3];          // 读取三个霍尔的对应状态
    uint8_t Hall_State;          // 当前霍尔状态
    uint8_t OldHall_State;       // 历史霍尔状态
    int32_t step_angle_error;    // 步进角度误差
    int32_t step_angle;          // 步进角度
    int32_t step_angleFitter;    // 步进角度 滤波值
    uint16_t Speed_count;        // 速度计数值
    uint16_t Speed_countFitter;  // 速度计数滤波值
    uint16_t Speed_count_old;    // 速度计数历史值
    uint32_t speed_coeff;        // 速度系数
    uint8_t Poles;               // 电机极对数
    uint8_t Move_State;          // 电机旋转状态
    uint16_t Speed_RPM;          // 电机旋转速度
    uint16_t Speed_RPMF;         // 电机旋转滤波速度
} Hall;

#define  Hall_DEFAULTS {0,0,0,0,0,0,0,0,0,0,4,0,0,0} // 参数初始化

// 不同电机换向顺序不同需要测试修改  反相序换向就实现反转


#define   Hall_num1   0x6     //无刷电机旋转霍尔变换顺序
#define   Hall_num2   0x4
#define   Hall_num3   0x5
#define   Hall_num4   0x1
#define   Hall_num5   0x3
#define   Hall_num6   0x2


/*
#define   Hall_num1   0x3     //无刷电机旋转霍尔变换顺序
#define   Hall_num2   0x1
#define   Hall_num3   0x5
#define   Hall_num4   0x4
#define   Hall_num5   0x6
#define   Hall_num6   0x2
*/
 	
void ThreeHallPara_init(void);  //三霍尔参数初始化
void ThreeHall_huanxkz(void);   //根据三霍尔切换PWM驱动MOS

#endif /* __THREEHALL_H*/
//===========================================================================
// End of file.
//===========================================================================
