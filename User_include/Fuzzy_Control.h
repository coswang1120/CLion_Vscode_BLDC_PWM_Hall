#ifndef __FUZZY_CONTROL_H
#define __FUZZY_CONTROL_H

//#include "stm32f10x.h"

// 模糊控制器結構體
typedef struct {
    float Error;      // 誤差
    float LastError;  // 上次誤差
    float Output;     // 輸出
    float Umax;       // 最大輸出限制
    float Umin;       // 最小輸出限制
    
    // 模糊規則基礎參數
    float KE;         // 誤差縮放因子
    float KEC;        // 誤差變化縮放因子
    float KU;         // 輸出縮放因子
} Fuzzy_Control, *p_Fuzzy_Control;

//typedef Fuzzy_Control *p_Fuzzy_Control;

#define Fuzzy_Control_DEFAULTS {0,0,0,0,0,0,0,0}  // 初始化参数


// 函數聲明
void Fuzzy_Controller(p_Fuzzy_Control pV);
void Fuzzy_Pare_init(void);
float getMembership(float x, float left, float middle, float right) ;
//extern Fuzzy_Control fuzzy_spd;
//extern Fuzzy_Control fuzzy_curr;

#endif 
