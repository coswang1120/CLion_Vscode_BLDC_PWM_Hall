//############################################################
// FILE: IQ_math.h
// Created on: 2017��1��18��
// Author: XQ
// summary: IQ_math_
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#ifndef __IQ_math_H
#define __IQ_math_H

#include "stm32f10x.h"

//��256����

#define Abs(A)    ((A>=0)?A:-A)  // ����ֵ����
#define Min(A,B)  ((A<=B)?A:B)   // ����С����
#define Max(A,B)  ((A>=B)?A:B)   // �������

#define _IQ15(A) ((int32_t)((A) * (1 << 15)))    // ����15λ 32768
#define _IQ(A) _IQ15(A)                          // ����IQ��ʽ 32768
#define _IQmpy(A, B) (int32_t)((A * B) >> 15)    // IQ��ʽ���
#define _IQ10mpy(A, B) (int32_t)((A * B) >> 10)  // IQ10��ʽ���
#define _IQdiv2(A) (int32_t)((A) >> 1)           // ��2
#define _IQmpy2(A) (int32_t)(A << 1)             // ��2
#define _IQdiv(A, B) (int32_t)((A << 15) / B)    // IQ��ʽ���
#define _IQ15int(A) ((A) >> 15)
typedef int32_t _iq15;

#define   _IQ15_MAX      (0x7FFF)
#define   _IQ15_MIN      (-0x8000)

/*float */
typedef float _iq15f;
#define _IQ15f(A)         (A)
#define _IQ15frac(A)      ((A) - (float)((long)(A)))
#define _IQ15fint(A)             ((int32_t)(A))
#define   _IQmpy_SAT(A,B) ({ \
    int64_t temp = (int64_t)(A) * (B); \
    (int32_t)((temp + (1 << 14)) >> 15); \
})	


#define SIN_RAD     0x0300
#define U0_90       0x0000 
#define U90_180     0x0100
#define U180_270    0x0200
#define U270_360    0x0300

typedef struct {
    int32_t IQAngle;  // ����ż�λ�ýǶ�0---65536����0---360��
    int32_t IQSin;    // IQ��ʽ���Ҳ�����-32768---32767  -1��1
    int32_t IQCos;    // IQ��ʽ���Ҳ�����-32768---32767  -1��1
} IQSin_Cos, *p_IQSin_Cos;

// static inline int32_t _IQ15_CHECK_RANGE(int32_t x) {
//     if (x > _IQ15_MAX) return _IQ15_MAX;
//     if (x < _IQ15_MIN) return _IQ15_MIN;
//     return x;
// }

#define IQSin_Cos_DEFAULTS  {0,0,0} // ��ʼ������

typedef struct {
    int32_t Alpha;      // ���ྲֹ����ϵ Alpha ��
    int32_t Beta;       // ���ྲֹ����ϵ Beta ��
    int32_t IQTan;      // IQ��ʽ���� 45��������1��IQ�ĸ�ʽ��
    int32_t IQAngle;    // IQ��ʽ�Ƕ�ֵ 0---65536 == 0---360��
    int32_t JZIQAngle;  // ����IQ��ʽ�Ƕ�ֵ
} IQAtan, *p_IQAtan;

#define IQAtan_DEFAULTS  {0,0,0,0,0}  // ��ʼ������

uint32_t IQSqrt(uint32_t  M); // ��������
void  IQSin_Cos_Cale(p_IQSin_Cos pV); //��ȡ�����Һ��� 
void  IQAtan_Cale(p_IQAtan  pV) ;  //��ȡ�����Һ���
int32_t IQsat( int32_t Uint,int32_t  U_max, int32_t U_min); //���Ƹ�ֵ����
#endif /* __IQ_math_H */

//===========================================================================
// No more.
//===========================================================================
