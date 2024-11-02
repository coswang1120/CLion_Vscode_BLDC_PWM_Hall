//############################################################
// FILE:  Usart_RS232.c
// Created on: 2017��1��18��
// Author: XQ
// summary: SCI_RS232
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����   
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#include "Usart_RS232.h"
#include "Timer.h"
#include "Task_function.h"
#include "stdio.h"
#include <math.h>
#include <string.h>
#include "stdlib.h"
#include "ADC_int.h"
#include "PI_Cale.h"
#include "IQ_math.h"

//extern char dataspeed;


extern u16 USART_RX_STA; 
uint8_t ch='0';
extern  uint16_t  DUTY;
static int kk=0;	
//static uint8_t  nums[6];
//static int kkk=0;;
extern long numdec;
extern    ADCSamp     ADCSampPare;
extern  Test           TestPare;
extern  TaskTime       TaskTimePare;
extern  logic          logicContr;
extern float syst;
extern  PI_Control   pi_spd ;
extern  PI_Control   pi_ICurr ;
//static u8 revnum=0;	
u32 Tag=0;
extern u8 Res;

//���ՠ�B
//bit15��	������ɘ��I
//bit14��	���յ�0x0d
//bit13~0��	���յ�����ЧλԪ�M��Ŀ


void Usart3_RS232_init(void)
{
	USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel =  USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
 
  USART_DeInit(USART3);
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init( USART3, &USART_InitStructure);
    
  USART_ITConfig(USART3, USART_IT_RXNE , ENABLE);
  USART_Cmd( USART3, ENABLE);  
}

void Uart3_Sendlen(uint8_t *data,u8 len)
{
	USART3->SR;
	do{
		USART_SendData(USART3,*data);
		while(!(USART3->SR&0x40));
		data++;  
		len--;
	}while(len!=0);
}
 
void  Uart3_RS232TX_sen(void)
{     
	int ccw;
	//int32_t x=0,y;
	//float frac;
	//_iq15f Y0=5538.569;
	//_iq15f Y0frac;
	//int32_t Y0int;
	//float xx=32.1416;
  //_iq15 yy;
	//_iq15 mx,my,mxy;
	//int32_t mpyx;		
	
  if(TaskTimePare.Tim500ms_flag==1)
  {
   	TestPare.Uart_txdr[0] = 0xaa ; // �������֡ͷaa 
   	TestPare.Uart_txdr[1] = TestPare.fact_BUS_Voil>>8; //����λ�����û��������и���������
   	TestPare.Uart_txdr[2] = TestPare.fact_BUS_Voil  ;  //
   	TestPare.Uart_txdr[3] = TestPare.Speed_target>>8 ; //
   	TestPare.Uart_txdr[4] = TestPare.Speed_target  ;   //
   	TestPare.Uart_txdr[5]=  logicContr.Run_mode ;      //
   	TestPare.Uart_txdr[6] = logicContr.Start_order ;
   	TestPare.Uart_txdr[7] = 0xcc ;  // �������֡βcc	
		
    //printf("speed : %04x  Run_mode:  %04x\r\n",TestPare.Uart_txdr[4],logicContr.Run_mode);
		if (logicContr.Run_mode==1)
		    {
				    ccw=1;				
				}
		else
		    {		
		        ccw=-1;
		    }		
		
		//printf("%c \r\n",ch);
				
		
			
		//syst=syst+0.5;
				
		//printf("Others:%d\r\n",DUTY);
				


////		x=3;
////		y=_IQ15int(x);
////		frac=_IQ15f(65533.7);				
////		Y0frac=_IQ15frac(65533.7);	
////		printf("%d ,%d,%f,%f \r\n", x,y,frac,Y0frac);
				
	
		//mpyx=_IQmpy(12000,100);
		//printf("%d \r\n", mpyx);		
		//y=(int32_t)((655354.9)<<15);	



		// (float)TestPare.Speed_fact  [true rotaional speed]   (int)logicContr.Run_mode    (int)ccw   (int)logicContr.Control_Mode       (float)TestPare.fact_BUS_Curr    (float)TestPare.fact_BUS_Voil    (float)ADCSampPare.BUS_CurrF;	  (float)ADCSampPare.PhaseV_Curr   (float)ADCSampPare.BUS_Voltage   (float)ADCSampPare.PhaseU_Curr
	  //printf("%4.2f,%d,%d,%4.2f,%4.2f,%4.2f,%4.2f,%6.2f,%6.2f \r\n",(float)TestPare.Speed_fact,(int)logicContr.Run_mode,(int)logicContr.Control_Mode,(float)ADCSampPare.BUS_CurrF,(float)ADCSampPare.PhaseV_Curr,(float)ADCSampPare.BUS_Voltage,(float)ADCSampPare.PhaseU_Curr,(float)pi_spd.OutF,(float)pi_ICurr.OutF);



		////Uart3_Sendlen(TestPare.Uart_txdr,8);   // old strategy with vendor
		  			
 	}
}
 
void  ReceiveData_chuli(void)  // ����ͨѶ�жϽ������ݵĴ�����
{
  uint8_t i=0; 
 if( TestPare.Rece_flag==1)  // ���������ж���ı�־λ
 {
	 TestPare.Rece_flag=0;
   for(i=0;i<8;i++   )
   TestPare.Uart_rxdr[i]  = TestPare.Uart_rxdr[i];
	
 }
 
}


int tran_asci(uint8_t uart0read)
{
    uint8_t decnum;
		switch(uart0read)
							 {
							 case 0x30 :
									decnum=0;
							    kk=kk+1;
									break;
							 case 0x31 :
									decnum=1;
							    kk=kk+1;
									break;
							 case 0x32 :
									decnum=2;
							    kk=kk+1;
									break;
							 case 0x33 :
									decnum=3;
							    kk=kk+1;
									break;
							 case 0x34 :
									decnum=4;
							    kk=kk+1;
									break;
							 case 0x35 :
									decnum=5;
							    kk=kk+1;
									break;
								case 0x36 :
									decnum=6;
							    kk=kk+1;
									break;
							 case 0x37 :
									decnum=7;
							    kk=kk+1;
									break;
							 case 0x38 :
									decnum=8;
							    kk=kk+1;
									break;
							 case 0x39 :
									decnum=9;
							    kk=kk+1;
									break;
							 case 0x40 :
									kk=0;
									break;
							 
							 }
      return decnum;
}

int32_t tran_dec(uint8_t *nums, int k)
{
  int    sum = 0;

	switch(k)
		 {
		 case 1 :			
				sum = nums[0];						
				break;
		 case 2 :
				sum = 10*nums[0]+1*nums[1];	
				break;
		 case 3 :
				sum = 100*nums[0]+10*nums[1]+1*nums[2];	
				break;
		 case 4 :
				sum = 1000*nums[0]+100*nums[1]+10*nums[2]+1*nums[3];	
				break;
		 case 5 :
				sum = 10000*nums[0]+1000*nums[1]+100*nums[2]+10*nums[3]+1*nums[4];
				break;
		 case 6 :
				sum = 100000*nums[0]+10000*nums[1]+1000*nums[2]+100*nums[3]+10*nums[4]+1*nums[5];
				break;								 
						 }
	//printf("%6d\n",sum);
  return sum;
}



// m

//===========================================================================
// No more.
//===========================================================================
