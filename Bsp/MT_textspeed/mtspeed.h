#ifndef __MTSPEED_H
#define __MTSPEED_H	 

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


typedef struct{
	__IO uint8_t Endup_Flag; 																 //������־
	__IO uint8_t Startup_Flag;                                //��ʼ�ɼ���־
	__IO uint8_t timecountflag;
	__IO uint8_t startcount;
	__IO uint16_t timecount;  
	__IO uint16_t speed_zero;
	__IO uint32_t Total_Time_M1;                               //M1  Ƶ��
	__IO uint32_t Total_Time_M2;                               //M2
  __IO uint32_t memory_value;
  uint16_t Rotate_Speed;                                   //ת��
}mt_rotate;

#ifdef __cplusplus
}
#endif

#endif  
