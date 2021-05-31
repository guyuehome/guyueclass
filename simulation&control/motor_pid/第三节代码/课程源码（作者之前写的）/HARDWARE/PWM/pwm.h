#ifndef __PWM_H
#define __PWM_H	 

#include "sys.h"  


void TIM3_PWM_Config_Init(u16 Psc,u16 Per);
extern void TIM3_SetPWM_Num(u16 value,u8 ch);

#endif
