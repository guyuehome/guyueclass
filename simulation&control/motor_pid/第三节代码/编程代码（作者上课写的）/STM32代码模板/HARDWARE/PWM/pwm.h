#ifndef __pwm_h
#define __pwm_h
#include "sys.h"
void TIM3_PWM_Config_Init(u16 Psc,u16 Per);
void TIM3_SetPWM_Num(u16 value,u8 ch);
#endif

