#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "oled.h"

void TIM4_NVIC_Init (void);
void TIM4_Init(u16 arr,u16 psc);

#endif
