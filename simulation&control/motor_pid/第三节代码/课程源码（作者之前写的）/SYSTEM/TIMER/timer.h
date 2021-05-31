#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "oled.h"
#include "led.h"
#include "beep.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//通用定时器 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/03
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

void TIM2_Int_Init(u16 arr,u16 psc); 
void TIM4_NVIC_Init (void);
void TIM4_Init(u16 arr,u16 psc);

 
#endif
