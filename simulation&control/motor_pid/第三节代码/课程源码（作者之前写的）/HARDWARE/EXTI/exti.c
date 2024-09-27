#include "exti.h"
#include "delay.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//外部中断 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/01  
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved	  
////////////////////////////////////////////////////////////////////////////////// 	  
 
 
//外部中断初始化函数

volatile unsigned int Motor_Count=0;

void Exit_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;                //结构体
	EXTI_InitTypeDef EXTI_InitStructure;							 	//定义EXTI结构体	
	NVIC_InitTypeDef NVIC_InitStructure;						 	  //定义NVIC结构体	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO ,ENABLE);
													  //开启GPIOA和复用功能的时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;           //选择1~4号引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //io口速度
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;        //配置为上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);              //初始化PA1

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
																										  //配置端口为中断模式

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;       
																										  //选择线路
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置 EXTI 线路为中断请求
	EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Rising;//上升沿降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
				//用来定义选中线路的新状态。它可以被设为 ENABLE 或者 DISABLE，我们ENABLE。
	EXTI_Init(&EXTI_InitStructure);                    //初始化外部中断

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;         //选择通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//主优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //用以使能或者失能指定的中断通道
	NVIC_Init(&NVIC_InitStructure);														//初始化NVIC中断控制器	
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus (EXTI_Line1)==1)
	{
		Motor_Count++;
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
