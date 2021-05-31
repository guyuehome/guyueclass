#include "tim.h"


void TIM4_Init(u16 arr,u16 psc)        							  //TIM3 初始化 arr重装载值 psc预分频系数
{
  	TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStrue;//声明TIM结构体
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//使能TIM3
    TIM4_NVIC_Init (); 																 //开启TIM3中断向量
	      
    TIM_TimeBaseInitStrue.TIM_Period=arr; 												//设置自动重装载值
    TIM_TimeBaseInitStrue.TIM_Prescaler=psc; 											//预分频系数
    TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; 		//计数器向上溢出
    TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; 				//时钟的分频因子，起到了一点点的延时作用，一般设为TIM_CKD_DIV1
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue); 								//TIM3初始化设置
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);										//设置中断类型，使能TIM3中断    
    TIM_Cmd(TIM4,ENABLE);																				  //设置定时器中断优先级，使能TIM3
}

void TIM4_NVIC_Init (void)                                       //开启TIM3中断向量
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;	   //设置抢占和子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}
