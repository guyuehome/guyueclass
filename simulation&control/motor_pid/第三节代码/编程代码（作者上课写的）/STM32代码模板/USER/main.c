//ALIENTEK miniSTM32开发板实验1
//跑马灯实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
#include "delay.h"
#include "sys.h"

/*
int main(void)
{	
	延时初始化	
	中断初始化
	OLED初始化
	PWM初始化
	定时器初始化
	电机初始化
	while(1)
	{	
		OLED显示函数
	}
}

*/
int pwm=0;		
int count;     //oled显示速度
int aim_speed=50;
char oledBuf[20];
int main(void)
{	
	delay_init();	    	 //延时函数初始化	
	Exit_Init();			//中断函数初始化
	
	OLED_Init();
	OLED_ColorTurn(0);//0正常显示，1 反色显示
	OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
	OLED_Clear();
	
	TIM3_PWM_Config_Init(0,9999);	
	TIM4_Init(500,7199);//50ms
	Motor_Init();
	while(1)
	{	
		sprintf(oledBuf," Motor_Control");
		OLED_ShowString(0,0,(u8*)oledBuf,16);//第一行显示	
		sprintf(oledBuf,"Aim_Speed:%3d",aim_speed);
		OLED_ShowString(0,16,(u8*)oledBuf,16);//第二行显示
		sprintf(oledBuf,"Act_Speed:%3d",count);
		OLED_ShowString(0,32,(u8*)oledBuf,16);//第三行显示
		sprintf(oledBuf,"Pwm_data;%5d",pwm); 
		OLED_ShowString(0,48,(u8*)oledBuf,16);//第四行显示
		OLED_Refresh();  //屏幕刷新
	}
}
void TIM4_IRQHandler(void)														//TIM4中断处理函数
{ 	 
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == 1)		//判断是否是TIM4中断
	{	
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);     //清楚中断标志位
		pwm=PID_control(Motor_Count,aim_speed);
		/*获取编码器采集到的脉冲数，得到电机的速度*/
		count=Motor_Count;
		TIM3_SetPWM_Num(motor_abs(pwm),1);
		Motor_Count=0;
		
    }
}