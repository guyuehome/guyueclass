#include<reg52.h>
#include "LPF.h"
#include "LPF_private.h"
#define uint unsigned int
#define uchar unsigned char
#define y P2
uint volt; 
uchar addr;
sbit CLK=P1^7;//定义时钟信号口
sbit DIN=P1^6;//定义2543数据写入口
sbit DOUT=P1^5;//定义2543数据读取口
sbit CS=P1^4;//定义2543片选信号口
sbit P2_5=P2^5;

void read2543(uchar addr)
{
	uint ad=0;
	uchar i;
	CLK=0;
	CS=0;//片选段，启动2543
	addr<<=4;//对地址位预处理
	for(i=0;i<12;i++) //12个时钟走完，完成一次读取测量
	{
		if(DOUT==1)
			ad=ad|0x01;//单片机读取ad数据
		DIN=addr&0x80;//2543读取测量地址位
		CLK=1;
		;;;//很短的延时
		CLK=0;//产生下降沿，产生时钟信号
		;;;
		addr<<=1;
		ad<<=1;//将数据移位准备下一位的读写
	}
	CS=1;//关2543
	ad>>=1;
	volt=ad;//取走转换结果
	}

void main()
{
	addr=0;
	LPF_initialize();
	TMOD=0x01;
	TH0=0xD8;
	TL0=0xF0;
	TR0=1;
	EA=1;
	ET0=1;
	while(1)
	{
		read2543(addr);
	}
}
void Timer0_ISR(void) interrupt 1
{
	TH0=0xD8;
	TL0=0xF0;
	LPF_U.In1=volt;
	LPF_step();
	y=LPF_Y.Out1/16;
	
}