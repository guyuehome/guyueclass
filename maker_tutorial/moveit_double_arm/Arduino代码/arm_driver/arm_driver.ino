#include "Servo.h"	//引用Arduino舵机驱动头文件

#include <ros.h>	//引用ros头文件
#include <sensor_msgs/JointState.h>	//引用sensor_msgs的JointState类型的数据

ros::NodeHandle nh;	//声明ROS句柄

//实例化舵机对象
Servo joint1;
Servo joint2;
Servo joint3;
Servo joint4;
Servo joint5;
Servo joint6;

/****************************************************************************
*函数名称：arm_callback														*
*函数类型：void																*
*函数功能：当订阅到joint_states话题时，将该话题的positio数据内容给舵机执行	*
*输入参数：data，当前订阅到的joint_states的内容								*
*输出参数：无																*
****************************************************************************/
void arm_callback( const sensor_msgs::JointState& data )
{	
	//joint_states的position内容是对应joint的弧度值，position的0对应我们机械臂的90°
	//-1.57~1.57（弧度值）对应0°~180°（角度值），所以有了下面的计算
	//由于计算的结构取整可能会超过0~180的范围，所以用了if...else if ...

	int joint1_angle = data.position[7]*360/6.28+90;
	if(joint1_angle<0)
		joint1_angle = 0;
	else if(joint1_angle>180)
		joint1_angle = 180;
	joint1.write(joint1_angle);
	
	int joint2_angle = data.position[8]*360/6.28+90;
	if(joint2_angle<0)
		joint2_angle = 0;
	else if(joint2_angle>180)
		joint2_angle = 180;
	joint2.write(joint2_angle);
	
	int joint3_angle = data.position[9]*360/6.28+90;
	if(joint3_angle<0)
		joint3_angle = 0;
	else if(joint3_angle>180)
		joint3_angle = 180;
	joint3.write(joint3_angle);
	
	int joint4_angle = data.position[10]*360/6.28+90;
	if(joint4_angle<0)
		joint4_angle = 0;
	else if(joint4_angle>180)
		joint4_angle = 180;
	joint4.write(joint4_angle);
	
	int joint5_angle = data.position[11]*360/6.28+90;
	if(joint5_angle<0)
		joint5_angle = 0;
	else if(joint5_angle>180)
		joint5_angle = 180;
	joint5.write(joint5_angle);

  /*
	int joint6_angle = data.position[12]*360/6.28+90;	
	if(joint6_angle<0)
		joint6_angle = 0;
	else if(joint6_angle>180)
		joint6_angle = 180;
	joint6.write(joint6_angle);
 */
}

//实例化订阅，订阅的话题名为joint_states，话题消息类型为sensor_msgs::JointState，回调函数为arm_callback
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", &arm_callback );

/****************************
*函数名称：setup			*
*函数类型：void				*
*函数功能：初始化相关硬件	*
*输入参数：无				*
*输出参数：无				*
****************************/
void setup()
{
  nh.initNode();	//初始化节点
  nh.subscribe(sub);	//订阅

  joint1.attach(3);	//joint1在D3引脚
  joint2.attach(5);	//joint2在D5引脚
  joint3.attach(6);	//joint3在D6引脚
  joint4.attach(9);	//joint4在D9引脚
  joint5.attach(10);	//joint5在D10引脚
  joint6.attach(11);	//joint6在11引脚
  
  pinMode(13,OUTPUT);	//设置D13引脚为输出模式
  
  //D13引脚在Arduino上面默认有个LED
  //在我们的驱动板当中D13有连接了一个蜂鸣器
  //这里的功能是开机提示
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
  delay(500);
  
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
  delay(500);
  
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
  delay(500);
}

/************************************
*函数名称：loop						*
*函数类型：void						*
*函数功能：程序执行内容，循环执行	*
*输入参数：无						*
*输出参数：无						*
************************************/
void loop()
{
  nh.spinOnce();	//在Arduino的loop函数，调用nh.spinOnce()，这样所有的ROS回调函数就会被处理
  delay(1);	//延时1ms
}
