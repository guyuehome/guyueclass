/*
* 日期: 2022/03/28
* 作者: DandD(董昊天)
* 描述: 
*/
//////////////////////////////////////////////////////////////
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include <MsTimer2.h>

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//////////////////////////////////////////////////////////////
//L298N
#define IN1 10
#define IN2 9
#define ENA 6
int PWMout;

//Encoder
#define pinA 2
#define pinB 3
#define pos 1
#define neg -1
int ppsA=0;
int c;
float velocity = 0;

//Potentiometer
int pot = 0;

//////////////////////////////////////////////////////////////
//ROS Messages
ros::NodeHandle nh;
std_msgs::Float32 ne_msg;
ros::Publisher motorne("motorne", &ne_msg);

void messageCb( const std_msgs::Int16& ctrl){
  PWMout = ctrl.data;
  analogWrite(ENA,PWMout);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  }
ros::Subscriber<std_msgs::Int16> sub("ctrl", messageCb );

//////////////////////////////////////////////////////////////
void setup()
{
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  pwm.begin();
  pwm.setPWMFreq(50);
  
  nh.initNode();
  nh.advertise(motorne);
  nh.subscribe(sub);
  
  Serial.begin(57600);
  
  attachInterrupt(0,CountA, FALLING);// 检测脉冲下降沿中断，并转到CountA函数
  MsTimer2::set(100,flash);          // 中断设置函数，0.1s
  MsTimer2::start();                 // 开始计时
}

void loop()
{}

void CountA() 
{
  if(digitalRead(pinB) == HIGH)
  {c = pos;}
  if(digitalRead(pinB) == LOW)
  {c = neg;}
  ppsA++;
}

void flash() 
{ 
  int w = ppsA;
  velocity = c*v(w);
  ne_msg.data = velocity;
  motorne.publish( &ne_msg );
  nh.spinOnce();
  ppsA = 0;
}

float v(float n)
{
  float vel = n/1.5;
  return vel;
}
