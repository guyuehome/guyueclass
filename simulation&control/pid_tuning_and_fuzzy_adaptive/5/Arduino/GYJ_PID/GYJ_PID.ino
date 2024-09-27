/*
* 日期: 2022/03/28
* 作者: DandD(董昊天)
* 描述: 
*/
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <MsTimer2.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

//ROS Messages
ros::NodeHandle nh;
std_msgs::Float32 ne_msg;
ros::Publisher motorne("motorne", &ne_msg);
std_msgs::Int16 ctrl_msg;
ros::Publisher ctrl("ctrl", &ctrl_msg);

void setup()
{
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pwm.begin();
  pwm.setPWMFreq(50);
  nh.initNode();
  nh.advertise(motorne);
  nh.advertise(ctrl);
  Serial.begin(57600);
  attachInterrupt(0,CountA, FALLING);// 检测脉冲下降沿中断，并转到CountA函数
  MsTimer2::set(100,flash);          // 中断设置函数，0.1s
  MsTimer2::start();                 // 开始计时
}
 
void loop()
{
  pot = analogRead(A0);
  PWMout = map(pot,0,1023,0,100);
  analogWrite(ENA,PWMout);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  ctrl_msg.data = PWMout;
  ctrl.publish( &ctrl_msg );
  nh.spinOnce();
  delay(100);
  }
 
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
/*
  pot = analogRead(A0);
  PWMout = map(pot,0,1023,0,255);
  analogWrite(ENA,PWMout);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  ctrl_msg.data = PWMout;
  ctrl.publish( &ctrl_msg );
*/  
  int w = ppsA;
  velocity = c*v(w);
  ne_msg.data = velocity;
  motorne.publish( &ne_msg );
  nh.spinOnce();
  ppsA = 0;
  
  //delay(2);
  //Serial.println(velocity);
  
}

float v(float n)
{
  float vel = n/1.5; // n/(15*0.1)
  return vel;
}
