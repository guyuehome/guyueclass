//#include <Wire.h>  //引用IIC协议库
//#include <Adafruit_GFX.h> //引用字体取模库
//#include <Adafruit_SH1106.h>  //引用SH1106驱动库，1.3寸OLED驱动时SH1106

//Adafruit_SH1106 display(4);

int pinA = 2; //接中断信号的脚,旋转编码器A
int pinB = 3; //接中断信号的脚,旋转编码器B
int leftMode = 10;  //左臂使能
int rightMode = 11; //右臂使能

int key[6] = {4,5,6,7,8,9};  //引脚定义
float angleValue[12] = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00}; //关节弧度定义
String joint[12] = {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6","Joint7","Joint8","Joint9","Joint10","Joint11","Joint12"}; //关节名定义

//加信号触发
void onFallingA()
{
  if(digitalRead(pinB) == LOW) //检测B是否为低电平
  {
    if(!digitalRead(leftMode) && digitalRead(rightMode))
    {
      int keyValue = keySwitch(); //获取当前使能按键
      if(keyValue != -1)  //-1表示数据无效
      {
        angleValue[keyValue]+=0.01; //每次加0.01
  
        if(angleValue[keyValue]>1.57)
          angleValue[keyValue] = 1.57;

        //串口输出
        Serial.print(joint[keyValue]);  
        Serial.print(" value is : ");
        Serial.println(angleValue[keyValue]);
      }
    }else if(digitalRead(leftMode) && !digitalRead(rightMode))
    {
      int keyValue = keySwitch(); //获取当前使能按键
      if(keyValue != -1)  //-1表示数据无效
      {
        angleValue[keyValue+6]+=0.01; //每次加0.01
  
        if(angleValue[keyValue+6]>1.57)
          angleValue[keyValue+6] = 1.57;
          
        //串口输出
        Serial.print(joint[keyValue+6]);  
        Serial.print(" value is : ");
        Serial.println(angleValue[keyValue+6]);
      }
    }
  }
}

//减信号触发
void onFallingB()
{
   if(digitalRead(pinA) == LOW) //检测A是否为低电平
   {
      if(!digitalRead(leftMode) && digitalRead(rightMode))
    {
      int keyValue = keySwitch(); //获取当前使能按键
      if(keyValue != -1)  //-1表示数据无效
      {
        angleValue[keyValue]-=0.01; //每次减0.01
  
        if(angleValue[keyValue]<-1.57)
          angleValue[keyValue] = -1.57;

        //串口输出
        Serial.print(joint[keyValue]);  
        Serial.print(" value is : ");
        Serial.println(angleValue[keyValue]);
      }
    }else if(digitalRead(leftMode) && !digitalRead(rightMode))
    {
      int keyValue = keySwitch(); //获取当前使能按键
      if(keyValue != -1)  //-1表示数据无效
      {
        angleValue[keyValue+6]-=0.01; //每次减0.01
  
        if(angleValue[keyValue+6]<-1.57)
          angleValue[keyValue+6] = -1.57;

        //串口输出
        Serial.print(joint[keyValue+6]);  
        Serial.print(" value is : ");
        Serial.println(angleValue[keyValue+6]);
      }
    }
  }
}

//按键使能
int keySwitch()
{
    for(int i=0;i<6;i++)  //轮询检测
    {
      if(!digitalRead(key[i]))  //如果对应按键为低电平
      {
        return i; //返回对应的按键值
      }  
    }
    //轮询结束没有低电平，返回-1
    return -1;
}

/*
void showLogo()
{
  display.clearDisplay();  //清空当前屏幕内容
  display.setTextSize(1); //设置显示字体的大小
  display.setTextColor(WHITE);  //设置显示字体的样式
  display.setCursor(20,20); //设置显示字体的坐标
  display.println("Hello, ROS!"); //设置显示字体的内容
  display.setTextColor(WHITE);  //设置显示字体的样式
  display.setCursor(20,28); //设置显示字体的坐标
  display.println("Hello, Future!"); //设置显示字体的内容
  display.display();  //显示
  delay(2000);  //延时2000ms
  display.clearDisplay(); //清空当前屏幕内容
}
*/
 
void setup()
{
  Serial.begin(9600); //串口初始化

  //旋转编码器及按键初始化
  pinMode(pinA,INPUT);
  pinMode(pinB,INPUT);

  for(int i=0;i<6;i++)
  {
    pinMode(key[i],INPUT); 
  }

  //注册中断及，中断触发方式为下降沿
  attachInterrupt( digitalPinToInterrupt(pinA), onFallingA, FALLING);
  attachInterrupt( digitalPinToInterrupt(pinB), onFallingB, FALLING);

  pinMode(leftMode,INPUT);
  pinMode(rightMode,INPUT);

  
  //display.begin(SH1106_SWITCHCAPVCC, 0x3C); //初始化OLED屏幕，IIC默认地址0x3C
  
  //display.display();  //显示
  //delay(2000);  //延时2000ms

  //showLogo();
  
}
 
void loop()
{
  delay(10);
}
