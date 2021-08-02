/*
 * Test with many publishers and subscribers
 *
 * This example works on arduino uno if the ros_lib is created with use of PROGMEM,
 *   i. e. without any changed to the application code at least topic types and md5sum
 *   are stored in flash memory and not in SRAM for the entire application lifetime
 *
 * If the ros_lib is created without use of flash menory this will compile but all the 
 * 	msg's tyes and md5sums will use up all the SRAM on Arduino uno and this 
 * 	will fail to run on arduino uno
 * 
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
std_msgs::Int16 int16_msg;
std_msgs::Int32 int32_msg;
std_msgs::Float32 float32_msg;
std_msgs::Empty empty_msg;
std_msgs::Time time_msg;


ros::Publisher chatter_str("chatter_str", &str_msg);
ros::Publisher chatter_int16("chatter_int16", &int16_msg);
ros::Publisher chatter_int32("chatter_int32", &int32_msg);
ros::Publisher chatter_float32("chatter_float32", &float32_msg);
ros::Publisher chatter_empty("chatter_empty", &empty_msg);
ros::Publisher chatter_time("chatter_time", &time_msg);

char hello[13] = "hello world!";


void messageCbEmpty( const std_msgs::Empty& msg){
  nh.loginfo( "Received emtpy msg" );
}

void messageCbStr( const std_msgs::String& msg){
  nh.loginfo( "Received str msg" );
}

void messageCbUInt8( const std_msgs::UInt8& msg){
  nh.loginfo( "Received uint8 msg" );
}

void messageCbUInt16( const std_msgs::UInt16& msg){
  nh.loginfo( "Received uint16 msg" );
}

void messageCbUInt32( const std_msgs::UInt32& msg){
  nh.loginfo( "Received uint32 msg" );
}

ros::Subscriber<std_msgs::Empty> sub1("msg_emtpy", &messageCbEmpty );
ros::Subscriber<std_msgs::String> sub2("msg_string", &messageCbStr );
ros::Subscriber<std_msgs::UInt8> sub3("msg_uint8", &messageCbUInt8 );
ros::Subscriber<std_msgs::UInt16> sub4("msg_uint16", &messageCbUInt16 );
ros::Subscriber<std_msgs::UInt32> sub5("msg_uint32", &messageCbUInt32);

void setup()
{
  nh.initNode();
  nh.advertise(chatter_str);
  nh.advertise(chatter_int16);
  nh.advertise(chatter_int32);
  nh.advertise(chatter_float32);
  nh.advertise(chatter_empty);
  nh.advertise(chatter_time);
  
  nh.subscribe( sub1 );
  nh.subscribe( sub2 );
  nh.subscribe( sub3 );
  nh.subscribe( sub4 );
  nh.subscribe( sub5 );
}

void loop()
{
  {
    str_msg.data = hello;
    chatter_str.publish( &str_msg );
  }
  
  {
    int16_msg.data = 10;
    chatter_int16.publish( &int16_msg );
  }
  
  {
    int32_msg.data = 102;
    chatter_int32.publish( &int32_msg );
  }
  
  {
    float32_msg.data = 10;
    chatter_float32.publish( &float32_msg );
  }
  
  {
    chatter_empty.publish( &empty_msg );
  }
  
  {
    time_msg.data = nh.now();
    chatter_time.publish( &time_msg );
  }
  
  nh.loginfo( "Published a lot of messages" );
  
  nh.spinOnce();
  delay(1000);
}
