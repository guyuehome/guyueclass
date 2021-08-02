/*
 * rosserial Publisher Example
 * 
 * Publishes free SRAM with use of FLASH memory for TOPIC/LOG 
 */

/// memory free function; copied from: https://github.com/maniacbug/MemoryFree/blob/master/MemoryFree.cpp

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

int freeMemory() 
{
  int free_memory;
  
  if((int)__brkval == 0)
  {
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }
  else
  {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  
  return free_memory;
}


#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
std_msgs::UInt32 int_msg;

const char chatter_topic[]  PROGMEM  = { "/chatter/hello_world" };
const char freemem_topic[]  PROGMEM  = { "/chatter/free_memory" };

ros::Publisher chatter_str( FCAST( chatter_topic ), &str_msg);
ros::Publisher chatter_int( FCAST( freemem_topic ), &int_msg);


char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  
  nh.advertise(chatter_str);
  nh.advertise(chatter_int);
}

void loop()
{
  str_msg.data = hello;
  
  int_msg.data = freeMemory();
  
  chatter_str.publish( &str_msg );
  
  chatter_int.publish( &int_msg );

  nh.loginfo( F("Published free memory and string message via rosserial") );
  
  nh.spinOnce();
  
  delay(1000);
}
