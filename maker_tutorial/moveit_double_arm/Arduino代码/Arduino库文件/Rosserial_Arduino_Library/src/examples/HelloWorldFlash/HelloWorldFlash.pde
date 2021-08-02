/*
 * rosserial Publisher Example
 * 
 * Demonstrates the usage of FLASH memory for ros topics and log messages
 * Note that the readout buffer for flash memory is currently set to 150 byte, 
 *  i. e. very long topic names or log msgs may cause it to overflow
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;


// Example a "normal publisher" still works if flash memory is allow by the generated ros_lib
// in this case only msg's type and md5sum will be stored in flash memory
ros::Publisher chatter1( "chatter1", &str_msg);


// Example of a publish of which the topic is stored in the flash memory and only 
// read out during topic negotiation, i. e. does not block SRAM the entire app life-time

// Declare topic name in PROGMEM
const char chatter_topic[]  PROGMEM  = { "chatter2" };

// set up the publisher with a flash topic
// the FCAST marco assures casting the chatter_topic to __FlashStringHelper *
// thereby assuring that the topic will not be accessed directly but read out 
// of flash memory before usage
ros::Publisher chatter2( FCAST( chatter_topic), &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  
  // no difference in advertising of chatter1 ('normal' SRAM topic) or chatter2 (PROGMEM topic)
  nh.advertise(chatter1);
  nh.advertise(chatter2);
}

void loop()
{
  
  str_msg.data = hello;
  
  // no difference in publishing on chatter1 ('normal' SRAM topic) or chatter2 (PROGMEM topic)
  chatter1.publish( &str_msg );
  chatter2.publish( &str_msg );

  // example logging
  // normal logging still works as before; message text is stored in SRAM and published directly 
  nh.loginfo("Hello world");

  // logging a PROGMEM string 
  // allows use of the the Arduino F marco (similar to Serial.print( F( "foo" ) ) in pure Arduino code)
  // string is stored in flash memory and read out when logging
  // reduces SRAM usage
  // WARNING: string is read out !!each time!! when logging; this might 
  //          be unsuitable when logging the string at high frequency  
  nh.loginfo( F("Hello world from flash") );
  
  nh.spinOnce();
  delay(1000);
}
