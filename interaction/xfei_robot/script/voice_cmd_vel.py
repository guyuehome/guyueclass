#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)   #最开始等于0，回调cleanup
        self.speed = 0.2                  #初始化this->speed = 2.0
        self.msg = Twist()                #初始化消息内容this->msg = Twist()

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('cmd_vel', Twist, queue_size = 1)#初始化对象发布者，固定形式
        rospy.Subscriber('voiceWords', String, self.speechCb)#rospy类全局调用订阅函数，并进入回调

        r = rospy.Rate(10.0)              #这里的循环可以按照语音接收的频率来改变
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)   #假如没执行回调函数speechCb，Twist()为0
            r.sleep()
        
    def speechCb(self, msg):              #此处的msg是订阅的'voiceWords'的消息
        rospy.loginfo(msg.data)

        if msg.data.find("加速") > -1:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
        if msg.data.find("减速") > -1:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2

        if msg.data.find("前进") > -1:    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif msg.data.find("左转弯") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:        
                self.msg.angular.z = self.speed*2
        elif msg.data.find("右转弯") > -1:    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:        
                self.msg.angular.z = -self.speed*2
        elif msg.data.find("后退") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif msg.data.find("停止") > -1 or msg.data.find("停") > -1:          
            self.msg = Twist()
        
        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

