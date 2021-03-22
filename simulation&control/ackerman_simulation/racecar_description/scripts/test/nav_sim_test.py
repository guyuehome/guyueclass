#!/usr/bin/env python
import rospy
import std_msgs.msg
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

import time 
import threading
pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDrive,queue_size=1)

def thread_job():
    rospy.spin()

def callback(data):
    speed = data.linear.x 
    turn = data.angular.z

    msg = AckermannDrive()


    msg.speed = speed
    msg.acceleration = 1
    msg.jerk = 1
    msg.steering_angle = turn
    msg.steering_angle_velocity = 1

    pub.publish(msg)

def SubscribeAndPublish():
    rospy.init_node('nav_sim', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    #rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    rospy.spin()


if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass


########################
