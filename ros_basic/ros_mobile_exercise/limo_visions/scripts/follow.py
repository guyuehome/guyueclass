#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
objPose = Pose()
objPose.position.x = 0
objPose.position.y = 0
objPose.position.z = 0

vel = Twist()
vel.linear.x = 0.0
vel.linear.y = 0.0
vel.linear.z = 0.0
vel.angular.x = 0.0
vel.angular.y = 0.0
vel.angular.z = 0.0


class follow_object:
    def __init__(self):    
        #订阅位姿信息

        self.Pose_sub = rospy.Subscriber("object_detect_pose", Pose, self.poseCallback)
        #发布速度指令
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def poseCallback(self,Pose):

        X = Pose.position.x;
        Y = Pose.position.y;
        Z = Pose.position.z;


        if Z >=14500 and Z <= 15500 :                            
            vel = Twist()
        elif Z < 14500 :
            vel = Twist()
            vel.linear.x = (1.0 - Z/14000) * 0.8
            vel = Twist()
            vel.linear.x = (1.0 - Z/15000) * 0.8
        else:
            print("No Z,cannot control!")
        self.vel_pub.publish(vel)
        rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))

        if X > 310 and X < 330 :
            vel = Twist()
        elif X < 310 :
            vel = Twist()
            vel.angular.z = (1.0 - X/320) 
        elif X > 330 :
            vel = Twist()
            vel.angular.z = (1.0 - X/320) 
        else:
            print("No X,cannot control!")
        self.vel_pub.publish(vel)
        rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("follow_object")
        rospy.loginfo("Starting follow object")
        follow_object()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down follow_object node."
        cv2.destroyAllWindows()
