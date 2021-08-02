#!/usr/bin/env python3
#coding:utf-8

import rospy
from sensor_msgs.msg import JointState

def callback(data):
    print("Joint1 Angle "+str(data.position[0]))
    print("Joint2 Angle "+str(data.position[1]))
    print("Joint3 Angle "+str(data.position[2]))
    print("Joint4 Angle "+str(data.position[3]))
    print("Joint5 Angle "+str(data.position[4]))
    print("Joint6 Angle "+str(data.position[5]))
    print("Joint7 Angle "+str(data.position[6]))
    print("Joint8 Angle "+str(data.position[7]))
    print("Joint9 Angle "+str(data.position[8]))
    print("Joint10 Angle "+str(data.position[9]))
    print("Joint11 Angle "+str(data.position[10]))
    print("Joint12 Angle "+str(data.position[11]))
    print("Joint13 Angle "+str(data.position[12]))
    print("Joint14 Angle "+str(data.position[13]))
    
    
def listener():
    rospy.init_node('Joint_try',anonymous=True)
    rospy.Subscriber('joint_states',JointState,callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
