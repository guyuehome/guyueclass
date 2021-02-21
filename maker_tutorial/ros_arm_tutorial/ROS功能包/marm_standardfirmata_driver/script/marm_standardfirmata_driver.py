#!/usr/bin/env python
#coding:utf-8

import rospy
from sensor_msgs.msg import JointState

from pyfirmata import Arduino,util
'''
global joint1 = 3
global joint2 = 5
global joint3 = 6
global joint4 = 9
global joint5 = 10
global joint6 = 11
'''
arm = Arduino('/dev/ttyUSB0',baudrate=115200)

def callback(data):
	joint1_angle = data.position[0]*360/6.28+90
	if joint1_angle<0:
		joint1_angle = 0
	elif joint1_angle>180:
		joint1_angle = 180
	rospy.loginfo(rospy.get_caller_id() + ']--->Joint1 Angle :%d', joint1_angle)
	arm.servo_config(3,0,255,joint1_angle)
	
	joint2_angle = data.position[1]*360/6.28+90
	if joint2_angle<0:
		joint2_angle = 0
	elif joint2_angle>180:
		joint2_angle = 180
	rospy.loginfo(rospy.get_caller_id() + ']--->Joint2 Angle :%d', joint2_angle)
	arm.servo_config(5,0,255,joint2_angle)

	joint3_angle = data.position[2]*360/6.28+90
	if joint3_angle<0:
		joint3_angle = 0
	elif joint3_angle>180:
		joint3_angle = 180
	rospy.loginfo(rospy.get_caller_id() + ']--->Joint3 Angle :%d', joint3_angle)
	arm.servo_config(6,0,255,joint3_angle)

	joint4_angle = data.position[4]*360/6.28+90
	if joint4_angle<0:
		joint4_angle = 0
	elif joint4_angle>180:
		joint4_angle = 180
	rospy.loginfo(rospy.get_caller_id() + ']--->Joint4 Angle :%d', joint4_angle)
	arm.servo_config(9,0,255,joint4_angle)

	joint5_angle = data.position[5]*360/6.28+90
	if joint5_angle<0:
		joint5_angle = 0
	elif joint5_angle>180:
		joint5_angle = 180
	rospy.loginfo(rospy.get_caller_id() + ']--->Joint5 Angle :%d', joint5_angle)
	arm.servo_config(10,0,255,joint5_angle)
	
	joint6_angle = data.position[6]*360/6.28+90
	if joint6_angle<0:
		joint6_angle = 0
	elif joint6_angle>50:
		joint6_angle = 50
	rospy.loginfo(rospy.get_caller_id() + ']--->Joint6 Angle :%d', joint6_angle)
	arm.servo_config(11,0,255,joint6_angle)

def driver():
	rospy.init_node('Arm_Driver', anonymous=True)
	rospy.Subscriber('joint_states', JointState, callback)
	rospy.spin()

if __name__ == '__main__':
	driver()
arm.exit()
