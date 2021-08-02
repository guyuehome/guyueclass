#!/usr/bin/env python3
#coding:utf-8

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import serial

ser = serial.Serial("/dev/ttyUSB0",9600)
joint_angle = [0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00]

def talker():

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher_teach')
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
    
        joint_str = JointState()
        joint_str.header = Header()
        joint_str.header.stamp = rospy.Time.now()
        joint_str.name=['joint1','joint2','joint3','joint4','joint5','joint6','joint7','joint8','joint9','joint10','joint11','joint12','joint13','joint14']
        
        msg = ser.readline().decode('utf-8')
        
        if msg[:7] == 'Joint1 ':
            joint_angle[0] = float(msg[-7:-2])
        elif msg[:7] == 'Joint2 ':
            joint_angle[1] = float(msg[-7:-2])
        elif msg[:7] == 'Joint3 ':
            joint_angle[2] = float(msg[-7:-2])
        elif msg[:7] == 'Joint4 ':
            joint_angle[3] = float(msg[-7:-2])
        elif msg[:7] == 'Joint5 ':
            joint_angle[4] = float(msg[-7:-2])
        elif msg[:7] == 'Joint6 ':
            joint_angle[5] = -float(msg[-7:-2])
            joint_angle[6] = float(msg[-7:-2])
            
        elif msg[:7] == 'Joint7 ':
            joint_angle[7] = float(msg[-7:-2])
        elif msg[:7] == 'Joint8 ':
            joint_angle[8] = float(msg[-7:-2])
        elif msg[:7] == 'Joint9 ':
            joint_angle[9] = float(msg[-7:-2])
        elif msg[:7] == 'Joint10':
            joint_angle[10] = float(msg[-7:-2])
        elif msg[:7] == 'Joint11':
            joint_angle[11] = float(msg[-7:-2])
        elif msg[:7] == 'Joint12':
            joint_angle[12] = -float(msg[-7:-2])
            joint_angle[13] = float(msg[-7:-2])
        
        joint_str.position=joint_angle;
        joint_str.velocity = []
        joint_str.effort = []
        pub.publish(joint_str)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        ser.close()
        #pass
