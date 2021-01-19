#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int

def webots_controller_function():
    # 初始化节点
    rospy.init_node('linear_motor_controller_node', anonymous=True)
 
    # 位置模式
    serviceName =  "/linear_rbt/my_lMotor/set_position"
    setPosClient = rospy.ServiceProxy(serviceName,set_float)
    resp = setPosClient.call(0)
    if resp.success:
        rospy.loginfo("位置模式设置成功" )
    else:
        rospy.logwarn("调用set_position服务失败")
               
    rate = rospy.Rate(1000)

    # Control loop
    stepCount = 0
    while not rospy.is_shutdown():
        resp = setPosClient.call(math.sin(stepCount)*0.5)
        if resp.success:
            print("Step:%s" %stepCount)
                
        stepCount += 0.1
        rate.sleep()
		
if __name__ == '__main__':
        webots_controller_function()
