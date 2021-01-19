#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int

TIME_STEP = 32
MAX_SPEED = 6.0

def webots_controller_function():
    # 初始化节点
    rospy.init_node('rotational_motor_controller_node', anonymous=True)

	# 设置速度模式
	serviceName =  "/rot_1/my_Rmotor/set_position"
	setPosClient = rospy.ServiceProxy(serviceName,set_float)
	resp = setPosClient.call(float("inf"))
    if resp.success:
        rospy.loginfo("位置模式设置成功" )
    else:
        rospy.logwarn("调用set_position服务失败")
		
    # 设置初始速度
    serviceName = "/rot_1/my_Rmotor/set_velocity"
    setVelClient = rospy.ServiceProxy(serviceName,set_float)
    setVelClient.call(0)
               
    # 设置循环频率
    rate = rospy.Rate(1000)

    # Control loop
    while not rospy.is_shutdown():
        resp = setVelClient.call(MAX_SPEED)
        if resp.success:
            print("OK!")
   
        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()
