#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from webots_ros.msg import Float64Stamped

def getPositionSensorValue(data):
    """ 位置传感器数据获取回调函数 """
    global positionValue
    positionValue = data.data

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('position_sensor_controller_node', anonymous=True)

    # 使能传感器
    serviceName = "/position_test_rbt/my_positionSensor/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)
	
    # 订阅传感器数据话题
    Sub = rospy.Subscriber("/position_test_rbt/my_positionSensor/value",Float64Stamped,getPositionSensorValue)
	
    # 设置速度模式
    serviceName =  "/position_test_rbt/my_Rmotor/set_position"
    setPosClient = rospy.ServiceProxy(serviceName,set_float)
    resp = setPosClient.call(float("inf"))
	
    # 设置初始速度
    serviceName = "/position_test_rbt/my_Rmotor/set_velocity"
    setVelClient = rospy.ServiceProxy(serviceName,set_float)
    resp = setVelClient.call(0)
               
    # 设置循环频率
    rate = rospy.Rate(1000)

    # Control loop
    stepCount = 0
    speed = 2.0
    while not rospy.is_shutdown():		
        if stepCount > 50 and stepCount <=100:
            speed = -2.0
        elif stepCount > 100:
            speed = 0.0
		
        setVelClient.call(speed)
		
        stepCount += 1
        print(positionValue)

        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()