#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from sensor_msgs.msg import Range


def getDistanceSensorValue(data):
    """ 传感器数据获取回调函数 """
    global DistanceValue
    DistanceValue = data.range

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('distance_sensor_controller_node', anonymous=True)

    # 使能传感器
    serviceName = "/distance_test_rbt/my_dist/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)

    # 订阅传感器数据话题
    Sub = rospy.Subscriber("/distance_test_rbt/my_dist/value",Range,getDistanceSensorValue)
	   
    # 设置循环频率
    rate = rospy.Rate(10)

    # Control loop
    while not rospy.is_shutdown():		
        print(DistanceValue)
        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()
