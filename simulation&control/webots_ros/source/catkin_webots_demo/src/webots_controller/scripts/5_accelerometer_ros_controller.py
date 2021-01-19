#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from sensor_msgs.msg import Imu

AccelermeterValue = 0.0

def getAccelermeterSensorValue(data):
    """ 传感器数据获取回调函数 """
    global AccelermeterValue
    AccelermeterValue = data.linear_acceleration.y - 9.81

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('accel_sensor_controller_node', anonymous=True)

    # 使能传感器
    serviceName = "/accel_test_rbt/my_accelerometer/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)

    # 订阅传感器数据话题
    Sub = rospy.Subscriber("/accel_test_rbt/my_accelerometer/values",Imu,getAccelermeterSensorValue)

    # 设置循环频率
    rate = rospy.Rate(10)

    # Control loop
    while not rospy.is_shutdown():
        print(AccelermeterValue)
        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()
