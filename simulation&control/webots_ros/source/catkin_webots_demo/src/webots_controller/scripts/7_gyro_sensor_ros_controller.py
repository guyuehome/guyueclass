#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from sensor_msgs.msg import Imu

gyroValue = []

def getGyroSensorValue(data):
    """ 传感器数据获取回调函数 """
    global gyroValue
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    gyroValue = [w_x,w_y,w_z]

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('gyro_sensor_controller_node', anonymous=True)

    # 电机启动, 1rad/s旋转
    serviceName =  "/gyro_rbt/my_Rmotor/set_position"
    setPosClient = rospy.ServiceProxy(serviceName,set_float)
    resp = setPosClient.call(float("inf"))
    serviceName = "/gyro_rbt/my_Rmotor/set_velocity"
    setVelClient = rospy.ServiceProxy(serviceName,set_float)
    resp = setVelClient.call(1)

    # 使能传感器
    serviceName = "/gyro_rbt/my_gyro/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)
    # 订阅传感器数据话题
    Sub = rospy.Subscriber("/gyro_rbt/my_gyro/values",Imu,getGyroSensorValue)
	   
    # 设置循环频率
    rate = rospy.Rate(10)

    # Control loop
    while not rospy.is_shutdown():		
        print(gyroValue)
        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()
