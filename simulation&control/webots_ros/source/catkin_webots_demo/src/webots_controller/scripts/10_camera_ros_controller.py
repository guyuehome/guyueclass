#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
import tf
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from sensor_msgs.msg import Image

ImageData = None

def getImage(data):
    """ 传感器数据获取回调函数 """
    global ImageData
    ImageData = data

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('camera_sensor_controller_node', anonymous=True)

    # 使能传感器
    serviceName = "/camera_rbt/my_camera/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)

    # 订阅传感器数据话题
    Sub = rospy.Subscriber("/camera_rbt/my_camera/image",Image,getImage)
	   
    # 设置循环频率
    rate = rospy.Rate(3)

    # Control loop
    while not rospy.is_shutdown():		
        #print(ImageData)
        rate.sleep()


if __name__ == '__main__':
        webots_controller_function()
