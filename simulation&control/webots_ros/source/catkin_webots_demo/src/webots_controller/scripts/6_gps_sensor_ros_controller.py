#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
import tf
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from sensor_msgs.msg import NavSatFix

GpsValue = []

def getGpsSensorValue(data):
    """ 传感器数据获取回调函数 """
    global GpsValue
    GpsValue = [data.latitude,data.longitude,data.altitude] # webots中：北x 东z 上y
	
    br = tf.TransformBroadcaster()	
    br.sendTransform((-GpsValue[0],GpsValue[1],GpsValue[2]),  # rqt中：南x 东y 上z
            tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),"gps","world")

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('gps_sensor_controller_node', anonymous=True)

    # 使能传感器
    serviceName = "/gps_rbt/my_gps/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)
	
    # 订阅传感器数据话题
    Sub = rospy.Subscriber("/gps_rbt/my_gps/values",NavSatFix,getGpsSensorValue)
	   
    # 设置循环频率
    rate = rospy.Rate(10)

    # Control loop
    while not rospy.is_shutdown():		
        print("机器人在webots中的位置：" + str(GpsValue))
        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()
