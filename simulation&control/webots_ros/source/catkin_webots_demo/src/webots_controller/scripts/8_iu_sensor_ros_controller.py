#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
import tf
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from sensor_msgs.msg import Imu

rpyValue = []

def getRpyValue(data):
    """ 传感器数据获取回调函数 """
    global rpyValue
    x_ = data.orientation.x
   	y_ = data.orientation.y
   	z_ = data.orientation.z
  	w_ = data.orientation.w
	
    q_ = [x_,y_,z_,w_]
    rpyValue = tf.transformations.euler_from_quaternion(q_)

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('rpy_sensor_controller_node', anonymous=True)

    # 使能传感器
    serviceName = "/rpy_rbt/my_iu/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)

    # 订阅传感器数据话题
    Sub = rospy.Subscriber("/rpy_rbt/my_iu/roll_pitch_yaw",Imu,getRpyValue)
	   
    # 设置循环频率
    rate = rospy.Rate(10)

    # Control loop
    while not rospy.is_shutdown():		
        print("机器人当前姿态：" + str(rpyValue))
        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()