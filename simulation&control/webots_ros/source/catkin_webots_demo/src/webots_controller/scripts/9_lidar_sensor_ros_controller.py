#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
# Author:罗伯特祥
import rospy
import math
import tf
from std_msgs.msg import String
from webots_ros.srv import set_float,set_int,set_bool
from sensor_msgs.msg import PointCloud,LaserScan,Image

	

def getLaserScanValue(data):
    """ 传感器数据获取回调函数 """
    LaserScanMsgs = data
    LaserScanMsgs.header.frame_id = "laserScan_correction_link"
    #print(LaserScanMsgs)
    # 发布修正坐标系标号的激光扫描数据
    pub = rospy.Publisher("/lidar_rbt/my_lidar/laser_scan_layer0_correction",LaserScan,queue_size=10)
    pub.publish(LaserScanMsgs) 
	
    br2 = tf.TransformBroadcaster()
    br2.sendTransform((0,0,1),
            tf.transformations.quaternion_from_euler(3.1416,0,1.5707),
            rospy.Time.now(),"laserScan_correction_link","world")

def webots_controller_function():
    """ webots动作控制器 """
    # 初始化节点
    rospy.init_node('lidar_sensor_controller_node', anonymous=True)

    # 使能传感器
    serviceName = "/lidar_rbt/my_lidar/enable"
    setEnableClient = rospy.ServiceProxy(serviceName,set_int)
    resp = setEnableClient.call(1)
    serviceName = "/lidar_rbt/my_lidar/enable_point_cloud"
    setEnableClient = rospy.ServiceProxy(serviceName,set_bool)
    resp = setEnableClient.call(True)

    # 订阅传感器数据话题
    sub = rospy.Subscriber("/lidar_rbt/my_lidar/laser_scan/layer0",LaserScan,getLaserScanValue)

    # 设置循环频率
    rate = rospy.Rate(100)

    # Control loop
    while not rospy.is_shutdown():		
        br1 = tf.TransformBroadcaster()
        br1.sendTransform((0,0,0.5),
                tf.transformations.quaternion_from_euler(1.5707,0,0),
                rospy.Time.now(),"lidar_rbt/my_lidar","world")
           
        rate.sleep()

if __name__ == '__main__':
        webots_controller_function()