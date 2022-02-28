#!/usr/bin/env python3

import rospy
from nav_msgs.msg  import Odometry

INTRODUCTION = """
---------------------------
订阅机器人的实时位姿并打印到终端
---------------------------
"""
# 打印功能信息
print(INTRODUCTION)


# 回调函数处理消息
def limoCallBack(msg):
    rospy.loginfo("limo pose: x:%06f, y:%0.6f, z:%0.6f",\
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.z)

# 初始化 ros 节点
rospy.init_node('limo_subscriber',anonymous = False)

# 初始化机器人位姿的订阅者
rospy.Subscriber("/odom",\
    Odometry, limoCallBack, queue_size=1)
    
# 循环等待信息数据
rospy.spin()
