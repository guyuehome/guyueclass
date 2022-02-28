#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

INTRODUCTION = """
---------------------------
发布控制命令使小车以圆形轨迹运动
---------------------------
"""
# 打印功能信息
print(INTRODUCTION)


# 初始化 ros 节点
rospy.init_node("draw_circle", anonymous=True)

# 初始化控制命令发布者
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# 初始化 Twist 控制消息
twist = Twist()
twist.linear.x = 0.3
twist.angular.z = 1.5

# 初始化 ros主循环
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # 发布控制命令
    cmd_vel_pub.publish(twist)
    rate.sleep()