#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2021 PS-Micro, Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

BEGIN_DISTANCE = 0.8
OVER_DISTANCE = 0.2


class MoveToTarget(object):
    def __init__(self):
        # ros 初始化
        rospy.init_node('move_to_target_ar', anonymous=True)

        # 发布控制机器人的消息
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # 创建一个Subscriber，订阅/object_detect_pose
        self.pose_sub = rospy.Subscriber("/ar_pose_marker",
                                         AlvarMarkers,
                                         self.poseCallback,
                                         queue_size=10)

        # 初始化状态标志
        print("Wait for Exploring ...")

    def poseCallback(self, msg):
        for marker in msg.markers:
            qr_x = marker.pose.pose.position.x
            qr_y = marker.pose.pose.position.y
            qr_z = marker.pose.pose.position.z
            rospy.loginfo("Target{} Pose: x:{}, y:{}, z:{}".format(
                marker.id, qr_x, qr_y, qr_z))

            #if marker.id is 0 and qr_z < BEGIN_DISTANCE:
            if marker.id is 0 :
                vel = Twist()
                vel.linear.x =  qr_z * 0.5  
                vel.angular.z = 1.0 * qr_y  
                self.vel_pub.publish(vel)
                rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))


if __name__ == '__main__':
    try:
        MoveToTarget()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploring logistics finished.")
