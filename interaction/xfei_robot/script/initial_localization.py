#!/usr/bin/env python
import sys, rospy, tf, actionlib
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
  rospy.init_node('initial_localization')
  pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
  p = PoseWithCovarianceStamped()
  p.header.frame_id = "map"
  p.pose.pose.position.x = -2
  p.pose.pose.position.y = -0.5
  p.pose.pose.position.z = 0
  p.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
  p.pose.covariance = \
    [ 0.1 , 0,    0, 0, 0, 0,
      0   , 0.1 , 0, 0, 0, 0,
      0   , 0   , 0, 0, 0, 0,
      0   , 0   , 0, 0, 0, 0,
      0   , 0   , 0, 0, 0, 0,
      0   , 0   , 0, 0, 0, 0.1 ]
  for t in range(0,5):
    rospy.sleep(1)
    pub.publish(p)

