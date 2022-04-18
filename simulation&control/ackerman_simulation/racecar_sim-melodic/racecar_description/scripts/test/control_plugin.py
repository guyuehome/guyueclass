#!/usr/bin/env python

import rospy
import math
import sys
import tf2_ros

from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

# vehicle name

car_name = str(sys.argv[1])

# subscriber topics

gazebo_odom_topic = '/{}/ground_truth'.format(car_name)
command_topic     = '/{}/command'.format(car_name)

# publisher topics

odom_pub_topic  = '/{}/base/odom'.format(car_name)
footprint_topic = '/{}/base/footprint'.format(car_name)

# control topics

LRW_topic   = '/{}/left_rear_wheel_velocity_controller/command'.format(car_name)
RRW_topic   = '/{}/right_rear_wheel_velocity_controller/command'.format(car_name)
LFW_topic   = '/{}/left_front_wheel_velocity_controller/command'.format(car_name)
RFW_topic   = '/{}/right_front_wheel_velocity_controller/command'.format(car_name)
LSH_topic   = '/{}/left_steering_hinge_position_controller/command'.format(car_name)
RSH_topic   = '/{}/right_steering_hinge_position_controller/command'.format(car_name)

# frame names

odom_frame = 'odom'
base_frame = '{}_base_link'.format(car_name)

# information publishers

footprint_pub = rospy.Publisher(footprint_topic, PolygonStamped, queue_size = 1)
odom_pub      = rospy.Publisher(odom_pub_topic, Odometry, queue_size = 1)
tf_pub        = tf2_ros.TransformBroadcaster()

# control publishers

pub_vel_LRW = rospy.Publisher(LRW_topic, Float64, queue_size = 1)
pub_vel_RRW = rospy.Publisher(RRW_topic, Float64, queue_size = 1)
pub_vel_LFW = rospy.Publisher(LFW_topic, Float64, queue_size = 1)
pub_vel_RFW = rospy.Publisher(RFW_topic, Float64, queue_size = 1)
pub_pos_LSH = rospy.Publisher(LSH_topic, Float64, queue_size = 1)
pub_pos_RSH = rospy.Publisher(RSH_topic, Float64, queue_size = 1)

# footprint parameters

global seq

seq = 0
footprint = PolygonStamped()

side_A = Point32()
side_B = Point32()
side_C = Point32()
side_D = Point32()
side_E = Point32()

[side_A.x, side_A.y, side_A.z] = [-0.1, -0.2,  0.0]
[side_B.x, side_B.y, side_B.z] = [ 0.5, -0.2,  0.0]
[side_C.x, side_C.y, side_C.z] = [ 0.6,  0.0,  0.0]
[side_D.x, side_D.y, side_D.z] = [ 0.5,  0.2,  0.0]
[side_E.x, side_E.y, side_E.z] = [-0.1,  0.2,  0.0]

footprint.header.frame_id = base_frame
footprint.polygon.points  = [side_A, side_B, side_C, side_D, side_E]

# footprint visualizer

def footprint_visualizer():

    global seq

    footprint.header.seq = seq
    seq = seq + 1
    footprint.header.stamp = rospy.Time.now()
    footprint_pub.publish(footprint)

def odom_callback(data):

    odom = Odometry()
    odom.header.frame_id      = odom_frame
    odom.child_frame_id       = base_frame
    odom.header.stamp         = rospy.Time.now()
    odom.pose   = data.pose
    odom.pose.pose.position.x = odom.pose.pose.position.x 
    odom.pose.pose.position.y = odom.pose.pose.position.y 
    odom.twist = data.twist

    tf = TransformStamped(header         = Header(
                          frame_id       = odom.header.frame_id,
                          stamp          = odom.header.stamp),
                          child_frame_id = odom.child_frame_id,
                          transform      = Transform(
                          translation    = odom.pose.pose.position,
                          rotation       = odom.pose.pose.orientation))

    # visualize footprint everytime odom changes

    footprint_visualizer()

    odom_pub.publish(odom)
    tf_pub.sendTransform(tf)

# command variables

global previous_speed

previous_speed   = 0.0
min_speed        = 0.0
max_speed        = 15.0  # 100.0
speed_delta      = 1.0 # 1.25
previous_speed   = 0.0

# command callback

def command_callback(data):

    global previous_speed

    steering_angle = Float64()
    speed          = Float64()

    steering_angle.data = data.steering_angle
    speed.data          = data.speed * max_speed

    acceleration_factor = speed_delta

    if not speed.data == 0.0:
        speed_dir = speed.data/abs(speed.data)
        speed.data = abs(speed.data)
    else:
        if previous_speed < 0.0:
            speed_dir = -1.0
        else:
            speed_dir = 1.0

    previous_speed = abs(previous_speed)

    if speed.data >= previous_speed + acceleration_factor:
        speed.data = previous_speed + acceleration_factor
    elif speed.data <= previous_speed - acceleration_factor:
        speed.data = previous_speed - acceleration_factor

    if speed.data > max_speed:
        speed.data = max_speed
    elif speed.data < min_speed:
        speed.data = min_speed

    speed.data     = speed_dir * speed.data
    previous_speed = speed.data

    pub_vel_LRW.publish(speed)
    pub_vel_RRW.publish(speed)
    pub_vel_LFW.publish(speed)
    pub_vel_RFW.publish(speed)

    pub_pos_LSH.publish(steering_angle)
    pub_pos_RSH.publish(steering_angle)

if __name__ == '__main__':

    try:

        rospy.init_node('control_plugin', anonymous = True)

        rospy.Subscriber(gazebo_odom_topic, Odometry, odom_callback)
        # rospy.Subscriber(command_topic, AckermannDrive, command_callback)
        rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDrive, command_callback)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
