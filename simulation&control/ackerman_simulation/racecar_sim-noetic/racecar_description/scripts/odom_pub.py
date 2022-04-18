#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import math
import tf2_ros
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

# 定义部分参数
frame_name  = "odom"
child_frame_name = "base_footprint"
pub_name    = "odom_topic"
imu_name    = "imu_data"
joint_states_name   = "/racecar/joint_states"
# 定义起始坐标
start_x     = -0.5
start_y     = 0
start_z     = 0
class odom_class:
    odom_topic = Odometry()
    odom_tf = TransformStamped()
    # Yaw Angle 偏航角
    yaw_angle = 0
    old_yaw_angle = 0
    old_position = 0
    # 微分线速度
    #velocity = 0
    def __init__(self):
        # 订阅 IMU 传感器（陀螺仪）
        rospy.Subscriber(imu_name, Imu, self.callback_imu, queue_size=1)
        # 订阅 关节
        rospy.Subscriber(joint_states_name, JointState, self.callback_join, queue_size=1)
        # 发布 odom 坐标
        self.pub = rospy.Publisher(pub_name, Odometry, queue_size = 1)
        # 发布 tf odom 到 odom_footprint
        self.br = tf2_ros.TransformBroadcaster()
        # 定义时间差
        rate = rospy.Rate(20)
        #
        self.now_time_joint = rospy.Time.now().to_sec()
        self.old_time_joint = rospy.Time.now().to_sec()
        #
        self.odom_topic.pose.pose.position.x = start_x
        self.odom_topic.pose.pose.position.y = start_y
        self.odom_topic.pose.pose.position.z = start_z
        self.odom_topic.header.frame_id = frame_name
        self.odom_topic.child_frame_id = child_frame_name
        self.odom_tf.header.frame_id = self.odom_topic.header.frame_id
        self.odom_tf.child_frame_id = self.odom_topic.child_frame_id
        #

    def callback_imu(self, data):
        # 正交分解 偏航角（必须）
        #（在起始位置：以中线x轴为分界线，左正半y轴为正，右负半y轴为负，超过负半x轴直接跳变正负最大值）
        self.yaw_angle = euler_from_quaternion(
            (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
        # 四元数 xy轴角度（四元数，必须）
        self.odom_topic.pose.pose.orientation.x = data.orientation.x
        self.odom_topic.pose.pose.orientation.y = data.orientation.y
        self.odom_topic.pose.pose.orientation.z = data.orientation.z
        self.odom_topic.pose.pose.orientation.w = data.orientation.w
        # 正交分解 xy轴角速度
        self.odom_topic.twist.twist.angular.x = data.angular_velocity.x
        self.odom_topic.twist.twist.angular.y = data.angular_velocity.y
        self.odom_topic.twist.twist.angular.z = data.angular_velocity.z
        #self.pub.publish(self.odom_topic)

    def callback_join(self, data):
        # 计算时间差
        self.now_time_joint = rospy.Time.now().to_sec()
        add_time_joint = self.now_time_joint - self.old_time_joint
        self.old_time_joint = self.now_time_joint
        # 记录时间
        self.odom_topic.header.stamp = rospy.Time.now()
        self.odom_tf.header.stamp = rospy.Time.now()
        #try:
        # 编码器 测速
        lrw_topic = data.name.index('left_rear_axle')
        rrw_topic = data.name.index('right_rear_axle')
        # 平均速度
        velocity = (data.velocity[lrw_topic] + data.velocity[rrw_topic])/(2 * 13.95348)
        # 平均路程
        position = velocity * add_time_joint 
        #  xy轴坐标(位置，必须)
        self.odom_topic.pose.pose.position.x += (position*math.cos(self.yaw_angle[2]))
        self.odom_topic.pose.pose.position.y += (position*math.sin(self.yaw_angle[2]))
        self.odom_topic.pose.pose.position.z += 0
        # 正交分解 xy轴速度（线速度，必须）
        # 速度是相对小车的，往前是正的，往后是负的.
        if ((self.yaw_angle[2] < 1.52) and (self.yaw_angle[2] > -1.52)):
            self.odom_topic.twist.twist.linear.x = velocity*math.cos(self.yaw_angle[2])
        else :
            self.odom_topic.twist.twist.linear.x = -velocity*math.cos(self.yaw_angle[2])
    
        self.odom_topic.twist.twist.linear.y = velocity*math.sin(self.yaw_angle[2])
        self.odom_topic.twist.twist.linear.z = 0

        self.pub.publish(self.odom_topic)
        # tf 坐标关系 ：odom 到 base
        self.odom_tf.transform.translation.x = self.odom_topic.pose.pose.position.x
        self.odom_tf.transform.translation.y = self.odom_topic.pose.pose.position.y
        self.odom_tf.transform.translation.z = self.odom_topic.pose.pose.position.z
        # tf 四元数关系 ：odom 到 base
        self.odom_tf.transform.rotation.x = self.odom_topic.pose.pose.orientation.x
        self.odom_tf.transform.rotation.y = self.odom_topic.pose.pose.orientation.y
        self.odom_tf.transform.rotation.z = self.odom_topic.pose.pose.orientation.z
        self.odom_tf.transform.rotation.w = self.odom_topic.pose.pose.orientation.w
        self.br.sendTransform(self.odom_tf)

        
def main():
    rospy.init_node("odom_imu")
    node = odom_class()
    rospy.spin()

if __name__ == '__main__':
    main()
