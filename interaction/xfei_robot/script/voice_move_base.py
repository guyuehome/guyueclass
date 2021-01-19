#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import actionlib
import roslib;
import rospy
from std_msgs.msg import String#添加了接受的消息类型
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

 
waypoints = [  #导航的坐标点
    [(1.522, 0.444, 0.0), (0.0, 0.0, -0.519, 0.85)],
    [(-2.0432, -0.439, 0.0), (0.0, 0.0, -0.559, 0.82902)]
]
 
 
def goal_pose(pose):  #坐标点转换成对应的消息类型
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
 
    return goal_pose

def callback(msg):
	rospy.loginfo(msg.data)#这里注意需要.data而不是msg本身，data里存的才是字符串
	if msg.data.find('一') > -1:
            pose = waypoints[0]
        elif msg.data.find('三') > -1:
            pose = waypoints[1]
        else:
            return;

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
        client.wait_for_server()
            
        print("goal:x=%f y=%f"%(pose[0][0],pose[0][1]))
        goal = goal_pose(pose)
        client.send_goal(goal)
        client.wait_for_result()


 
 
if __name__ == '__main__':

    rospy.init_node('message_publisher')

    pub=rospy.Publisher('voiceWakeup',String,queue_size = 1)

    rospy.sleep(1)
    pub.publish('wake up')#唤醒语音识别系统

    sub = rospy.Subscriber('voiceWords', String, callback)#监听识别结果的消息传入回调函数
    rospy.spin()

