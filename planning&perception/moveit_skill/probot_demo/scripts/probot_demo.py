#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander
from copy import deepcopy

class ProbotDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('probot_demo')
        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
               
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        position1_up = [0.25105589628219604, -0.3714791238307953, -0.5240143537521362, -3.899861258105375e-05, 0.8954282999038696, 0.2508518099784851]
        position1_down = [0.25105589628219604, -0.9745101928710938, -0.4310262203216553, -0.0001481490326113999, 1.4054712057113647, 0.25096750259399414]
        position2_up = [-0.5309033393859863, -0.37207022309303284, -0.524405837059021, 0.00010066419781651348, 0.896560549736023, 0.2511337101459503]
        position2_down = [-0.5309033393859863, -0.9512181878089905, -0.44115084409713745, 0.00035425653913989663, 1.3924535512924194, 0.2508637011051178]

        # 设置机器臂当前的状态作为运动初始状态        
        arm.set_joint_value_target(position1_up)           
        arm.go()
        arm.set_joint_value_target(position1_down)           
        arm.go()
        
        arm.set_joint_value_target(position1_up)           
        arm.go()
        arm.set_joint_value_target(position2_up)           
        arm.go()
        arm.set_joint_value_target(position2_down)           
        arm.go()

        arm.set_joint_value_target(position2_up)           
        arm.go()

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose

        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)
            
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        waypoints.append(deepcopy(wpose))
        
        wpose.position.y += 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.x -= 0.1
        wpose.position.y -= 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        waypoints.append(deepcopy(wpose))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        target_position = [-0.012496701441705227, -0.41917115449905396, -0.19433578848838806, 0.018087295815348625, 2.2288901805877686, -0.013513846322894096]
        arm.set_joint_value_target(target_position)           
        arm.go()

        target_position = [-0.012496701441705227, -0.2822195291519165, -0.05068935826420784, 0.030009545385837555, 1.9483672380447388, 2.1740968227386475]
        arm.set_joint_value_target(target_position)           
        arm.go()

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    ProbotDemo()
    
