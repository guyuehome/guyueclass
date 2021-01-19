/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Omid Heidari 
   Commentator：GuYueHome
*/

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <ros/ros.h>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
LERPInterface::LERPInterface(const ros::NodeHandle& nh) : nh_(nh), name_("LERPInterface")
{
}

//运动规划求解函数
bool LERPInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req,
                          moveit_msgs::MotionPlanDetailedResponse& res)
{
  // 通过ROS Parameter服务器加载规划器需要的参数
  nh_.getParam("num_steps", num_steps_);

  //调用算法前所需要的一些基本数据，包括起始时间、机器人模型、起始位姿、关节名称等
  ros::WallTime start_time = ros::WallTime::now();
  robot_model::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
  robot_state::RobotStatePtr start_state(new robot_state::RobotState(robot_model));
  *start_state = planning_scene->getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = start_state->getJointModelGroup(req.group_name);
  std::vector<std::string> joint_names = joint_model_group->getVariableNames();
  dof_ = joint_names.size();
  std::vector<double> start_joint_values;
  start_state->copyJointGroupPositions(joint_model_group, start_joint_values);

  // 该规划器只支持一个目标位姿的情况，设置目标姿态
  std::vector<moveit_msgs::Constraints> goal_constraints = req.goal_constraints;
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = goal_constraints[0].joint_constraints;

  std::vector<double> goal_joint_values;
  for (auto constraint : goal_joint_constraint)
  {
    goal_joint_values.push_back(constraint.position);
  }

  //调用线性插补算法
  trajectory_msgs::JointTrajectory joint_trajectory;
  interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);

  //封装需要反馈的应答数据，关节名、轨迹数据、时间戳等
  res.trajectory.resize(1);
  res.trajectory[0].joint_trajectory.joint_names = joint_names;
  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;
  res.trajectory[0].joint_trajectory = joint_trajectory;

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

  res.group_name = req.group_name;
  res.trajectory_start.joint_state.name = joint_names;
  res.trajectory_start.joint_state.position = start_joint_values;

  return true;
}

//线性插补算法
void LERPInterface::interpolate(const std::vector<std::string> joint_names, robot_state::RobotStatePtr& rob_state,
                                const robot_state::JointModelGroup* joint_model_group,
                                const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                                trajectory_msgs::JointTrajectory& joint_trajectory)
{
  joint_trajectory.points.resize(num_steps_ + 1);

  //根据获取的num_steps_参数，计算插补点之间的平均时间
  std::vector<double> dt_vector;
  for (int joint_index = 0; joint_index < dof_; ++joint_index)
  {
    double dt = (goal_joint_vals[joint_index] - start_joint_vals[joint_index]) / num_steps_;
    dt_vector.push_back(dt);
  }

  //根据平均时间，计算每个插补点的关节角度
  for (int step = 0; step <= num_steps_; ++step)
  {
    std::vector<double> joint_values;
    for (int k = 0; k < dof_; ++k)
    {
      double joint_value = start_joint_vals[k] + step * dt_vector[k];
      joint_values.push_back(joint_value);
    }
    rob_state->setJointGroupPositions(joint_model_group, joint_values);
    rob_state->update();

    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points[step].positions = joint_values;

    //将规划出的轨迹点打印出来
    for(unsigned int i=0; i<joint_values.size(); i++)
        printf("%0.4f  ", joint_values[i]);
    printf("\n");
  }
}

}  // namespace lerp_interface
