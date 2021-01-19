#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_test");
 
  //订阅move_base服务器的消息  
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);
 
  //等待连接服务器，5s等待时间限制 
  while(!move_base.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for move_base action server...");
  }
 
  ROS_INFO("Connected to move base server");

  //设定目标点
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
  goal.target_pose.pose.position.x = -5.543;
  goal.target_pose.pose.position.y = -4.779;

  goal.target_pose.pose.orientation.z = 0.645;
  goal.target_pose.pose.orientation.w = 0.764;
 
  ROS_INFO("Sending goal");
  move_base.sendGoal(goal);
 
  move_base.waitForResult();
 
  if(move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal succeeded!");
  else
    ROS_INFO("Goal failed");
 
  return 0;

}
