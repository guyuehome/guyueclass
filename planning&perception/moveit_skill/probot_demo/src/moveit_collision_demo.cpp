/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_collision_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      sleep_t.sleep();
    }

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 创建运动规划的情景
    moveit_msgs::PlanningScene planning_scene;

    // 声明一个障碍物体
    moveit_msgs::CollisionObject add_object;
    add_object.id = "cylinder";
    add_object.header.frame_id = "base_link";

    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive add_object_primitive;
    add_object_primitive.type = add_object_primitive.CYLINDER;
    add_object_primitive.dimensions.resize(3);
    add_object_primitive.dimensions[0] = 0.6;
    add_object_primitive.dimensions[1] = 0.05;

    // 设置障碍物的位置
    geometry_msgs::Pose add_object_pose;
    add_object_pose.orientation.w = 1.0;
    add_object_pose.position.x =  0.2;
    add_object_pose.position.y = -0.2;
    add_object_pose.position.z =  0.3;

    // 将障碍物的属性、位置加入到障碍物的实例中
    add_object.primitives.push_back(add_object_primitive);
    add_object.primitive_poses.push_back(add_object_pose);
    add_object.operation = add_object.ADD;

    // 声明一个附着物体
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "tool0";
    attached_object.object.header.frame_id = "tool0";
    attached_object.object.id = "box";

    // 设置附着物体的位置
    geometry_msgs::Pose attached_object_pose;
    attached_object_pose.orientation.w = 1.0;
    attached_object_pose.position.y = -0.05;

    // 设置附着物体的外形、尺寸等属性   
    shape_msgs::SolidPrimitive attached_object_primitive;
    attached_object_primitive.type = attached_object_primitive.BOX;
    attached_object_primitive.dimensions.resize(3);
    attached_object_primitive.dimensions[0] = 0.02;
    attached_object_primitive.dimensions[1] = 0.1;
    attached_object_primitive.dimensions[2] = 0.02;

    attached_object.object.primitives.push_back(attached_object_primitive);
    attached_object.object.primitive_poses.push_back(attached_object_pose);
    attached_object.object.operation = attached_object.object.ADD;
    attached_object.touch_links = std::vector<std::string>{ "link_5", "link_6", "tool0" };

    // 所有障碍物加入列表后，再把障碍物加入到当前的情景中
    planning_scene.world.collision_objects.push_back(add_object);
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    // 声明去附着属性的物体
    moveit_msgs::CollisionObject remove_box_object;
    remove_box_object.id = "box";
    remove_box_object.header.frame_id = "tool0";
    remove_box_object.operation = remove_box_object.REMOVE;

    // 将物体附着到机器人上，并从场景中删除
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_box_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_diff_publisher.publish(planning_scene);

    // 机器人运动目标位置
    double targetPose[6] = {-1.4937774598490543, -0.7161902803216305, 0.30090498871655375, -9.053240794012654e-05, 0.4152909726912105, -1.493909198433862};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    // 机器人蔽障运动
    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(2);

    // 声明去附着属性的物体
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "box";
    detach_object.link_name = "tool0";
    detach_object.object.operation = attached_object.object.REMOVE;

    // 声明障碍物体
    moveit_msgs::CollisionObject remove_cylinder_object;
    remove_cylinder_object.id = "cylinder";
    remove_cylinder_object.header.frame_id = "base_link";
    remove_cylinder_object.operation = remove_box_object.REMOVE;

    // 在场景中删除障碍物体，去除
    ROS_INFO("Detaching the object from the robot and returning it to the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene.world.collision_objects.clear();
    //planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.world.collision_objects.push_back(remove_cylinder_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    sleep(1);

    // 控制机械臂先回到初始化位置
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown();

    return 0;
}
