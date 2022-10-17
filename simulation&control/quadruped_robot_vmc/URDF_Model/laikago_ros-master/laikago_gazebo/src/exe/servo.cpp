/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "../body.h"

using namespace std;
using namespace laikago_model;

bool start_up = true;

class multiThread
{
public:
    multiThread(){
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/laikago_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/laikago_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/laikago_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/laikago_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/laikago_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/laikago_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/laikago_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/laikago_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/laikago_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/laikago_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/laikago_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/laikago_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;
    }

    void FRhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].position = msg.position;
        lowState.motorState[0].velocity = msg.velocity;
        lowState.motorState[0].torque = msg.torque;
    }

    void FRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].position = msg.position;
        lowState.motorState[1].velocity = msg.velocity;
        lowState.motorState[1].torque = msg.torque;
    }

    void FRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].position = msg.position;
        lowState.motorState[2].velocity = msg.velocity;
        lowState.motorState[2].torque = msg.torque;
    }

    void FLhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].position = msg.position;
        lowState.motorState[3].velocity = msg.velocity;
        lowState.motorState[3].torque = msg.torque;
    }

    void FLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].position = msg.position;
        lowState.motorState[4].velocity = msg.velocity;
        lowState.motorState[4].torque = msg.torque;
    }

    void FLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].position = msg.position;
        lowState.motorState[5].velocity = msg.velocity;
        lowState.motorState[5].torque = msg.torque;
    }

    void RRhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].position = msg.position;
        lowState.motorState[6].velocity = msg.velocity;
        lowState.motorState[6].torque = msg.torque;
    }

    void RRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].position = msg.position;
        lowState.motorState[7].velocity = msg.velocity;
        lowState.motorState[7].torque = msg.torque;
    }

    void RRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].position = msg.position;
        lowState.motorState[8].velocity = msg.velocity;
        lowState.motorState[8].torque = msg.torque;
    }

    void RLhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].position = msg.position;
        lowState.motorState[9].velocity = msg.velocity;
        lowState.motorState[9].torque = msg.torque;
    }

    void RLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].position = msg.position;
        lowState.motorState[10].velocity = msg.velocity;
        lowState.motorState[10].torque = msg.torque;
    }

    void RLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].position = msg.position;
        lowState.motorState[11].velocity = msg.velocity;
        lowState.motorState[11].torque = msg.torque;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laikago_gazebo_servo");

    multiThread listen_publish_obj;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    // ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);

    motion_init();

    while (ros::ok()){
        /*
        control logic
        */
        lowState_pub.publish(lowState);
        sendServoCmd();
    }
    return 0;
}
