/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "laikago_controller/joint_controller.h"
#include "laikago_controller/laikago_control_tool.h"
#include <pluginlib/class_list_macros.h>

namespace laikago_controller {

    ServoCmd servoCmd;

    LaikagoJointController::LaikagoJointController(){}

    LaikagoJointController::~LaikagoJointController(){
        sub_cmd.shutdown();
    }

    void LaikagoJointController::setCommandCB(const laikago_msgs::MotorCmdConstPtr& msg)
    {
        lastCmd.mode = msg->mode;
        lastCmd.position = msg->position;
        lastCmd.positionStiffness = msg->positionStiffness;
        lastCmd.velocity = msg->velocity;
        lastCmd.velocityStiffness = msg->velocityStiffness;
        lastCmd.torque = msg->torque;
        // the writeFromNonRT can be used in RT, if you have the guarantee that
        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        //  * there is only one single rt thread
        command.writeFromNonRT(lastCmd);
    }

    // Controller initialization in non-realtime
    bool LaikagoJointController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        name_space = n.getNamespace();
        if (!n.getParam("joint", joint_name)){
            ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
            return false;
        }
        urdf::Model urdf; // Get URDF info about joint
        if (!urdf.initParamWithNodeHandle("robot_description", n)){
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf){
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        joint = robot->getHandle(joint_name);
        // Start command subscriber
        sub_cmd = n.subscribe("command", 20, &LaikagoJointController::setCommandCB, this);
 
        // Start realtime state publisher
        controller_state_publisher_.reset(
            new realtime_tools::RealtimePublisher<laikago_msgs::MotorState>(n, name_space + "/state", 1));        
        return true;
    }

    void LaikagoJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void LaikagoJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void LaikagoJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }

    // Controller startup in realtime
    void LaikagoJointController::starting(const ros::Time& time)
    {
        double init_pos = joint.getPosition();
        lastCmd.position = init_pos;
        lastState.position = init_pos;
        lastCmd.velocity = 0;
        lastState.velocity = 0;
        lastCmd.torque = 0;
        lastState.torque = 0;
        command.initRT(lastCmd);

        pid_controller_.reset();
    }

    // Controller update loop in realtime
    void LaikagoJointController::update(const ros::Time& time, const ros::Duration& period)
    {
        double currentPos, currentVel, calcTorque;
        lastCmd = *(command.readFromRT());


        servoCmd.pos = lastCmd.position;
        positionLimits(servoCmd.pos);
        servoCmd.posStiffness = lastCmd.positionStiffness;
        if(fabs(lastCmd.position - PosStopF) < 0.00001){
            servoCmd.posStiffness = 0;
        }
        servoCmd.vel = lastCmd.velocity;
        velocityLimits(servoCmd.vel);
        servoCmd.velStiffness = lastCmd.velocityStiffness;
        if(fabs(lastCmd.velocity - VelStopF) < 0.00001){
            servoCmd.velStiffness = 0;
        }
        servoCmd.torque = lastCmd.torque;
        effortLimits(servoCmd.torque);

        currentPos = joint.getPosition();
        currentVel = computeVel(currentPos, (double)lastState.position, (double)lastState.velocity, period.toSec());
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);      
        effortLimits(calcTorque);

        joint.setCommand(calcTorque);

        lastState.position = currentPos;
        lastState.velocity = currentVel;
        lastState.torque = calcTorque;
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.position = lastState.position;
            controller_state_publisher_->msg_.velocity = lastState.velocity;
            controller_state_publisher_->msg_.torque = lastState.torque;
            controller_state_publisher_->unlockAndPublish();
        }
    }

    // Controller stopping in realtime
    void LaikagoJointController::stopping(){}

    void LaikagoJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void LaikagoJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void LaikagoJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(laikago_controller::LaikagoJointController, controller_interface::ControllerBase);
