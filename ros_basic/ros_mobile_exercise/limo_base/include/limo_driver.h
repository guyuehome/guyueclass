/*
 * Copyright (c) 2021, Agilex Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LIMO_DRIVER_H
#define LIMO_DRIVER_H

#include <iostream>
#include <thread>
#include <memory.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <limo_base/LimoStatus.h>
#include "serial_port.h"
#include "limo_protocol.h"

namespace AgileX {

class LimoDriver {
public:
    LimoDriver();
    ~LimoDriver();

private:
    void connect(std::string dev_name, uint32_t bouadrate);
    void readData();
    void processRxData(uint8_t data);
    void parseFrame(const LimoFrame& frame);
    void sendFrame(const LimoFrame& frame);
    void setMotionCommand(double linear_vel, double steer_angle,
                          double lateral_vel, double angular_vel);
    void enableCommandedMode();
    void publishOdometry(double stamp, double linear_velocity,
                         double angular_velocity, double lateral_velocity,
                         double steering_angle);
    void publishLimoState(double stamp, uint8_t vehicle_state, uint8_t control_mode,
                          double battery_voltage, uint16_t error_code, int8_t motion_mode);
    void publishIMUData(double stamp);
    void processErrorCode(uint16_t error_code);
    void twistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    double normalizeAngle(double angle);
    double degToRad(double deg);
    double convertInnerAngleToCentral(double inner_angle);
    double convertCentralAngleToInner(double central_angle);

private:
    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;

    std::string odom_frame_;
    std::string base_frame_;
    bool pub_odom_tf_ = false;
    bool use_mcnamu_ = false;

    ros::Publisher odom_publisher_;
    ros::Publisher status_publisher_;
    ros::Publisher imu_publisher_;
    ros::Subscriber motion_cmd_sub_;
    tf::TransformBroadcaster tf_broadcaster_;

    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;
    double present_theta_;
    double last_theta_;
    double delta_theta_;
    double real_theta_;
    double rad;
    
    ImuData imu_data_;
    uint8_t motion_mode_;  // current motion type

    static constexpr double max_inner_angle_ = 0.48869;  // 28 degree
    static constexpr double track_ = 0.172;           // m (left right wheel distance)
    static constexpr double wheelbase_ = 0.2;         // m (front rear wheel distance)
    static constexpr double left_angle_scale_ = 2.47;
    static constexpr double right_angle_scale_ = 1.64;
};

}

#endif // LIMO_DRIVER_H
