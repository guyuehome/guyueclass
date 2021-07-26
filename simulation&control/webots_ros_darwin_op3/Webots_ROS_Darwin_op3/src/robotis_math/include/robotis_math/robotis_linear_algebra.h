/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * robotis_linear_algebra.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef ROBOTIS_MATH_ROBOTIS_LINEAR_ALGEBRA_H_
#define ROBOTIS_MATH_ROBOTIS_LINEAR_ALGEBRA_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <cmath>

#include <eigen3/Eigen/Dense>
#include "step_data_define.h"

namespace robotis_framework
{

Eigen::Vector3d getTransitionXYZ(double position_x, double position_y, double position_z);
Eigen::Matrix4d getTransformationXYZRPY(double position_x, double position_y, double position_z , double roll, double pitch, double yaw);
Eigen::Matrix4d getInverseTransformation(const Eigen::MatrixXd& transform);
Eigen::Matrix3d getInertiaXYZ(double ixx, double ixy, double ixz , double iyy , double iyz, double izz);
Eigen::Matrix3d getRotationX(double angle);
Eigen::Matrix3d getRotationY(double angle);
Eigen::Matrix3d getRotationZ(double angle);
Eigen::Matrix4d getRotation4d(double roll, double pitch, double yaw);
Eigen::Matrix4d getTranslation4D(double position_x, double position_y, double position_z);

Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d& rotation);
Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d& rotation);
Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond& quaternion);
Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond& quaternion);

Eigen::Matrix3d calcHatto(const Eigen::Vector3d& matrix3d);
Eigen::Matrix3d calcRodrigues(const Eigen::Matrix3d& hat_matrix, double angle);
Eigen::Vector3d convertRotToOmega(const Eigen::Matrix3d& rotation);
Eigen::Vector3d calcCross(const Eigen::Vector3d& vector3d_a, const Eigen::Vector3d& vector3d_b);
double calcInner(const Eigen::MatrixXd& vector3d_a, const Eigen::MatrixXd& vector3d_b);

Pose3D getPose3DfromTransformMatrix(const Eigen::Matrix4d& transform);

}



#endif /* ROBOTIS_MATH_ROBOTIS_LINEAR_ALGEBRA_H_ */
