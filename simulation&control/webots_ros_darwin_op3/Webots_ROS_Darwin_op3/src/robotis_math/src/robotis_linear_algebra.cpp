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
 * robotis_linear_algebra.cpp
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#include "robotis_math/robotis_linear_algebra.h"

namespace robotis_framework
{

Eigen::Vector3d getTransitionXYZ(double position_x, double position_y, double position_z)
{
  Eigen::Vector3d position;

  position <<
      position_x,
      position_y,
      position_z;

  return position;
}

Eigen::Matrix4d getTransformationXYZRPY(double position_x, double position_y, double position_z , double roll , double pitch , double yaw)
{
  Eigen::Matrix4d transformation = getRotation4d(roll, pitch, yaw);
  transformation.coeffRef(0,3) = position_x;
  transformation.coeffRef(1,3) = position_y;
  transformation.coeffRef(2,3) = position_z;

  return transformation;
}

Eigen::Matrix4d getInverseTransformation(const Eigen::MatrixXd& transform)
{
  // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

  Eigen::Vector3d vec_boa;
  Eigen::Vector3d vec_x, vec_y, vec_z;
  Eigen::Matrix4d inv_t;

  vec_boa(0) = -transform(0,3);
  vec_boa(1) = -transform(1,3);
  vec_boa(2) = -transform(2,3);

  vec_x(0) = transform(0,0); vec_x(1) = transform(1,0); vec_x(2) = transform(2,0);
  vec_y(0) = transform(0,1); vec_y(1) = transform(1,1); vec_y(2) = transform(2,1);
  vec_z(0) = transform(0,2); vec_z(1) = transform(1,2); vec_z(2) = transform(2,2);

  inv_t <<
      vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
      vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
      vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
      0, 0, 0, 1;

  return inv_t;
}

Eigen::Matrix3d getInertiaXYZ(double ixx, double ixy, double ixz , double iyy , double iyz, double izz)
{
  Eigen::Matrix3d inertia;

  inertia <<
      ixx, ixy, ixz,
      ixy, iyy, iyz,
      ixz, iyz, izz;

  return inertia;
}

Eigen::Matrix3d getRotationX(double angle)
{
  Eigen::Matrix3d rotation(3,3);

  rotation <<
      1.0, 0.0, 0.0,
      0.0, cos(angle), -sin(angle),
      0.0, sin(angle), cos(angle);

  return rotation;
}

Eigen::Matrix3d getRotationY(double angle)
{
  Eigen::Matrix3d rotation(3,3);

  rotation <<
      cos(angle), 0.0, sin(angle),
      0.0, 1.0, 0.0,
      -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::Matrix3d getRotationZ(double angle)
{
  Eigen::Matrix3d rotation(3,3);

  rotation <<
      cos(angle), -sin(angle), 0.0,
      sin(angle), cos(angle), 0.0,
      0.0, 0.0, 1.0;

  return rotation;
}

Eigen::Matrix4d getRotation4d(double roll, double pitch, double yaw )
{
  double sr = sin(roll), cr = cos(roll);
  double sp = sin(pitch), cp = cos(pitch);
  double sy = sin(yaw), cy = cos(yaw);

  Eigen::Matrix4d mat_roll;
  Eigen::Matrix4d mat_pitch;
  Eigen::Matrix4d mat_yaw;

  mat_roll <<
      1, 0, 0, 0,
      0, cr, -sr, 0,
      0, sr, cr, 0,
      0, 0, 0, 1;

  mat_pitch <<
      cp, 0, sp, 0,
      0, 1, 0, 0,
      -sp, 0, cp, 0,
      0, 0, 0, 1;

  mat_yaw <<
      cy, -sy, 0, 0,
      sy, cy, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Eigen::Matrix4d mat_rpy = (mat_yaw*mat_pitch)*mat_roll;

  return mat_rpy;
}

Eigen::Matrix4d getTranslation4D(double position_x, double position_y, double position_z)
{
  Eigen::Matrix4d mat_translation;

  mat_translation <<
      1, 0, 0, position_x,
      0, 1, 0, position_y,
      0, 0, 1, position_z,
      0, 0, 0,          1;

  return mat_translation;
}

Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d& rotation)
{
  Eigen::Vector3d rpy;// = Eigen::MatrixXd::Zero(3,1);

  rpy.coeffRef(0,0) = atan2(rotation.coeff(2,1), rotation.coeff(2,2));
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1), 2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2 (rotation.coeff(1,0), rotation.coeff(0,0));

  return rpy;
}

Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d rotation = getRotationZ(yaw)*getRotationY(pitch)*getRotationX(roll);

  return rotation;
}

Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond quaternion;
  quaternion = convertRPYToRotation(roll,pitch,yaw);

  return quaternion;
}

Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d& rotation)
{
  Eigen::Quaterniond quaternion;
  quaternion = rotation;

  return quaternion;
}

Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond& quaternion)
{
  Eigen::Vector3d rpy = convertRotationToRPY(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond& quaternion)
{
  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  return rotation;
}

Eigen::Matrix3d calcHatto(const Eigen::Vector3d& matrix3d)
{
  Eigen::MatrixXd hatto(3,3);

  hatto <<
      0.0, -matrix3d.coeff(2,0), matrix3d.coeff(1,0),
      matrix3d.coeff(2,0), 0.0, -matrix3d.coeff(0,0),
      -matrix3d.coeff(1,0), matrix3d.coeff(0,0), 0.0;

  return hatto;
}

Eigen::Matrix3d calcRodrigues(const Eigen::Matrix3d& hat_matrix, double angle)
{
//  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(3,3);
//  Eigen::MatrixXd rodrigues = identity+hat_matrix*sin(angle)+hat_matrix*hat_matrix*(1-cos(angle));
  Eigen::Matrix3d rodrigues = hat_matrix*sin(angle)+hat_matrix*hat_matrix*(1-cos(angle));
  rodrigues.coeffRef(0,0) += 1;
  rodrigues.coeffRef(1,1) += 1;
  rodrigues.coeffRef(2,2) += 1;

  return rodrigues;
}

Eigen::Vector3d convertRotToOmega(const Eigen::Matrix3d& rotation)
{
  double eps = 1e-10;

  double alpha = (rotation.coeff(0,0)+rotation.coeff(1,1)+rotation.coeff(2,2)-1.0)/2.0;
  double alpha_dash = fabs( alpha - 1.0 );

  Eigen::Vector3d rot_to_omega;

  if( alpha_dash < eps )
  {
    rot_to_omega <<
        0.0,
        0.0,
        0.0;
  }
  else
  {
    double theta = acos(alpha);

    rot_to_omega <<
        rotation.coeff(2,1)-rotation.coeff(1,2),
        rotation.coeff(0,2)-rotation.coeff(2,0),
        rotation.coeff(1,0)-rotation.coeff(0,1);

    rot_to_omega = 0.5*(theta/sin(theta))*rot_to_omega;
  }

  return rot_to_omega;
}

Eigen::Vector3d calcCross(const Eigen::Vector3d& vector3d_a, const Eigen::Vector3d& vector3d_b)
{
  Eigen::Vector3d cross;

  cross <<
      vector3d_a.coeff(1,0)*vector3d_b.coeff(2,0)-vector3d_a.coeff(2,0)*vector3d_b.coeff(1,0),
      vector3d_a.coeff(2,0)*vector3d_b.coeff(0,0)-vector3d_a.coeff(0,0)*vector3d_b.coeff(2,0),
      vector3d_a.coeff(0,0)*vector3d_b.coeff(1,0)-vector3d_a.coeff(1,0)*vector3d_b.coeff(0,0);

  return cross;
}

double calcInner(Eigen::MatrixXd vector3d_a, Eigen::MatrixXd vector3d_b)
{
  return vector3d_a.dot(vector3d_b);
}

Pose3D getPose3DfromTransformMatrix(const Eigen::Matrix4d& transform)
{
  Pose3D pose_3d;

  pose_3d.x     = transform.coeff(0, 3);
  pose_3d.y     = transform.coeff(1, 3);
  pose_3d.z     = transform.coeff(2, 3);
  pose_3d.roll  = atan2( transform.coeff(2,1), transform.coeff(2,2));
  pose_3d.pitch = atan2(-transform.coeff(2,0), sqrt(transform.coeff(2,1)*transform.coeff(2,1) + transform.coeff(2,2)*transform.coeff(2,2)) );
  pose_3d.yaw   = atan2( transform.coeff(1,0), transform.coeff(0,0));

  return pose_3d;
}

}
