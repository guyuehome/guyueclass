//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: turtletrajectory_types.h
//
// Code generated for Simulink model 'turtletrajectory'.
//
// Model version                  : 1.1
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Aug 14 12:56:48 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_turtletrajectory_types_h_
#define RTW_HEADER_turtletrajectory_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_turtletrajectory_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_turtletrajectory_geometry_msgs_Vector3_

// MsgType=geometry_msgs/Vector3
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_turtletrajectory_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_turtletrajectory_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_turtletrajectory_geometry_msgs_Twist_

// MsgType=geometry_msgs/Twist
typedef struct {
  // MsgType=geometry_msgs/Vector3
  SL_Bus_turtletrajectory_geometry_msgs_Vector3 Linear;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_turtletrajectory_geometry_msgs_Vector3 Angular;
} SL_Bus_turtletrajectory_geometry_msgs_Twist;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_turtletrajectory_turtlesim_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_turtletrajectory_turtlesim_Pose_

// MsgType=turtlesim/Pose
typedef struct {
  real32_T X;
  real32_T Y;
  real32_T Theta;
  real32_T LinearVelocity;
  real32_T AngularVelocity;
} SL_Bus_turtletrajectory_turtlesim_Pose;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_e_T
#define typedef_robotics_slros_internal_blo_e_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_blo_e_T;

#endif                                 //typedef_robotics_slros_internal_blo_e_T

// Parameters (default storage)
typedef struct P_turtletrajectory_T_ P_turtletrajectory_T;

// Forward declaration for rtModel
typedef struct tag_RTM_turtletrajectory_T RT_MODEL_turtletrajectory_T;

#endif                                 // RTW_HEADER_turtletrajectory_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
