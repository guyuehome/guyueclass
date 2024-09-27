//
// File: FuzzyPID_types.h
//
// Code generated for Simulink model 'FuzzyPID'.
//
// Model version                  : 1.30
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Wed Mar 30 17:31:29 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_FuzzyPID_types_h_
#define RTW_HEADER_FuzzyPID_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_std_msgs_Int16_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_std_msgs_Int16_

// MsgType=std_msgs/Int16
struct SL_Bus_FuzzyPID_std_msgs_Int16
{
  int16_T Data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_std_msgs_Float32_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_std_msgs_Float32_

// MsgType=std_msgs/Float32
struct SL_Bus_FuzzyPID_std_msgs_Float32
{
  real32_T Data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
struct SL_Bus_FuzzyPID_geometry_msgs_Point
{
  real_T X;
  real_T Y;
  real_T Z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_geometry_msgs_Quaternion_

// MsgType=geometry_msgs/Quaternion
struct SL_Bus_FuzzyPID_geometry_msgs_Quaternion
{
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_geometry_msgs_Pose_

// MsgType=geometry_msgs/Pose
struct SL_Bus_FuzzyPID_geometry_msgs_Pose
{
  // MsgType=geometry_msgs/Point
  SL_Bus_FuzzyPID_geometry_msgs_Point Position;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_FuzzyPID_geometry_msgs_Quaternion Orientation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_ros_time_Time_

// MsgType=ros_time/Time
struct SL_Bus_FuzzyPID_ros_time_Time
{
  real_T Sec;
  real_T Nsec;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_std_msgs_Header_

// MsgType=std_msgs/Header
struct SL_Bus_FuzzyPID_std_msgs_Header
{
  uint32_T Seq;

  // MsgType=ros_time/Time
  SL_Bus_FuzzyPID_ros_time_Time Stamp;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_sensor_msgs_Joy_
#define DEFINED_TYPEDEF_FOR_SL_Bus_FuzzyPID_sensor_msgs_Joy_

// MsgType=sensor_msgs/Joy
struct SL_Bus_FuzzyPID_sensor_msgs_Joy
{
  // MsgType=std_msgs/Header
  SL_Bus_FuzzyPID_std_msgs_Header Header;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Axes_SL_Info:TruncateAction=warn
  real32_T Axes[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Axes
  SL_Bus_ROSVariableLengthArrayInfo Axes_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Buttons_SL_Info:TruncateAction=warn 
  int32_T Buttons[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Buttons
  SL_Bus_ROSVariableLengthArrayInfo Buttons_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_slmtA0QxwbT8yc5iP4GAzD_
#define DEFINED_TYPEDEF_FOR_struct_slmtA0QxwbT8yc5iP4GAzD_

struct struct_slmtA0QxwbT8yc5iP4GAzD
{
  uint8_T SimulinkDiagnostic;
  uint8_T Model[8];
  uint8_T Block[33];
  uint8_T OutOfRangeInputValue;
  uint8_T NoRuleFired;
  uint8_T EmptyOutputFuzzySet;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_RCP1oovUDF2zF6C9vcJmtG_
#define DEFINED_TYPEDEF_FOR_struct_RCP1oovUDF2zF6C9vcJmtG_

struct struct_RCP1oovUDF2zF6C9vcJmtG
{
  uint8_T type[5];
  int32_T origTypeLength;
  real_T params[3];
  int32_T origParamLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_xaJpTrGDqNmvd5vaYMDkGD_
#define DEFINED_TYPEDEF_FOR_struct_xaJpTrGDqNmvd5vaYMDkGD_

struct struct_xaJpTrGDqNmvd5vaYMDkGD
{
  struct_RCP1oovUDF2zF6C9vcJmtG mf[7];
  int32_T origNumMF;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_gR18QYWZ84M9bzJEMwytNH_
#define DEFINED_TYPEDEF_FOR_struct_gR18QYWZ84M9bzJEMwytNH_

struct struct_gR18QYWZ84M9bzJEMwytNH
{
  uint8_T type[7];
  uint8_T andMethod[3];
  uint8_T orMethod[3];
  uint8_T defuzzMethod[8];
  uint8_T impMethod[3];
  uint8_T aggMethod[3];
  real_T inputRange[4];
  real_T outputRange[6];
  struct_xaJpTrGDqNmvd5vaYMDkGD inputMF[2];
  struct_xaJpTrGDqNmvd5vaYMDkGD outputMF[3];
  real_T antecedent[98];
  real_T consequent[147];
  real_T connection[49];
  real_T weight[49];
  int32_T numSamples;
  int32_T numInputs;
  int32_T numOutputs;
  int32_T numRules;
  int32_T numInputMFs[2];
  int32_T numCumInputMFs[2];
  int32_T numOutputMFs[3];
  int32_T numCumOutputMFs[3];
  real_T outputSamplePoints[303];
  int32_T orrSize[2];
  int32_T aggSize[2];
  int32_T irrSize[2];
  int32_T rfsSize[2];
  int32_T sumSize[2];
  int32_T inputFuzzySetType;
};

#endif

#ifndef struct_ros_slroscpp_internal_block_P_T
#define struct_ros_slroscpp_internal_block_P_T

struct ros_slroscpp_internal_block_P_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                // struct_ros_slroscpp_internal_block_P_T

#ifndef struct_ros_slroscpp_internal_block_S_T
#define struct_ros_slroscpp_internal_block_S_T

struct ros_slroscpp_internal_block_S_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                // struct_ros_slroscpp_internal_block_S_T

// Parameters (default storage)
typedef struct P_FuzzyPID_T_ P_FuzzyPID_T;

// Forward declaration for rtModel
typedef struct tag_RTM_FuzzyPID_T RT_MODEL_FuzzyPID_T;

#endif                                 // RTW_HEADER_FuzzyPID_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
