//
// File: PID_tuning.h
//
// Code generated for Simulink model 'PID_tuning'.
//
// Model version                  : 1.24
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Sun Mar 27 21:04:03 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_PID_tuning_h_
#define RTW_HEADER_PID_tuning_h_
#include "rtwtypes.h"
#include "slros_initialize.h"
#include "PID_tuning_types.h"

extern "C" {

#include "rt_nonfinite.h"

}
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
struct B_PID_tuning_T {
  SL_Bus_PID_tuning_sensor_msgs_Joy In1;// '<S12>/In1'
  SL_Bus_PID_tuning_sensor_msgs_Joy b_varargout_2;
  real_T DataTypeConversion7[128];     // '<Root>/Data Type Conversion7'
  int32_T DataTypeConversion8[128];    // '<Root>/Data Type Conversion8'
  SL_Bus_PID_tuning_geometry_msgs_Pose BusAssignment2;// '<Root>/Bus Assignment2' 
  SL_Bus_PID_tuning_std_msgs_Float32 In1_i;// '<S13>/In1'
};

// Block states (default storage) for system '<Root>'
struct DW_PID_tuning_T {
  ros_slroscpp_internal_block_P_T obj; // '<S9>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_e;// '<S8>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_j;// '<S7>/SinkBlock'
  ros_slroscpp_internal_block_S_T obj_el;// '<S11>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_d;// '<S10>/SourceBlock'
  real_T UnitDelay3_DSTATE;            // '<Root>/Unit Delay3'
  real_T UnitDelay2_DSTATE;            // '<Root>/Unit Delay2'
  real_T UnitDelay1_DSTATE;            // '<Root>/Unit Delay1'
  real_T UnitDelay_DSTATE;             // '<Root>/Unit Delay'
  real_T UnitDelay6_DSTATE;            // '<Root>/Unit Delay6'
  real_T UnitDelay5_DSTATE;            // '<Root>/Unit Delay5'
  real_T UnitDelay4_DSTATE;            // '<Root>/Unit Delay4'
  real32_T UnitDelay7_DSTATE;          // '<Root>/Unit Delay7'
};

// Parameters (default storage)
struct P_PID_tuning_T_ {
  SL_Bus_PID_tuning_sensor_msgs_Joy Out1_Y0;// Computed Parameter: Out1_Y0
                                               //  Referenced by: '<S12>/Out1'

  SL_Bus_PID_tuning_sensor_msgs_Joy Constant_Value;// Computed Parameter: Constant_Value
                                                      //  Referenced by: '<S10>/Constant'

  SL_Bus_PID_tuning_geometry_msgs_Pose Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                           //  Referenced by: '<S3>/Constant'

  SL_Bus_PID_tuning_std_msgs_Float32 Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                         //  Referenced by: '<S2>/Constant'

  SL_Bus_PID_tuning_std_msgs_Float32 Out1_Y0_p;// Computed Parameter: Out1_Y0_p
                                                  //  Referenced by: '<S13>/Out1'

  SL_Bus_PID_tuning_std_msgs_Float32 Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                         //  Referenced by: '<S11>/Constant'

  SL_Bus_PID_tuning_std_msgs_Int16 Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                       //  Referenced by: '<S1>/Constant'

  real_T Gain1_Gain;                   // Expression: 0.1
                                          //  Referenced by: '<Root>/Gain1'

  real_T Constant_Value_bm;            // Expression: 0
                                          //  Referenced by: '<Root>/Constant'

  real_T UnitDelay3_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<Root>/Unit Delay3'

  real_T UnitDelay2_InitialCondition;  // Expression: 0.4
                                          //  Referenced by: '<Root>/Unit Delay2'

  real_T UnitDelay1_InitialCondition;  // Expression: 1.25
                                          //  Referenced by: '<Root>/Unit Delay1'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<Root>/Unit Delay'

  real_T UnitDelay6_InitialCondition;  // Expression: 1
                                          //  Referenced by: '<Root>/Unit Delay6'

  real_T Saturation1_UpperSat;         // Expression: 250
                                          //  Referenced by: '<Root>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: 0
                                          //  Referenced by: '<Root>/Saturation1'

  real_T UnitDelay5_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<Root>/Unit Delay5'

  real_T UnitDelay4_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<Root>/Unit Delay4'

  real_T Gain_Gain;                    // Expression: 1/0.1
                                          //  Referenced by: '<Root>/Gain'

  real_T Saturation_UpperSat;          // Expression: 255
                                          //  Referenced by: '<Root>/Saturation'

  real_T Saturation_LowerSat;          // Expression: 0
                                          //  Referenced by: '<Root>/Saturation'

  real32_T UnitDelay7_InitialCondition;
                              // Computed Parameter: UnitDelay7_InitialCondition
                                 //  Referenced by: '<Root>/Unit Delay7'

};

// Real-time Model Data Structure
struct tag_RTM_PID_tuning_T {
  const char_T * volatile errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_PID_tuning_T PID_tuning_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern struct B_PID_tuning_T PID_tuning_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern struct DW_PID_tuning_T PID_tuning_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void PID_tuning_initialize(void);
  extern void PID_tuning_step(void);
  extern void PID_tuning_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_PID_tuning_T *const PID_tuning_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<Root>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<Root>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<Root>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<Root>/Data Type Conversion5' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'PID_tuning'
//  '<S1>'   : 'PID_tuning/Blank Message'
//  '<S2>'   : 'PID_tuning/Blank Message1'
//  '<S3>'   : 'PID_tuning/Blank Message2'
//  '<S4>'   : 'PID_tuning/MATLAB Function'
//  '<S5>'   : 'PID_tuning/MATLAB Function1'
//  '<S6>'   : 'PID_tuning/MATLAB Function2'
//  '<S7>'   : 'PID_tuning/Publish'
//  '<S8>'   : 'PID_tuning/Publish1'
//  '<S9>'   : 'PID_tuning/Publish2'
//  '<S10>'  : 'PID_tuning/Subscribe'
//  '<S11>'  : 'PID_tuning/Subscribe1'
//  '<S12>'  : 'PID_tuning/Subscribe/Enabled Subsystem'
//  '<S13>'  : 'PID_tuning/Subscribe1/Enabled Subsystem'

#endif                                 // RTW_HEADER_PID_tuning_h_

//
// File trailer for generated code.
//
// [EOF]
//
