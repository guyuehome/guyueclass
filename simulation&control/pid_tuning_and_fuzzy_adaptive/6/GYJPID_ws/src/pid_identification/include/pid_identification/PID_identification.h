//
// File: PID_identification.h
//
// Code generated for Simulink model 'PID_identification'.
//
// Model version                  : 1.5
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Sun Mar 27 13:28:42 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_PID_identification_h_
#define RTW_HEADER_PID_identification_h_
#include "rtwtypes.h"
#include "slros_initialize.h"
#include "PID_identification_types.h"

extern "C" {

#include "rtGetInf.h"

}
  extern "C"
{

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
struct B_PID_identification_T {
  SL_Bus_PID_identification_sensor_msgs_Joy In1;// '<S5>/In1'
  SL_Bus_PID_identification_sensor_msgs_Joy b_varargout_2;
};

// Block states (default storage) for system '<Root>'
struct DW_PID_identification_T {
  ros_slroscpp_internal_block_P_T obj; // '<S3>/SinkBlock'
  ros_slroscpp_internal_block_S_T obj_d;// '<S4>/SourceBlock'
  real32_T UnitDelay_DSTATE;           // '<Root>/Unit Delay'
};

// Parameters (default storage)
struct P_PID_identification_T_ {
  SL_Bus_PID_identification_sensor_msgs_Joy Out1_Y0;// Computed Parameter: Out1_Y0
                                                       //  Referenced by: '<S5>/Out1'

  SL_Bus_PID_identification_sensor_msgs_Joy Constant_Value;// Computed Parameter: Constant_Value
                                                              //  Referenced by: '<S4>/Constant'

  SL_Bus_PID_identification_std_msgs_Int16 Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                               //  Referenced by: '<S1>/Constant'

  real32_T UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<Root>/Unit Delay'

  real32_T Saturation_UpperSat;       // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<Root>/Saturation'

  real32_T Saturation_LowerSat;       // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<Root>/Saturation'

};

// Real-time Model Data Structure
struct tag_RTM_PID_identification_T {
  const char_T * volatile errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_PID_identification_T PID_identification_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern struct B_PID_identification_T PID_identification_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern struct DW_PID_identification_T PID_identification_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void PID_identification_initialize(void);
  extern void PID_identification_step(void);
  extern void PID_identification_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_PID_identification_T *const PID_identification_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion


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
//  '<Root>' : 'PID_identification'
//  '<S1>'   : 'PID_identification/Blank Message'
//  '<S2>'   : 'PID_identification/MATLAB Function'
//  '<S3>'   : 'PID_identification/Publish'
//  '<S4>'   : 'PID_identification/Subscribe'
//  '<S5>'   : 'PID_identification/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_PID_identification_h_

//
// File trailer for generated code.
//
// [EOF]
//
