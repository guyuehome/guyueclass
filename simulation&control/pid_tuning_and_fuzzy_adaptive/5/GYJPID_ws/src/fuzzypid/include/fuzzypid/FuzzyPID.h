//
// File: FuzzyPID.h
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
#ifndef RTW_HEADER_FuzzyPID_h_
#define RTW_HEADER_FuzzyPID_h_
#include "rtwtypes.h"
#include "slros_initialize.h"
#include "FuzzyPID_types.h"

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
struct B_FuzzyPID_T {
  real_T outputMFCache[2121];
  real_T aggregatedOutputs[303];       // '<S4>/Evaluate Rule Consequents'
  SL_Bus_FuzzyPID_sensor_msgs_Joy In1; // '<S16>/In1'
  SL_Bus_FuzzyPID_sensor_msgs_Joy b_varargout_2;
  real_T dv[101];
  real_T dv1[101];
  real_T antecedentOutputs[49];        // '<S4>/Evaluate Rule Antecedents'
  real_T inputMFCache[14];
  SL_Bus_FuzzyPID_geometry_msgs_Pose BusAssignment2;// '<Root>/Bus Assignment2'
  real_T defuzzifiedOutputs[3];        // '<S4>/Defuzzify Outputs'
  real_T dv2[3];
  real_T dv3[2];
  real_T area;
  real_T Add;                          // '<Root>/Add'
  real_T Gain;                         // '<Root>/Gain'
  real_T rtb_TmpSignalConversionAtSFun_m;
  real_T rtb_TmpSignalConversionAtSFun_c;
  real_T rtb_TmpSignalConversionAtSFun_k;
  SL_Bus_FuzzyPID_std_msgs_Float32 In1_i;// '<S17>/In1'
};

// Block states (default storage) for system '<Root>'
struct DW_FuzzyPID_T {
  ros_slroscpp_internal_block_P_T obj; // '<S10>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_e;// '<S9>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_j;// '<S8>/SinkBlock'
  ros_slroscpp_internal_block_S_T obj_el;// '<S12>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_d;// '<S11>/SourceBlock'
  real_T UnitDelay1_DSTATE;            // '<Root>/Unit Delay1'
  real_T UnitDelay4_DSTATE;            // '<Root>/Unit Delay4'
  real_T UnitDelay5_DSTATE;            // '<Root>/Unit Delay5'
  real32_T UnitDelay7_DSTATE;          // '<Root>/Unit Delay7'
};

// Parameters (default storage)
struct P_FuzzyPID_T_ {
  SL_Bus_FuzzyPID_sensor_msgs_Joy Out1_Y0;// Computed Parameter: Out1_Y0
                                             //  Referenced by: '<S16>/Out1'

  SL_Bus_FuzzyPID_sensor_msgs_Joy Constant_Value;// Computed Parameter: Constant_Value
                                                    //  Referenced by: '<S11>/Constant'

  SL_Bus_FuzzyPID_geometry_msgs_Pose Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                         //  Referenced by: '<S3>/Constant'

  SL_Bus_FuzzyPID_std_msgs_Float32 Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                       //  Referenced by: '<S2>/Constant'

  SL_Bus_FuzzyPID_std_msgs_Float32 Out1_Y0_p;// Computed Parameter: Out1_Y0_p
                                                //  Referenced by: '<S17>/Out1'

  SL_Bus_FuzzyPID_std_msgs_Float32 Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                       //  Referenced by: '<S12>/Constant'

  SL_Bus_FuzzyPID_std_msgs_Int16 Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                     //  Referenced by: '<S1>/Constant'

  real_T OutputSamplePoints_Value[303];// Expression: fis.outputSamplePoints
                                          //  Referenced by: '<S4>/Output Sample Points'

  real_T Gain1_Gain;                   // Expression: 0.1
                                          //  Referenced by: '<Root>/Gain1'

  real_T Constant_Value_bm;            // Expression: 0
                                          //  Referenced by: '<Root>/Constant'

  real_T UnitDelay1_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<Root>/Unit Delay1'

  real_T Saturation1_UpperSat;         // Expression: 250
                                          //  Referenced by: '<Root>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: 0
                                          //  Referenced by: '<Root>/Saturation1'

  real_T Gain14_Gain;                  // Expression: 0.01
                                          //  Referenced by: '<Root>/Gain14'

  real_T Saturation2_UpperSat;         // Expression: 3
                                          //  Referenced by: '<Root>/Saturation2'

  real_T Saturation2_LowerSat;         // Expression: -3
                                          //  Referenced by: '<Root>/Saturation2'

  real_T UnitDelay4_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<Root>/Unit Delay4'

  real_T Gain_Gain;                    // Expression: 1/0.1
                                          //  Referenced by: '<Root>/Gain'

  real_T Gain15_Gain;                  // Expression: 0.001
                                          //  Referenced by: '<Root>/Gain15'

  real_T Saturation3_UpperSat;         // Expression: 3
                                          //  Referenced by: '<Root>/Saturation3'

  real_T Saturation3_LowerSat;         // Expression: -3
                                          //  Referenced by: '<Root>/Saturation3'

  real_T Gain2_Gain;                   // Expression: 1
                                          //  Referenced by: '<Root>/Gain2'

  real_T Constant1_Value;              // Expression: 0.25
                                          //  Referenced by: '<Root>/Constant1'

  real_T UnitDelay5_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<Root>/Unit Delay5'

  real_T Gain3_Gain;                   // Expression: 10
                                          //  Referenced by: '<Root>/Gain3'

  real_T Constant2_Value;              // Expression: 0.72
                                          //  Referenced by: '<Root>/Constant2'

  real_T Gain4_Gain;                   // Expression: 0
                                          //  Referenced by: '<Root>/Gain4'

  real_T Constant3_Value;              // Expression: 0
                                          //  Referenced by: '<Root>/Constant3'

  real_T Saturation_UpperSat;          // Expression: 255
                                          //  Referenced by: '<Root>/Saturation'

  real_T Saturation_LowerSat;          // Expression: 0
                                          //  Referenced by: '<Root>/Saturation'

  real32_T UnitDelay7_InitialCondition;
                              // Computed Parameter: UnitDelay7_InitialCondition
                                 //  Referenced by: '<Root>/Unit Delay7'

};

// Real-time Model Data Structure
struct tag_RTM_FuzzyPID_T {
  const char_T * volatile errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_FuzzyPID_T FuzzyPID_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern struct B_FuzzyPID_T FuzzyPID_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern struct DW_FuzzyPID_T FuzzyPID_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void FuzzyPID_initialize(void);
  extern void FuzzyPID_step(void);
  extern void FuzzyPID_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_FuzzyPID_T *const FuzzyPID_M;

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
//  Block '<S4>/InputConversion' : Eliminate redundant data type conversion


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
//  '<Root>' : 'FuzzyPID'
//  '<S1>'   : 'FuzzyPID/Blank Message'
//  '<S2>'   : 'FuzzyPID/Blank Message1'
//  '<S3>'   : 'FuzzyPID/Blank Message2'
//  '<S4>'   : 'FuzzyPID/Fuzzy Logic  Controller1'
//  '<S5>'   : 'FuzzyPID/MATLAB Function'
//  '<S6>'   : 'FuzzyPID/MATLAB Function1'
//  '<S7>'   : 'FuzzyPID/MATLAB Function2'
//  '<S8>'   : 'FuzzyPID/Publish'
//  '<S9>'   : 'FuzzyPID/Publish1'
//  '<S10>'  : 'FuzzyPID/Publish2'
//  '<S11>'  : 'FuzzyPID/Subscribe'
//  '<S12>'  : 'FuzzyPID/Subscribe1'
//  '<S13>'  : 'FuzzyPID/Fuzzy Logic  Controller1/Defuzzify Outputs'
//  '<S14>'  : 'FuzzyPID/Fuzzy Logic  Controller1/Evaluate Rule Antecedents'
//  '<S15>'  : 'FuzzyPID/Fuzzy Logic  Controller1/Evaluate Rule Consequents'
//  '<S16>'  : 'FuzzyPID/Subscribe/Enabled Subsystem'
//  '<S17>'  : 'FuzzyPID/Subscribe1/Enabled Subsystem'

#endif                                 // RTW_HEADER_FuzzyPID_h_

//
// File trailer for generated code.
//
// [EOF]
//
