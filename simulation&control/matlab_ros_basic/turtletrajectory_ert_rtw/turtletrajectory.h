//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: turtletrajectory.h
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
#ifndef RTW_HEADER_turtletrajectory_h_
#define RTW_HEADER_turtletrajectory_h_
#include <float.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef turtletrajectory_COMMON_INCLUDES_
# define turtletrajectory_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // turtletrajectory_COMMON_INCLUDES_

#include "turtletrajectory_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_turtletrajectory_geometry_msgs_Twist BusAssignment;// '<Root>/Bus Assignment' 
  SL_Bus_turtletrajectory_turtlesim_Pose In1;// '<S18>/In1'
  real_T Out;                          // '<S10>/Out'
  real_T OutportBuffer_InsertedFor_vx_at;// '<S8>/Constant'
} B_turtletrajectory_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slros_internal_block_T obj; // '<S3>/SinkBlock'
  robotics_slros_internal_blo_e_T obj_f;// '<S5>/SourceBlock'
  real_T UnitDelay_DSTATE;             // '<S14>/Unit Delay'
  real_T UnitDelay_DSTATE_c;           // '<S15>/Unit Delay'
  real_T UnitDelay_DSTATE_g;           // '<S16>/Unit Delay'
  uint8_T Output_DSTATE;               // '<S11>/Output'
} DW_turtletrajectory_T;

// Parameters (default storage)
struct P_turtletrajectory_T_ {
  real_T RepeatingSequenceStair_OutValue[4];// Mask Parameter: RepeatingSequenceStair_OutValue
                                            //  Referenced by: '<S10>/Vector'

  uint8_T LimitedCounter_uplimit;      // Mask Parameter: LimitedCounter_uplimit
                                       //  Referenced by: '<S13>/FixPt Switch'

  SL_Bus_turtletrajectory_geometry_msgs_Twist Constant_Value;// Computed Parameter: Constant_Value
                                                             //  Referenced by: '<S1>/Constant'

  SL_Bus_turtletrajectory_turtlesim_Pose Out1_Y0;// Computed Parameter: Out1_Y0
                                                 //  Referenced by: '<S18>/Out1'

  SL_Bus_turtletrajectory_turtlesim_Pose Constant_Value_g;// Computed Parameter: Constant_Value_g
                                                          //  Referenced by: '<S5>/Constant'

  real_T wz_Y0;                        // Computed Parameter: wz_Y0
                                       //  Referenced by: '<S8>/wz'

  real_T Constant_Value_b;             // Expression: 2
                                       //  Referenced by: '<S8>/Constant'

  real_T Constant_Value_m;             // Expression: 2
                                       //  Referenced by: '<S4>/Constant'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                       //  Referenced by: '<S14>/Unit Delay'

  real_T Gain_Gain;                    // Expression: 0.01
                                       //  Referenced by: '<S14>/Gain'

  real_T UnitDelay_InitialCondition_d; // Expression: 555
                                       //  Referenced by: '<S15>/Unit Delay'

  real_T Gain_Gain_m;                  // Expression: 0.01
                                       //  Referenced by: '<S15>/Gain'

  real_T UnitDelay_InitialCondition_e; // Expression: 555
                                       //  Referenced by: '<S16>/Unit Delay'

  real_T Gain_Gain_i;                  // Expression: 0.01
                                       //  Referenced by: '<S16>/Gain'

  real_T Constant1_Value;              // Expression: 2*pi
                                       //  Referenced by: '<S4>/Constant1'

  uint8_T Constant_Value_c;            // Computed Parameter: Constant_Value_c
                                       //  Referenced by: '<S13>/Constant'

  uint8_T Output_InitialCondition;     // Computed Parameter: Output_InitialCondition
                                       //  Referenced by: '<S11>/Output'

  uint8_T FixPtConstant_Value;         // Computed Parameter: FixPtConstant_Value
                                       //  Referenced by: '<S12>/FixPt Constant'

};

// Real-time Model Data Structure
struct tag_RTM_turtletrajectory_T {
  const char_T *errorStatus;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_turtletrajectory_T turtletrajectory_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
extern B_turtletrajectory_T turtletrajectory_B;

// Block states (default storage)
extern DW_turtletrajectory_T turtletrajectory_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void turtletrajectory_initialize(void);
  extern void turtletrajectory_step(void);
  extern void turtletrajectory_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_turtletrajectory_T *const turtletrajectory_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S11>/Data Type Propagation' : Unused code path elimination
//  Block '<S12>/FixPt Data Type Duplicate' : Unused code path elimination
//  Block '<S13>/FixPt Data Type Duplicate1' : Unused code path elimination
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<S4>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S4>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion1' : Eliminate redundant data type conversion


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
//  '<Root>' : 'turtletrajectory'
//  '<S1>'   : 'turtletrajectory/Blank Message'
//  '<S2>'   : 'turtletrajectory/Desired Trajectory (Planning)'
//  '<S3>'   : 'turtletrajectory/Publish'
//  '<S4>'   : 'turtletrajectory/Pure Persuit'
//  '<S5>'   : 'turtletrajectory/Subscribe'
//  '<S6>'   : 'turtletrajectory/XY Real'
//  '<S7>'   : 'turtletrajectory/XY Ref'
//  '<S8>'   : 'turtletrajectory/Desired Trajectory (Planning)/Function-Call Subsystem'
//  '<S9>'   : 'turtletrajectory/Desired Trajectory (Planning)/Subsystem3'
//  '<S10>'  : 'turtletrajectory/Desired Trajectory (Planning)/Function-Call Subsystem/Repeating Sequence Stair'
//  '<S11>'  : 'turtletrajectory/Desired Trajectory (Planning)/Function-Call Subsystem/Repeating Sequence Stair/LimitedCounter'
//  '<S12>'  : 'turtletrajectory/Desired Trajectory (Planning)/Function-Call Subsystem/Repeating Sequence Stair/LimitedCounter/Increment Real World'
//  '<S13>'  : 'turtletrajectory/Desired Trajectory (Planning)/Function-Call Subsystem/Repeating Sequence Stair/LimitedCounter/Wrap To Zero'
//  '<S14>'  : 'turtletrajectory/Desired Trajectory (Planning)/Subsystem3/Subsystem'
//  '<S15>'  : 'turtletrajectory/Desired Trajectory (Planning)/Subsystem3/Subsystem1'
//  '<S16>'  : 'turtletrajectory/Desired Trajectory (Planning)/Subsystem3/Subsystem2'
//  '<S17>'  : 'turtletrajectory/Pure Persuit/MATLAB Function'
//  '<S18>'  : 'turtletrajectory/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_turtletrajectory_h_

//
// File trailer for generated code.
//
// [EOF]
//
