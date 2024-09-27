//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: turtletrajectory_data.cpp
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
#include "turtletrajectory.h"
#include "turtletrajectory_private.h"

// Block parameters (default storage)
P_turtletrajectory_T turtletrajectory_P = {
  // Mask Parameter: RepeatingSequenceStair_OutValue
  //  Referenced by: '<S10>/Vector'

  { 1.0, 2.0, 3.0, 2.0 },

  // Mask Parameter: LimitedCounter_uplimit
  //  Referenced by: '<S13>/FixPt Switch'

  3U,

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S1>/Constant'

  {
    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Linear

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // Angular
  },

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S18>/Out1'

  {
    0.0F,                              // X
    0.0F,                              // Y
    0.0F,                              // Theta
    0.0F,                              // LinearVelocity
    0.0F                               // AngularVelocity
  },

  // Computed Parameter: Constant_Value_g
  //  Referenced by: '<S5>/Constant'

  {
    0.0F,                              // X
    0.0F,                              // Y
    0.0F,                              // Theta
    0.0F,                              // LinearVelocity
    0.0F                               // AngularVelocity
  },

  // Computed Parameter: wz_Y0
  //  Referenced by: '<S8>/wz'

  0.0,

  // Expression: 2
  //  Referenced by: '<S8>/Constant'

  2.0,

  // Expression: 2
  //  Referenced by: '<S4>/Constant'

  2.0,

  // Expression: 0
  //  Referenced by: '<S14>/Unit Delay'

  0.0,

  // Expression: 0.01
  //  Referenced by: '<S14>/Gain'

  0.01,

  // Expression: 555
  //  Referenced by: '<S15>/Unit Delay'

  555.0,

  // Expression: 0.01
  //  Referenced by: '<S15>/Gain'

  0.01,

  // Expression: 555
  //  Referenced by: '<S16>/Unit Delay'

  555.0,

  // Expression: 0.01
  //  Referenced by: '<S16>/Gain'

  0.01,

  // Expression: 2*pi
  //  Referenced by: '<S4>/Constant1'

  6.2831853071795862,

  // Computed Parameter: Constant_Value_c
  //  Referenced by: '<S13>/Constant'

  0U,

  // Computed Parameter: Output_InitialCondition
  //  Referenced by: '<S11>/Output'

  0U,

  // Computed Parameter: FixPtConstant_Value
  //  Referenced by: '<S12>/FixPt Constant'

  1U
};

//
// File trailer for generated code.
//
// [EOF]
//
