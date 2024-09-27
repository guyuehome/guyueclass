//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: turtletrajectory.cpp
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

// Block signals (default storage)
B_turtletrajectory_T turtletrajectory_B;

// Block states (default storage)
DW_turtletrajectory_T turtletrajectory_DW;

// Real-time model
RT_MODEL_turtletrajectory_T turtletrajectory_M_;
RT_MODEL_turtletrajectory_T *const turtletrajectory_M = &turtletrajectory_M_;

// Forward declaration for local functions
static void matlabCodegenHandle_matlabCod_e(robotics_slros_internal_blo_e_T *obj);
static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj);
static void rate_scheduler(void);

//
//   This function updates active task flag for each subrate.
// The function is called at model base rate, hence the
// generated code self-manages all its subrates.
//
static void rate_scheduler(void)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (turtletrajectory_M->Timing.TaskCounters.TID[1])++;
  if ((turtletrajectory_M->Timing.TaskCounters.TID[1]) > 99) {// Sample time: [1.0s, 0.0s] 
    turtletrajectory_M->Timing.TaskCounters.TID[1] = 0;
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2((real_T)u0_0, (real_T)u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  boolean_T yEq;
  real_T q;
  y = u0;
  if (rtIsNaN(u0) || rtIsInf(u0) || (rtIsNaN(u1) || rtIsInf(u1))) {
    if (u1 != 0.0) {
      y = (rtNaN);
    }
  } else if (u0 == 0.0) {
    y = u1 * 0.0;
  } else {
    if (u1 != 0.0) {
      y = fmod(u0, u1);
      yEq = (y == 0.0);
      if ((!yEq) && (u1 > floor(u1))) {
        q = fabs(u0 / u1);
        yEq = (fabs(q - floor(q + 0.5)) <= DBL_EPSILON * q);
      }

      if (yEq) {
        y = u1 * 0.0;
      } else {
        if ((u0 < 0.0) != (u1 < 0.0)) {
          y += u1;
        }
      }
    }
  }

  return y;
}

static void matlabCodegenHandle_matlabCod_e(robotics_slros_internal_blo_e_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void turtletrajectory_step(void)
{
  SL_Bus_turtletrajectory_turtlesim_Pose b_varargout_2;
  boolean_T b_varargout_1;
  real_T rtb_ld_g;
  real_T rtb_Sum1;
  real_T rtb_ld;
  if (turtletrajectory_M->Timing.TaskCounters.TID[1] == 0) {
    // S-Function (fcncallgen): '<S2>/Function-Call Generator' incorporates:
    //   SubSystem: '<S2>/Function-Call Subsystem'

    // SignalConversion: '<S10>/Out' incorporates:
    //   Constant: '<S10>/Vector'
    //   MultiPortSwitch: '<S10>/Output'
    //   UnitDelay: '<S11>/Output'

    turtletrajectory_B.Out =
      turtletrajectory_P.RepeatingSequenceStair_OutValue[turtletrajectory_DW.Output_DSTATE];

    // Sum: '<S12>/FixPt Sum1' incorporates:
    //   Constant: '<S12>/FixPt Constant'
    //   UnitDelay: '<S11>/Output'

    turtletrajectory_DW.Output_DSTATE = (uint8_T)((uint32_T)
      turtletrajectory_DW.Output_DSTATE + turtletrajectory_P.FixPtConstant_Value);

    // Switch: '<S13>/FixPt Switch' incorporates:
    //   Constant: '<S13>/Constant'
    //   UnitDelay: '<S11>/Output'

    if (turtletrajectory_DW.Output_DSTATE >
        turtletrajectory_P.LimitedCounter_uplimit) {
      turtletrajectory_DW.Output_DSTATE = turtletrajectory_P.Constant_Value_c;
    }

    // End of Switch: '<S13>/FixPt Switch'

    // SignalConversion: '<S8>/OutportBuffer_InsertedFor_vx_at_inport_0' incorporates:
    //   Constant: '<S8>/Constant'

    turtletrajectory_B.OutportBuffer_InsertedFor_vx_at =
      turtletrajectory_P.Constant_Value_b;

    // End of Outputs for S-Function (fcncallgen): '<S2>/Function-Call Generator' 
  }

  // Sum: '<S14>/Add' incorporates:
  //   UnitDelay: '<S14>/Unit Delay'

  turtletrajectory_DW.UnitDelay_DSTATE += turtletrajectory_B.Out;

  // Gain: '<S14>/Gain' incorporates:
  //   UnitDelay: '<S14>/Unit Delay'

  rtb_Sum1 = turtletrajectory_P.Gain_Gain * turtletrajectory_DW.UnitDelay_DSTATE;

  // Sum: '<S15>/Add' incorporates:
  //   Product: '<S9>/Product'
  //   Trigonometry: '<S9>/Sin1'
  //   UnitDelay: '<S15>/Unit Delay'

  turtletrajectory_DW.UnitDelay_DSTATE_c += cos(rtb_Sum1) *
    turtletrajectory_B.OutportBuffer_InsertedFor_vx_at;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S5>/SourceBlock' incorporates:
  //   Inport: '<S18>/In1'

  b_varargout_1 = Sub_turtletrajectory_108.getLatestMessage(&b_varargout_2);

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S18>/Enable'

  if (b_varargout_1) {
    turtletrajectory_B.In1 = b_varargout_2;
  }

  // End of MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Sum: '<S4>/Sum' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion2'
  //   Gain: '<S15>/Gain'
  //   SignalConversion: '<Root>/SigConversion_InsertedFor_Bus Selector_at_outport_0'
  //   UnitDelay: '<S15>/Unit Delay'

  rtb_ld_g = turtletrajectory_P.Gain_Gain_m *
    turtletrajectory_DW.UnitDelay_DSTATE_c - turtletrajectory_B.In1.X;

  // Sum: '<S16>/Add' incorporates:
  //   Product: '<S9>/Product1'
  //   Trigonometry: '<S9>/Sin'
  //   UnitDelay: '<S16>/Unit Delay'

  turtletrajectory_DW.UnitDelay_DSTATE_g +=
    turtletrajectory_B.OutportBuffer_InsertedFor_vx_at * sin(rtb_Sum1);

  // Sum: '<S4>/Sum1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion1'
  //   Gain: '<S16>/Gain'
  //   SignalConversion: '<Root>/SigConversion_InsertedFor_Bus Selector_at_outport_1'
  //   UnitDelay: '<S16>/Unit Delay'

  rtb_Sum1 = turtletrajectory_P.Gain_Gain_i *
    turtletrajectory_DW.UnitDelay_DSTATE_g - turtletrajectory_B.In1.Y;

  // Fcn: '<S4>/Fcn1'
  rtb_ld = rt_atan2d_snf(rtb_Sum1, rtb_ld_g);

  // MATLAB Function: '<S4>/MATLAB Function'
  if (rtb_ld < 0.0) {
    rtb_ld += 6.2831853071795862;
  }

  // Fcn: '<S4>/Fcn'
  rtb_Sum1 = rt_powd_snf(rtb_ld_g, 2.0) + rt_powd_snf(rtb_Sum1, 2.0);

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Constant: '<S1>/Constant'
  //   Constant: '<S4>/Constant'

  turtletrajectory_B.BusAssignment = turtletrajectory_P.Constant_Value;
  turtletrajectory_B.BusAssignment.Linear.X =
    turtletrajectory_P.Constant_Value_m;

  // Fcn: '<S4>/Fcn'
  if (rtb_Sum1 < 0.0) {
    rtb_Sum1 = -sqrt(-rtb_Sum1);
  } else {
    rtb_Sum1 = sqrt(rtb_Sum1);
  }

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Constant: '<S4>/Constant'
  //   Constant: '<S4>/Constant1'
  //   DataTypeConversion: '<S4>/Data Type Conversion'
  //   Fcn: '<S4>/Pure Pursuit'
  //   MATLAB Function: '<S4>/MATLAB Function'
  //   Math: '<S4>/Mod'

  turtletrajectory_B.BusAssignment.Angular.Z = 2.0 *
    turtletrajectory_P.Constant_Value_m * sin(rtb_ld - rt_modd_snf((real_T)
    turtletrajectory_B.In1.Theta, turtletrajectory_P.Constant1_Value)) /
    rtb_Sum1;

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_turtletrajectory_103.publish(&turtletrajectory_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'
  rate_scheduler();
}

// Model initialize function
void turtletrajectory_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // initialize real-time model
  (void) memset((void *)turtletrajectory_M, 0,
                sizeof(RT_MODEL_turtletrajectory_T));

  // block I/O
  (void) memset(((void *) &turtletrajectory_B), 0,
                sizeof(B_turtletrajectory_T));

  // states (dwork)
  (void) memset((void *)&turtletrajectory_DW, 0,
                sizeof(DW_turtletrajectory_T));

  {
    static const char_T tmp[16] = { '/', 't', 'u', 'r', 't', 'l', 'e', '1', '/',
      'c', 'm', 'd', '_', 'v', 'e', 'l' };

    static const char_T tmp_0[13] = { '/', 't', 'u', 'r', 't', 'l', 'e', '1',
      '/', 'p', 'o', 's', 'e' };

    char_T tmp_1[17];
    char_T tmp_2[14];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S5>/SourceBlock'
    turtletrajectory_DW.obj_f.matlabCodegenIsDeleted = true;
    turtletrajectory_DW.obj_f.isInitialized = 0;
    turtletrajectory_DW.obj_f.matlabCodegenIsDeleted = false;
    turtletrajectory_DW.obj_f.isSetupComplete = false;
    turtletrajectory_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_2[i] = tmp_0[i];
    }

    tmp_2[13] = '\x00';
    Sub_turtletrajectory_108.createSubscriber(tmp_2, 1);
    turtletrajectory_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    turtletrajectory_DW.obj.matlabCodegenIsDeleted = true;
    turtletrajectory_DW.obj.isInitialized = 0;
    turtletrajectory_DW.obj.matlabCodegenIsDeleted = false;
    turtletrajectory_DW.obj.isSetupComplete = false;
    turtletrajectory_DW.obj.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      tmp_1[i] = tmp[i];
    }

    tmp_1[16] = '\x00';
    Pub_turtletrajectory_103.createPublisher(tmp_1, 1);
    turtletrajectory_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish'

    // InitializeConditions for UnitDelay: '<S14>/Unit Delay'
    turtletrajectory_DW.UnitDelay_DSTATE =
      turtletrajectory_P.UnitDelay_InitialCondition;

    // InitializeConditions for UnitDelay: '<S15>/Unit Delay'
    turtletrajectory_DW.UnitDelay_DSTATE_c =
      turtletrajectory_P.UnitDelay_InitialCondition_d;

    // InitializeConditions for UnitDelay: '<S16>/Unit Delay'
    turtletrajectory_DW.UnitDelay_DSTATE_g =
      turtletrajectory_P.UnitDelay_InitialCondition_e;

    // SystemInitialize for S-Function (fcncallgen): '<S2>/Function-Call Generator' incorporates:
    //   SubSystem: '<S2>/Function-Call Subsystem'

    // InitializeConditions for UnitDelay: '<S11>/Output'
    turtletrajectory_DW.Output_DSTATE =
      turtletrajectory_P.Output_InitialCondition;

    // SystemInitialize for SignalConversion: '<S8>/OutportBuffer_InsertedFor_vx_at_inport_0' incorporates:
    //   Constant: '<S8>/Constant'

    turtletrajectory_B.OutportBuffer_InsertedFor_vx_at =
      turtletrajectory_P.Constant_Value_b;

    // SystemInitialize for Outport: '<S8>/wz'
    turtletrajectory_B.Out = turtletrajectory_P.wz_Y0;

    // End of SystemInitialize for S-Function (fcncallgen): '<S2>/Function-Call Generator' 

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S5>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S18>/Out1'
    turtletrajectory_B.In1 = turtletrajectory_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S5>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void turtletrajectory_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  matlabCodegenHandle_matlabCod_e(&turtletrajectory_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&turtletrajectory_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
