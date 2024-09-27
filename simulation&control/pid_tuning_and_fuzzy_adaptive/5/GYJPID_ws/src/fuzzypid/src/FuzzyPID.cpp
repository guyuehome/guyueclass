//
// File: FuzzyPID.cpp
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
#include "FuzzyPID.h"
#include "rtwtypes.h"
#include <string.h>

extern "C" {

#include "rt_nonfinite.h"

}
#include <math.h>
#include "FuzzyPID_types.h"

// Block signals (default storage)
B_FuzzyPID_T FuzzyPID_B;

// Block states (default storage)
DW_FuzzyPID_T FuzzyPID_DW;

// Real-time model
RT_MODEL_FuzzyPID_T FuzzyPID_M_ = RT_MODEL_FuzzyPID_T();
RT_MODEL_FuzzyPID_T *const FuzzyPID_M = &FuzzyPID_M_;

// Forward declaration for local functions
static real_T Fuz_evaluateCommonMembershipFcn(real_T x, const real_T params[2]);
static real_T FuzzyPID_trimf(real_T x, const real_T params[3]);
static real_T F_evaluateCommonMembershipFcn_j(real_T x, const real_T params[2]);
static void F_evaluateCommonMembershipFcn_c(const real_T x[101], const real_T
  params[2], real_T y[101]);
static void FuzzyPID_trimf_b(const real_T x[101], const real_T params[3], real_T
  y[101]);
static void evaluateCommonMembershipFcn_cv(const real_T x[101], const real_T
  params[2], real_T y[101]);

// Function for MATLAB Function: '<S4>/Evaluate Rule Antecedents'
static real_T Fuz_evaluateCommonMembershipFcn(real_T x, const real_T params[2])
{
  real_T y;
  if (params[0] >= params[1]) {
    y = (x <= (params[0] + params[1]) / 2.0);
  } else {
    real_T value;
    y = 0.0;
    if (x <= params[0]) {
      y = 1.0;
    }

    if ((params[0] < x) && (x <= (params[0] + params[1]) / 2.0)) {
      value = 1.0 / (params[0] - params[1]) * (x - params[0]);
      y = 1.0 - 2.0 * value * value;
    }

    if (((params[0] + params[1]) / 2.0 < x) && (x <= params[1])) {
      value = 1.0 / (params[0] - params[1]) * (params[1] - x);
      y = 2.0 * value * value;
    }

    if (params[1] <= x) {
      y = 0.0;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S4>/Evaluate Rule Antecedents'
static real_T FuzzyPID_trimf(real_T x, const real_T params[3])
{
  real_T y;
  y = 0.0;
  if ((params[0] != params[1]) && (params[0] < x) && (x < params[1])) {
    y = 1.0 / (params[1] - params[0]) * (x - params[0]);
  }

  if ((params[1] != params[2]) && (params[1] < x) && (x < params[2])) {
    y = 1.0 / (params[2] - params[1]) * (params[2] - x);
  }

  if (x == params[1]) {
    y = 1.0;
  }

  return y;
}

// Function for MATLAB Function: '<S4>/Evaluate Rule Antecedents'
static real_T F_evaluateCommonMembershipFcn_j(real_T x, const real_T params[2])
{
  real_T y;
  if (params[0] >= params[1]) {
    y = (x >= (params[0] + params[1]) / 2.0);
  } else {
    real_T value;
    y = 0.0;
    if ((params[0] < x) && (x <= (params[0] + params[1]) / 2.0)) {
      value = 1.0 / (params[1] - params[0]) * (x - params[0]);
      y = 2.0 * value * value;
    }

    if (((params[0] + params[1]) / 2.0 < x) && (x <= params[1])) {
      value = 1.0 / (params[1] - params[0]) * (params[1] - x);
      y = 1.0 - 2.0 * value * value;
    }

    if (params[1] <= x) {
      y = 1.0;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S4>/Evaluate Rule Consequents'
static void F_evaluateCommonMembershipFcn_c(const real_T x[101], const real_T
  params[2], real_T y[101])
{
  real_T x0;
  real_T x1;
  x1 = params[0];
  x0 = params[1];
  if (params[0] >= params[1]) {
    x1 = (params[0] + params[1]) / 2.0;
    for (int32_T i = 0; i < 101; i++) {
      y[i] = (x[i] <= x1);
    }
  } else {
    for (int32_T i = 0; i < 101; i++) {
      real_T value;
      real_T x_0;
      x_0 = x[i];
      y[i] = 0.0;
      if (x_0 <= x1) {
        y[i] = 1.0;
      }

      if ((x1 < x_0) && (x_0 <= (x1 + x0) / 2.0)) {
        value = 1.0 / (x1 - x0) * (x_0 - x1);
        y[i] = 1.0 - 2.0 * value * value;
      }

      if (((x1 + x0) / 2.0 < x_0) && (x_0 <= x0)) {
        value = 1.0 / (x1 - x0) * (x0 - x_0);
        y[i] = 2.0 * value * value;
      }

      if (x0 <= x_0) {
        y[i] = 0.0;
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/Evaluate Rule Consequents'
static void FuzzyPID_trimf_b(const real_T x[101], const real_T params[3], real_T
  y[101])
{
  real_T a;
  real_T b;
  real_T c;
  a = params[0];
  b = params[1];
  c = params[2];
  for (int32_T i = 0; i < 101; i++) {
    real_T x_0;
    x_0 = x[i];
    y[i] = 0.0;
    if ((a != b) && (a < x_0) && (x_0 < b)) {
      y[i] = 1.0 / (b - a) * (x_0 - a);
    }

    if ((b != c) && (b < x_0) && (x_0 < c)) {
      y[i] = 1.0 / (c - b) * (c - x_0);
    }

    if (x_0 == b) {
      y[i] = 1.0;
    }
  }
}

// Function for MATLAB Function: '<S4>/Evaluate Rule Consequents'
static void evaluateCommonMembershipFcn_cv(const real_T x[101], const real_T
  params[2], real_T y[101])
{
  real_T x0;
  real_T x1;
  x0 = params[0];
  x1 = params[1];
  if (params[0] >= params[1]) {
    x0 = (params[0] + params[1]) / 2.0;
    for (int32_T i = 0; i < 101; i++) {
      y[i] = (x[i] >= x0);
    }
  } else {
    for (int32_T i = 0; i < 101; i++) {
      real_T value;
      real_T x_0;
      x_0 = x[i];
      y[i] = 0.0;
      if ((x0 < x_0) && (x_0 <= (x0 + x1) / 2.0)) {
        value = 1.0 / (x1 - x0) * (x_0 - x0);
        y[i] = 2.0 * value * value;
      }

      if (((x0 + x1) / 2.0 < x_0) && (x_0 <= x1)) {
        value = 1.0 / (x1 - x0) * (x1 - x_0);
        y[i] = 1.0 - 2.0 * value * value;
      }

      if (x1 <= x_0) {
        y[i] = 1.0;
      }
    }
  }
}

// Model step function
void FuzzyPID_step(void)
{
  SL_Bus_FuzzyPID_std_msgs_Float32 b_varargout_2;
  SL_Bus_FuzzyPID_std_msgs_Float32 rtb_BusAssignment1;
  SL_Bus_FuzzyPID_std_msgs_Int16 rtb_BusAssignment;
  boolean_T b_varargout_1;
  static const real_T d[3] = { -3.0, -2.0, 0.0 };

  static const real_T c[3] = { 0.0, 2.0, 3.0 };

  static const int8_T b[98] = { 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3,
    3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5,
    6, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2, 3,
    4, 5, 6, 7 };

  static const real_T b_0[3] = { -0.3, -0.2, 0.0 };

  static const real_T c_0[3] = { -0.3, -0.1, 0.1 };

  static const real_T d_0[3] = { -0.1, 0.1, 0.3 };

  static const real_T e[3] = { 0.0, 0.2, 0.3 };

  static const real_T f[2] = { -0.06, -0.02 };

  static const real_T g[3] = { -0.06, -0.04, 0.0 };

  static const real_T h[3] = { -0.06, -0.02, 0.02 };

  static const real_T i_0[3] = { 0.0, 0.04, 0.06 };

  static const int8_T l[147] = { 7, 7, 6, 6, 5, 4, 4, 7, 7, 6, 5, 5, 4, 3, 6, 6,
    6, 5, 4, 3, 3, 6, 6, 5, 4, 3, 2, 2, 5, 5, 4, 3, 3, 2, 2, 5, 4, 3, 2, 2, 2, 1,
    4, 4, 2, 2, 2, 1, 1, 1, 1, 2, 2, 3, 4, 4, 1, 1, 2, 3, 3, 4, 4, 1, 2, 3, 3, 4,
    5, 5, 2, 2, 3, 4, 5, 6, 6, 2, 3, 4, 5, 5, 6, 7, 4, 4, 5, 5, 6, 7, 7, 4, 4, 5,
    6, 6, 7, 7, 5, 3, 1, 1, 1, 2, 5, 5, 3, 1, 2, 2, 3, 4, 4, 3, 2, 2, 3, 3, 4, 4,
    3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 7, 5, 5, 5, 5, 5, 7, 7, 6, 6, 6, 5, 5,
    7 };

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S12>/SourceBlock'
  b_varargout_1 = Sub_FuzzyPID_18.getLatestMessage(&b_varargout_2);

  // Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S17>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S17>/In1'
    FuzzyPID_B.In1_i = b_varargout_2;
  }

  // End of MATLABSystem: '<S12>/SourceBlock'
  // End of Outputs for SubSystem: '<S12>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S11>/SourceBlock'
  b_varargout_1 = Sub_FuzzyPID_3.getLatestMessage(&FuzzyPID_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S16>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S16>/In1'
    FuzzyPID_B.In1 = FuzzyPID_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S11>/SourceBlock'
  // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Saturate: '<Root>/Saturation1' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'
  //   UnitDelay: '<Root>/Unit Delay1'

  FuzzyPID_DW.UnitDelay1_DSTATE = FuzzyPID_B.In1.Axes[7] * 50.0F +
    static_cast<real32_T>(FuzzyPID_DW.UnitDelay1_DSTATE);
  if (FuzzyPID_DW.UnitDelay1_DSTATE > FuzzyPID_P.Saturation1_UpperSat) {
    FuzzyPID_DW.UnitDelay1_DSTATE = FuzzyPID_P.Saturation1_UpperSat;
  } else if (FuzzyPID_DW.UnitDelay1_DSTATE < FuzzyPID_P.Saturation1_LowerSat) {
    FuzzyPID_DW.UnitDelay1_DSTATE = FuzzyPID_P.Saturation1_LowerSat;
  }

  // End of Saturate: '<Root>/Saturation1'

  // MATLAB Function: '<Root>/MATLAB Function2'
  if (!(FuzzyPID_B.In1_i.Data < 0.0F)) {
    FuzzyPID_DW.UnitDelay7_DSTATE = FuzzyPID_B.In1_i.Data;
  }

  // End of MATLAB Function: '<Root>/MATLAB Function2'

  // Sum: '<Root>/Add' incorporates:
  //   UnitDelay: '<Root>/Unit Delay1'

  FuzzyPID_B.Add = FuzzyPID_DW.UnitDelay1_DSTATE - FuzzyPID_DW.UnitDelay7_DSTATE;

  // Gain: '<Root>/Gain' incorporates:
  //   Sum: '<Root>/Add1'
  //   UnitDelay: '<Root>/Unit Delay4'

  FuzzyPID_B.Gain = (FuzzyPID_B.Add - FuzzyPID_DW.UnitDelay4_DSTATE) *
    FuzzyPID_P.Gain_Gain;

  // Gain: '<Root>/Gain14'
  FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c = FuzzyPID_P.Gain14_Gain *
    FuzzyPID_B.Add;

  // Saturate: '<Root>/Saturation2'
  if (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c >
      FuzzyPID_P.Saturation2_UpperSat) {
    // Outputs for Atomic SubSystem: '<Root>/Fuzzy Logic  Controller1'
    // SignalConversion generated from: '<S14>/ SFunction ' incorporates:
    //   MATLAB Function: '<S4>/Evaluate Rule Antecedents'

    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c = FuzzyPID_P.Saturation2_UpperSat;

    // End of Outputs for SubSystem: '<Root>/Fuzzy Logic  Controller1'
  } else if (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c <
             FuzzyPID_P.Saturation2_LowerSat) {
    // Outputs for Atomic SubSystem: '<Root>/Fuzzy Logic  Controller1'
    // SignalConversion generated from: '<S14>/ SFunction ' incorporates:
    //   MATLAB Function: '<S4>/Evaluate Rule Antecedents'

    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c = FuzzyPID_P.Saturation2_LowerSat;

    // End of Outputs for SubSystem: '<Root>/Fuzzy Logic  Controller1'
  }

  // End of Saturate: '<Root>/Saturation2'

  // Gain: '<Root>/Gain15'
  FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k = FuzzyPID_P.Gain15_Gain *
    FuzzyPID_B.Gain;

  // Saturate: '<Root>/Saturation3'
  if (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k >
      FuzzyPID_P.Saturation3_UpperSat) {
    // Outputs for Atomic SubSystem: '<Root>/Fuzzy Logic  Controller1'
    // SignalConversion generated from: '<S14>/ SFunction ' incorporates:
    //   MATLAB Function: '<S4>/Evaluate Rule Antecedents'

    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k = FuzzyPID_P.Saturation3_UpperSat;

    // End of Outputs for SubSystem: '<Root>/Fuzzy Logic  Controller1'
  } else if (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k <
             FuzzyPID_P.Saturation3_LowerSat) {
    // Outputs for Atomic SubSystem: '<Root>/Fuzzy Logic  Controller1'
    // SignalConversion generated from: '<S14>/ SFunction ' incorporates:
    //   MATLAB Function: '<S4>/Evaluate Rule Antecedents'

    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k = FuzzyPID_P.Saturation3_LowerSat;

    // End of Outputs for SubSystem: '<Root>/Fuzzy Logic  Controller1'
  }

  // End of Saturate: '<Root>/Saturation3'

  // Outputs for Atomic SubSystem: '<Root>/Fuzzy Logic  Controller1'
  // MATLAB Function: '<S4>/Evaluate Rule Antecedents'
  FuzzyPID_B.area = 0.0;
  FuzzyPID_B.dv3[0] = -3.0;
  FuzzyPID_B.dv3[1] = -1.0;
  FuzzyPID_B.inputMFCache[0] = Fuz_evaluateCommonMembershipFcn
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c, FuzzyPID_B.dv3);
  FuzzyPID_B.inputMFCache[1] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c, d);
  FuzzyPID_B.dv2[0] = -3.0;
  FuzzyPID_B.dv2[1] = -1.0;
  FuzzyPID_B.dv2[2] = 1.0;
  FuzzyPID_B.inputMFCache[2] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c, FuzzyPID_B.dv2);
  FuzzyPID_B.dv2[0] = -2.0;
  FuzzyPID_B.dv2[1] = 0.0;
  FuzzyPID_B.dv2[2] = 2.0;
  FuzzyPID_B.inputMFCache[3] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c, FuzzyPID_B.dv2);
  FuzzyPID_B.dv2[0] = -1.0;
  FuzzyPID_B.dv2[1] = 1.0;
  FuzzyPID_B.dv2[2] = 3.0;
  FuzzyPID_B.inputMFCache[4] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c, FuzzyPID_B.dv2);
  FuzzyPID_B.inputMFCache[5] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c, c);
  FuzzyPID_B.dv3[0] = 1.0;
  FuzzyPID_B.dv3[1] = 3.0;
  FuzzyPID_B.inputMFCache[6] = F_evaluateCommonMembershipFcn_j
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c, FuzzyPID_B.dv3);
  FuzzyPID_B.dv3[0] = -3.0;
  FuzzyPID_B.dv3[1] = -1.0;
  FuzzyPID_B.inputMFCache[7] = Fuz_evaluateCommonMembershipFcn
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k, FuzzyPID_B.dv3);
  FuzzyPID_B.inputMFCache[8] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k, d);
  FuzzyPID_B.dv2[0] = -3.0;
  FuzzyPID_B.dv2[1] = -1.0;
  FuzzyPID_B.dv2[2] = 1.0;
  FuzzyPID_B.inputMFCache[9] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k, FuzzyPID_B.dv2);
  FuzzyPID_B.dv2[0] = -2.0;
  FuzzyPID_B.dv2[1] = 0.0;
  FuzzyPID_B.dv2[2] = 2.0;
  FuzzyPID_B.inputMFCache[10] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k, FuzzyPID_B.dv2);
  FuzzyPID_B.dv2[0] = -1.0;
  FuzzyPID_B.dv2[1] = 1.0;
  FuzzyPID_B.dv2[2] = 3.0;
  FuzzyPID_B.inputMFCache[11] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k, FuzzyPID_B.dv2);
  FuzzyPID_B.inputMFCache[12] = FuzzyPID_trimf
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k, c);
  FuzzyPID_B.dv3[0] = 1.0;
  FuzzyPID_B.dv3[1] = 3.0;
  FuzzyPID_B.inputMFCache[13] = F_evaluateCommonMembershipFcn_j
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k, FuzzyPID_B.dv3);
  for (int32_T i = 0; i < 49; i++) {
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k = FuzzyPID_B.inputMFCache[b[i] -
      1];
    if (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k < 1.0) {
      FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c =
        FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k;
    } else {
      FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c = 1.0;
    }

    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k = FuzzyPID_B.inputMFCache[b[i +
      49] + 6];
    if (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c >
        FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k) {
      FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c =
        FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k;
    }

    FuzzyPID_B.area += FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c;
    FuzzyPID_B.antecedentOutputs[i] = FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c;
  }

  // MATLAB Function: '<S4>/Evaluate Rule Consequents' incorporates:
  //   Constant: '<S4>/Output Sample Points'

  memset(&FuzzyPID_B.aggregatedOutputs[0], 0, 303U * sizeof(real_T));
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID];
  }

  FuzzyPID_B.dv3[0] = -0.3;
  FuzzyPID_B.dv3[1] = -0.1;
  F_evaluateCommonMembershipFcn_c(FuzzyPID_B.dv, FuzzyPID_B.dv3, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, b_0, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 1] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, c_0, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 2] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID];
  }

  FuzzyPID_B.dv2[0] = -0.2;
  FuzzyPID_B.dv2[1] = 0.0;
  FuzzyPID_B.dv2[2] = 0.2;
  FuzzyPID_trimf_b(FuzzyPID_B.dv, FuzzyPID_B.dv2, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 3] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, d_0, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 4] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, e, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 5] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID];
  }

  FuzzyPID_B.dv3[0] = 0.1;
  FuzzyPID_B.dv3[1] = 0.3;
  evaluateCommonMembershipFcn_cv(FuzzyPID_B.dv, FuzzyPID_B.dv3, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 6] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      1];
  }

  F_evaluateCommonMembershipFcn_c(FuzzyPID_B.dv, f, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 7] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      1];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, g, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 8] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      1];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, h, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 9] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      1];
  }

  FuzzyPID_B.dv2[0] = -0.04;
  FuzzyPID_B.dv2[1] = 0.0;
  FuzzyPID_B.dv2[2] = 0.04;
  FuzzyPID_trimf_b(FuzzyPID_B.dv, FuzzyPID_B.dv2, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 10] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      1];
  }

  FuzzyPID_B.dv2[0] = -0.02;
  FuzzyPID_B.dv2[1] = 0.02;
  FuzzyPID_B.dv2[2] = 0.06;
  FuzzyPID_trimf_b(FuzzyPID_B.dv, FuzzyPID_B.dv2, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 11] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      1];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, i_0, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 12] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      1];
  }

  FuzzyPID_B.dv3[0] = 0.02;
  FuzzyPID_B.dv3[1] = 0.06;
  evaluateCommonMembershipFcn_cv(FuzzyPID_B.dv, FuzzyPID_B.dv3, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 13] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      2];
  }

  FuzzyPID_B.dv3[0] = -3.0;
  FuzzyPID_B.dv3[1] = -1.0;
  F_evaluateCommonMembershipFcn_c(FuzzyPID_B.dv, FuzzyPID_B.dv3, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 14] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      2];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, d, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 15] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      2];
  }

  FuzzyPID_B.dv2[0] = -3.0;
  FuzzyPID_B.dv2[1] = -1.0;
  FuzzyPID_B.dv2[2] = 1.0;
  FuzzyPID_trimf_b(FuzzyPID_B.dv, FuzzyPID_B.dv2, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 16] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      2];
  }

  FuzzyPID_B.dv2[0] = -2.0;
  FuzzyPID_B.dv2[1] = 0.0;
  FuzzyPID_B.dv2[2] = 2.0;
  FuzzyPID_trimf_b(FuzzyPID_B.dv, FuzzyPID_B.dv2, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 17] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      2];
  }

  FuzzyPID_B.dv2[0] = -1.0;
  FuzzyPID_B.dv2[1] = 1.0;
  FuzzyPID_B.dv2[2] = 3.0;
  FuzzyPID_trimf_b(FuzzyPID_B.dv, FuzzyPID_B.dv2, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 18] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      2];
  }

  FuzzyPID_trimf_b(FuzzyPID_B.dv, c, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 19] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.dv[outputID] = FuzzyPID_P.OutputSamplePoints_Value[3 * outputID +
      2];
  }

  FuzzyPID_B.dv3[0] = 1.0;
  FuzzyPID_B.dv3[1] = 3.0;
  evaluateCommonMembershipFcn_cv(FuzzyPID_B.dv, FuzzyPID_B.dv3, FuzzyPID_B.dv1);
  for (int32_T outputID = 0; outputID < 101; outputID++) {
    FuzzyPID_B.outputMFCache[21 * outputID + 20] = FuzzyPID_B.dv1[outputID];
  }

  for (int32_T outputID = 0; outputID < 3; outputID++) {
    for (int32_T i = 0; i < 49; i++) {
      for (int32_T sampleID = 0; sampleID < 101; sampleID++) {
        int32_T rtb_TmpSignalConversionAtSFun_0;
        FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c = FuzzyPID_B.outputMFCache
          [((l[49 * outputID + i] + 7 * outputID) + 21 * sampleID) - 1];
        FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k =
          FuzzyPID_B.antecedentOutputs[i];
        FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m =
          FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c;
        b_varargout_1 = rtIsNaN(FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c);
        rtb_TmpSignalConversionAtSFun_0 = 101 * outputID + sampleID;
        FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c =
          FuzzyPID_B.aggregatedOutputs[rtb_TmpSignalConversionAtSFun_0];
        if ((!(FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m >
               FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k)) && (!b_varargout_1))
        {
          FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k =
            FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m;
        }

        if (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c <
            FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k) {
          FuzzyPID_B.aggregatedOutputs[rtb_TmpSignalConversionAtSFun_0] =
            FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k;
        } else {
          FuzzyPID_B.aggregatedOutputs[rtb_TmpSignalConversionAtSFun_0] =
            FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c;
        }
      }
    }
  }

  // End of MATLAB Function: '<S4>/Evaluate Rule Consequents'

  // MATLAB Function: '<S4>/Defuzzify Outputs' incorporates:
  //   Constant: '<S4>/Output Sample Points'
  //   MATLAB Function: '<S4>/Evaluate Rule Antecedents'

  if (FuzzyPID_B.area == 0.0) {
    FuzzyPID_B.defuzzifiedOutputs[0] = 0.0;
    FuzzyPID_B.defuzzifiedOutputs[1] = 0.0;
    FuzzyPID_B.defuzzifiedOutputs[2] = 0.0;
  } else {
    for (int32_T outputID = 0; outputID < 3; outputID++) {
      FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k = 0.0;
      FuzzyPID_B.area = 0.0;
      for (int32_T i = 0; i < 101; i++) {
        FuzzyPID_B.area += FuzzyPID_B.aggregatedOutputs[101 * outputID + i];
      }

      if (FuzzyPID_B.area == 0.0) {
        FuzzyPID_B.defuzzifiedOutputs[outputID] =
          (FuzzyPID_P.OutputSamplePoints_Value[outputID + 300] +
           FuzzyPID_P.OutputSamplePoints_Value[outputID]) / 2.0;
      } else {
        for (int32_T i = 0; i < 101; i++) {
          FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k +=
            FuzzyPID_P.OutputSamplePoints_Value[3 * i + outputID] *
            FuzzyPID_B.aggregatedOutputs[101 * outputID + i];
        }

        FuzzyPID_B.defuzzifiedOutputs[outputID] = 1.0 / FuzzyPID_B.area *
          FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k;
      }
    }
  }

  // End of MATLAB Function: '<S4>/Defuzzify Outputs'
  // End of Outputs for SubSystem: '<Root>/Fuzzy Logic  Controller1'

  // Sum: '<Root>/Add3' incorporates:
  //   Constant: '<Root>/Constant1'
  //   Gain: '<Root>/Gain2'

  FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k = FuzzyPID_P.Gain2_Gain *
    FuzzyPID_B.defuzzifiedOutputs[0] + FuzzyPID_P.Constant1_Value;

  // Sum: '<Root>/Add2' incorporates:
  //   UnitDelay: '<Root>/Unit Delay5'

  FuzzyPID_DW.UnitDelay5_DSTATE += FuzzyPID_B.Add;

  // Sum: '<Root>/Add4' incorporates:
  //   Constant: '<Root>/Constant2'
  //   Gain: '<Root>/Gain3'

  FuzzyPID_B.area = FuzzyPID_P.Gain3_Gain * FuzzyPID_B.defuzzifiedOutputs[1] +
    FuzzyPID_P.Constant2_Value;

  // Sum: '<Root>/Add5' incorporates:
  //   Constant: '<Root>/Constant3'
  //   Gain: '<Root>/Gain4'

  FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c = FuzzyPID_P.Gain4_Gain *
    FuzzyPID_B.defuzzifiedOutputs[2] + FuzzyPID_P.Constant3_Value;

  // Switch: '<Root>/Switch' incorporates:
  //   Constant: '<Root>/Constant'
  //   Gain: '<Root>/Gain1'
  //   UnitDelay: '<Root>/Unit Delay1'
  //   UnitDelay: '<Root>/Unit Delay5'

  if (FuzzyPID_DW.UnitDelay1_DSTATE != 0.0) {
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m = FuzzyPID_P.Gain1_Gain *
      FuzzyPID_DW.UnitDelay5_DSTATE;
  } else {
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m = FuzzyPID_P.Constant_Value_bm;
  }

  // End of Switch: '<Root>/Switch'

  // MATLAB Function: '<Root>/MATLAB Function1'
  FuzzyPID_B.Gain = (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k * FuzzyPID_B.Add
                     + FuzzyPID_B.area *
                     FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m) +
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c * FuzzyPID_B.Gain;

  // Saturate: '<Root>/Saturation'
  if (FuzzyPID_B.Gain > FuzzyPID_P.Saturation_UpperSat) {
    FuzzyPID_B.Gain = FuzzyPID_P.Saturation_UpperSat;
  } else if (FuzzyPID_B.Gain < FuzzyPID_P.Saturation_LowerSat) {
    FuzzyPID_B.Gain = FuzzyPID_P.Saturation_LowerSat;
  }

  // End of Saturate: '<Root>/Saturation'

  // DataTypeConversion: '<Root>/Data Type Conversion'
  FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m = floor(FuzzyPID_B.Gain);
  if (rtIsNaN(FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m) || rtIsInf
      (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m)) {
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m = 0.0;
  } else {
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m = fmod
      (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m, 65536.0);
  }

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion'

  rtb_BusAssignment.Data = static_cast<int16_T>
    (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m < 0.0 ? static_cast<int32_T>(
      static_cast<int16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m)))) : static_cast<int32_T>(
      static_cast<int16_T>(static_cast<uint16_T>
       (FuzzyPID_B.rtb_TmpSignalConversionAtSFun_m))));

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_FuzzyPID_2.publish(&rtb_BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // BusAssignment: '<Root>/Bus Assignment2' incorporates:
  //   Constant: '<S3>/Constant'

  FuzzyPID_B.BusAssignment2 = FuzzyPID_P.Constant_Value_b;
  FuzzyPID_B.BusAssignment2.Position.X =
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_k;
  FuzzyPID_B.BusAssignment2.Position.Y = FuzzyPID_B.area;
  FuzzyPID_B.BusAssignment2.Position.Z =
    FuzzyPID_B.rtb_TmpSignalConversionAtSFun_c;

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S10>/SinkBlock'
  Pub_FuzzyPID_37.publish(&FuzzyPID_B.BusAssignment2);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // BusAssignment: '<Root>/Bus Assignment1' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion1'
  //   UnitDelay: '<Root>/Unit Delay1'

  rtb_BusAssignment1.Data = static_cast<real32_T>(FuzzyPID_DW.UnitDelay1_DSTATE);

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_FuzzyPID_34.publish(&rtb_BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish1'

  // Update for MATLAB Function: '<Root>/MATLAB Function2' incorporates:
  //   UnitDelay: '<Root>/Unit Delay7'

  FuzzyPID_DW.UnitDelay7_DSTATE = FuzzyPID_B.In1_i.Data;

  // Update for UnitDelay: '<Root>/Unit Delay4'
  FuzzyPID_DW.UnitDelay4_DSTATE = FuzzyPID_B.Add;
}

// Model initialize function
void FuzzyPID_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    char_T b_zeroDelimTopic[9];
    char_T b_zeroDelimTopic_2[8];
    char_T b_zeroDelimTopic_1[6];
    char_T b_zeroDelimTopic_0[5];
    static const char_T tmp[8] = { '/', 'm', 'o', 't', 'o', 'r', 'n', 'e' };

    static const char_T tmp_0[5] = { '/', 'c', 't', 'r', 'l' };

    static const char_T tmp_1[7] = { '/', 'p', 'a', 'r', 'a', 'm', 's' };

    static const char_T tmp_2[5] = { '/', 'n', 'd', 'e', 's' };

    // InitializeConditions for MATLAB Function: '<Root>/MATLAB Function2' incorporates:
    //   UnitDelay: '<Root>/Unit Delay7'

    FuzzyPID_DW.UnitDelay7_DSTATE = FuzzyPID_P.UnitDelay7_InitialCondition;

    // InitializeConditions for Saturate: '<Root>/Saturation1' incorporates:
    //   UnitDelay: '<Root>/Unit Delay1'

    FuzzyPID_DW.UnitDelay1_DSTATE = FuzzyPID_P.UnitDelay1_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay4'
    FuzzyPID_DW.UnitDelay4_DSTATE = FuzzyPID_P.UnitDelay4_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay5'
    FuzzyPID_DW.UnitDelay5_DSTATE = FuzzyPID_P.UnitDelay5_InitialCondition;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S12>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S17>/In1' incorporates:
    //   Outport: '<S17>/Out1'

    FuzzyPID_B.In1_i = FuzzyPID_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S12>/Enabled Subsystem'

    // Start for MATLABSystem: '<S12>/SourceBlock'
    FuzzyPID_DW.obj_el.matlabCodegenIsDeleted = false;
    FuzzyPID_DW.obj_el.isInitialized = 1;
    for (int32_T i = 0; i < 8; i++) {
      b_zeroDelimTopic[i] = tmp[i];
    }

    b_zeroDelimTopic[8] = '\x00';
    Sub_FuzzyPID_18.createSubscriber(&b_zeroDelimTopic[0], 1);
    FuzzyPID_DW.obj_el.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S16>/In1' incorporates:
    //   Outport: '<S16>/Out1'

    FuzzyPID_B.In1 = FuzzyPID_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem'

    // Start for MATLABSystem: '<S11>/SourceBlock'
    FuzzyPID_DW.obj_d.matlabCodegenIsDeleted = false;
    FuzzyPID_DW.obj_d.isInitialized = 1;
    b_zeroDelimTopic_0[0] = '/';
    b_zeroDelimTopic_0[1] = 'j';
    b_zeroDelimTopic_0[2] = 'o';
    b_zeroDelimTopic_0[3] = 'y';
    b_zeroDelimTopic_0[4] = '\x00';
    Sub_FuzzyPID_3.createSubscriber(&b_zeroDelimTopic_0[0], 1);
    FuzzyPID_DW.obj_d.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    FuzzyPID_DW.obj_j.matlabCodegenIsDeleted = false;
    FuzzyPID_DW.obj_j.isInitialized = 1;
    for (int32_T i = 0; i < 5; i++) {
      b_zeroDelimTopic_1[i] = tmp_0[i];
    }

    b_zeroDelimTopic_1[5] = '\x00';
    Pub_FuzzyPID_2.createPublisher(&b_zeroDelimTopic_1[0], 1);
    FuzzyPID_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    FuzzyPID_DW.obj.matlabCodegenIsDeleted = false;
    FuzzyPID_DW.obj.isInitialized = 1;
    for (int32_T i = 0; i < 7; i++) {
      b_zeroDelimTopic_2[i] = tmp_1[i];
    }

    b_zeroDelimTopic_2[7] = '\x00';
    Pub_FuzzyPID_37.createPublisher(&b_zeroDelimTopic_2[0], 1);
    FuzzyPID_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    FuzzyPID_DW.obj_e.matlabCodegenIsDeleted = false;
    FuzzyPID_DW.obj_e.isInitialized = 1;
    for (int32_T i = 0; i < 5; i++) {
      b_zeroDelimTopic_1[i] = tmp_2[i];
    }

    b_zeroDelimTopic_1[5] = '\x00';
    Pub_FuzzyPID_34.createPublisher(&b_zeroDelimTopic_1[0], 1);
    FuzzyPID_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'
  }
}

// Model terminate function
void FuzzyPID_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S12>/SourceBlock'
  if (!FuzzyPID_DW.obj_el.matlabCodegenIsDeleted) {
    FuzzyPID_DW.obj_el.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S12>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S11>/SourceBlock'
  if (!FuzzyPID_DW.obj_d.matlabCodegenIsDeleted) {
    FuzzyPID_DW.obj_d.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S11>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  if (!FuzzyPID_DW.obj_j.matlabCodegenIsDeleted) {
    FuzzyPID_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S8>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  if (!FuzzyPID_DW.obj.matlabCodegenIsDeleted) {
    FuzzyPID_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S10>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  if (!FuzzyPID_DW.obj_e.matlabCodegenIsDeleted) {
    FuzzyPID_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S9>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
