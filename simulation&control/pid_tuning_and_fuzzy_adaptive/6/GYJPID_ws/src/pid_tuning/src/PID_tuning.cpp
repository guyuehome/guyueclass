//
// File: PID_tuning.cpp
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
#include "PID_tuning.h"
#include "rtwtypes.h"
#include "PID_tuning_private.h"
#include <math.h>

extern "C" {

#include "rt_nonfinite.h"

}
#include "PID_tuning_types.h"

// Block signals (default storage)
B_PID_tuning_T PID_tuning_B;

// Block states (default storage)
DW_PID_tuning_T PID_tuning_DW;

// Real-time model
RT_MODEL_PID_tuning_T PID_tuning_M_ = RT_MODEL_PID_tuning_T();
RT_MODEL_PID_tuning_T *const PID_tuning_M = &PID_tuning_M_;
real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
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

// Model step function
void PID_tuning_step(void)
{
  SL_Bus_PID_tuning_std_msgs_Float32 b_varargout_2;
  SL_Bus_PID_tuning_std_msgs_Float32 rtb_BusAssignment1;
  SL_Bus_PID_tuning_std_msgs_Int16 rtb_BusAssignment;
  real_T rtb_Add;
  real_T u0;
  boolean_T b_varargout_1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S10>/SourceBlock'
  b_varargout_1 = Sub_PID_tuning_3.getLatestMessage(&PID_tuning_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S10>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S12>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S12>/In1'
    PID_tuning_B.In1 = PID_tuning_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S10>/SourceBlock'
  // End of Outputs for SubSystem: '<S10>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'
  for (int32_T i = 0; i < 128; i++) {
    // DataTypeConversion: '<Root>/Data Type Conversion7'
    PID_tuning_B.DataTypeConversion7[i] = PID_tuning_B.In1.Axes[i];

    // DataTypeConversion: '<Root>/Data Type Conversion8'
    PID_tuning_B.DataTypeConversion8[i] = PID_tuning_B.In1.Buttons[i];
  }

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   UnitDelay: '<Root>/Unit Delay'
  //   UnitDelay: '<Root>/Unit Delay1'
  //   UnitDelay: '<Root>/Unit Delay2'
  //   UnitDelay: '<Root>/Unit Delay3'
  //   UnitDelay: '<Root>/Unit Delay6'

  if (PID_tuning_B.DataTypeConversion8[0] == 1) {
    PID_tuning_DW.UnitDelay3_DSTATE = 0.0;
    PID_tuning_DW.UnitDelay6_DSTATE = 1.0;
    PID_tuning_DW.UnitDelay2_DSTATE = 0.4;
    PID_tuning_DW.UnitDelay1_DSTATE = 1.25;
    PID_tuning_DW.UnitDelay_DSTATE = 0.0;
  } else if (PID_tuning_B.DataTypeConversion8[1] == 1) {
    PID_tuning_DW.UnitDelay3_DSTATE = 0.0;
    PID_tuning_DW.UnitDelay6_DSTATE = 1.0;
    PID_tuning_DW.UnitDelay2_DSTATE = 0.25;
    PID_tuning_DW.UnitDelay1_DSTATE = 0.72;
    PID_tuning_DW.UnitDelay_DSTATE = 0.0;
  } else if (PID_tuning_B.DataTypeConversion8[2] == 1) {
    PID_tuning_DW.UnitDelay3_DSTATE = 0.0;
    PID_tuning_DW.UnitDelay6_DSTATE = 1.0;
    PID_tuning_DW.UnitDelay2_DSTATE = 0.64;
    PID_tuning_DW.UnitDelay1_DSTATE = 0.98;
    PID_tuning_DW.UnitDelay_DSTATE = 0.1;
  } else {
    PID_tuning_DW.UnitDelay3_DSTATE += PID_tuning_B.DataTypeConversion7[7] *
      50.0;
    PID_tuning_DW.UnitDelay6_DSTATE *= rt_powd_snf(10.0,
      PID_tuning_B.DataTypeConversion7[6]);
    PID_tuning_DW.UnitDelay2_DSTATE += PID_tuning_B.DataTypeConversion7[0] *
      PID_tuning_DW.UnitDelay6_DSTATE;
    PID_tuning_DW.UnitDelay1_DSTATE += PID_tuning_B.DataTypeConversion7[1] *
      PID_tuning_DW.UnitDelay6_DSTATE;
    PID_tuning_DW.UnitDelay_DSTATE += PID_tuning_B.DataTypeConversion7[3] *
      PID_tuning_DW.UnitDelay6_DSTATE;
  }

  // End of MATLAB Function: '<Root>/MATLAB Function'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S11>/SourceBlock'
  b_varargout_1 = Sub_PID_tuning_18.getLatestMessage(&b_varargout_2);

  // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S13>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S13>/In1'
    PID_tuning_B.In1_i = b_varargout_2;
  }

  // End of MATLABSystem: '<S11>/SourceBlock'
  // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // Saturate: '<Root>/Saturation1'
  if (PID_tuning_DW.UnitDelay3_DSTATE > PID_tuning_P.Saturation1_UpperSat) {
    PID_tuning_DW.UnitDelay3_DSTATE = PID_tuning_P.Saturation1_UpperSat;
  } else if (PID_tuning_DW.UnitDelay3_DSTATE < PID_tuning_P.Saturation1_LowerSat)
  {
    PID_tuning_DW.UnitDelay3_DSTATE = PID_tuning_P.Saturation1_LowerSat;
  }

  // End of Saturate: '<Root>/Saturation1'

  // MATLAB Function: '<Root>/MATLAB Function2'
  if (!(PID_tuning_B.In1_i.Data < 0.0F)) {
    PID_tuning_DW.UnitDelay7_DSTATE = PID_tuning_B.In1_i.Data;
  }

  // End of MATLAB Function: '<Root>/MATLAB Function2'

  // Sum: '<Root>/Add' incorporates:
  //   UnitDelay: '<Root>/Unit Delay3'

  rtb_Add = PID_tuning_DW.UnitDelay3_DSTATE - PID_tuning_DW.UnitDelay7_DSTATE;

  // Sum: '<Root>/Add2' incorporates:
  //   UnitDelay: '<Root>/Unit Delay5'

  PID_tuning_DW.UnitDelay5_DSTATE += rtb_Add;

  // Switch: '<Root>/Switch' incorporates:
  //   Constant: '<Root>/Constant'
  //   Gain: '<Root>/Gain1'
  //   UnitDelay: '<Root>/Unit Delay3'
  //   UnitDelay: '<Root>/Unit Delay5'

  if (PID_tuning_DW.UnitDelay3_DSTATE != 0.0) {
    u0 = PID_tuning_P.Gain1_Gain * PID_tuning_DW.UnitDelay5_DSTATE;
  } else {
    u0 = PID_tuning_P.Constant_Value_bm;
  }

  // End of Switch: '<Root>/Switch'

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Gain: '<Root>/Gain'
  //   Sum: '<Root>/Add1'
  //   UnitDelay: '<Root>/Unit Delay'
  //   UnitDelay: '<Root>/Unit Delay1'
  //   UnitDelay: '<Root>/Unit Delay2'
  //   UnitDelay: '<Root>/Unit Delay4'

  u0 = (rtb_Add - PID_tuning_DW.UnitDelay4_DSTATE) * PID_tuning_P.Gain_Gain *
    PID_tuning_DW.UnitDelay_DSTATE + (PID_tuning_DW.UnitDelay2_DSTATE * rtb_Add
    + PID_tuning_DW.UnitDelay1_DSTATE * u0);

  // Saturate: '<Root>/Saturation'
  if (u0 > PID_tuning_P.Saturation_UpperSat) {
    u0 = PID_tuning_P.Saturation_UpperSat;
  } else if (u0 < PID_tuning_P.Saturation_LowerSat) {
    u0 = PID_tuning_P.Saturation_LowerSat;
  }

  // End of Saturate: '<Root>/Saturation'

  // DataTypeConversion: '<Root>/Data Type Conversion'
  u0 = floor(u0);
  if (rtIsNaN(u0) || rtIsInf(u0)) {
    u0 = 0.0;
  } else {
    u0 = fmod(u0, 65536.0);
  }

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion'

  rtb_BusAssignment.Data = static_cast<int16_T>(u0 < 0.0 ? static_cast<int32_T>(
    static_cast<int16_T>(-static_cast<int16_T>(static_cast<uint16_T>(-u0)))) :
    static_cast<int32_T>(static_cast<int16_T>(static_cast<uint16_T>(u0))));

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S7>/SinkBlock'
  Pub_PID_tuning_2.publish(&rtb_BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // BusAssignment: '<Root>/Bus Assignment1' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion1'
  //   UnitDelay: '<Root>/Unit Delay3'

  rtb_BusAssignment1.Data = static_cast<real32_T>
    (PID_tuning_DW.UnitDelay3_DSTATE);

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_PID_tuning_34.publish(&rtb_BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish1'

  // BusAssignment: '<Root>/Bus Assignment2' incorporates:
  //   Constant: '<S3>/Constant'
  //   UnitDelay: '<Root>/Unit Delay'
  //   UnitDelay: '<Root>/Unit Delay1'
  //   UnitDelay: '<Root>/Unit Delay2'
  //   UnitDelay: '<Root>/Unit Delay6'

  PID_tuning_B.BusAssignment2 = PID_tuning_P.Constant_Value_b;
  PID_tuning_B.BusAssignment2.Position.X = PID_tuning_DW.UnitDelay2_DSTATE;
  PID_tuning_B.BusAssignment2.Position.Y = PID_tuning_DW.UnitDelay1_DSTATE;
  PID_tuning_B.BusAssignment2.Position.Z = PID_tuning_DW.UnitDelay_DSTATE;
  PID_tuning_B.BusAssignment2.Orientation.X = PID_tuning_DW.UnitDelay6_DSTATE;

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_PID_tuning_37.publish(&PID_tuning_B.BusAssignment2);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // Update for MATLAB Function: '<Root>/MATLAB Function2' incorporates:
  //   UnitDelay: '<Root>/Unit Delay7'

  PID_tuning_DW.UnitDelay7_DSTATE = PID_tuning_B.In1_i.Data;

  // Update for UnitDelay: '<Root>/Unit Delay4'
  PID_tuning_DW.UnitDelay4_DSTATE = rtb_Add;
}

// Model initialize function
void PID_tuning_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    char_T b_zeroDelimTopic_0[9];
    char_T b_zeroDelimTopic_2[8];
    char_T b_zeroDelimTopic_1[6];
    char_T b_zeroDelimTopic[5];
    static const char_T tmp[8] = { '/', 'm', 'o', 't', 'o', 'r', 'n', 'e' };

    static const char_T tmp_0[5] = { '/', 'c', 't', 'r', 'l' };

    static const char_T tmp_1[5] = { '/', 'n', 'd', 'e', 's' };

    static const char_T tmp_2[7] = { '/', 'p', 'a', 'r', 'a', 'm', 's' };

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay3'
    PID_tuning_DW.UnitDelay3_DSTATE = PID_tuning_P.UnitDelay3_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay2'
    PID_tuning_DW.UnitDelay2_DSTATE = PID_tuning_P.UnitDelay2_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay1'
    PID_tuning_DW.UnitDelay1_DSTATE = PID_tuning_P.UnitDelay1_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay'
    PID_tuning_DW.UnitDelay_DSTATE = PID_tuning_P.UnitDelay_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay6'
    PID_tuning_DW.UnitDelay6_DSTATE = PID_tuning_P.UnitDelay6_InitialCondition;

    // InitializeConditions for MATLAB Function: '<Root>/MATLAB Function2' incorporates:
    //   UnitDelay: '<Root>/Unit Delay7'

    PID_tuning_DW.UnitDelay7_DSTATE = PID_tuning_P.UnitDelay7_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay5'
    PID_tuning_DW.UnitDelay5_DSTATE = PID_tuning_P.UnitDelay5_InitialCondition;

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay4'
    PID_tuning_DW.UnitDelay4_DSTATE = PID_tuning_P.UnitDelay4_InitialCondition;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S10>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S12>/In1' incorporates:
    //   Outport: '<S12>/Out1'

    PID_tuning_B.In1 = PID_tuning_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S10>/Enabled Subsystem'

    // Start for MATLABSystem: '<S10>/SourceBlock'
    PID_tuning_DW.obj_d.matlabCodegenIsDeleted = false;
    PID_tuning_DW.obj_d.isInitialized = 1;
    b_zeroDelimTopic[0] = '/';
    b_zeroDelimTopic[1] = 'j';
    b_zeroDelimTopic[2] = 'o';
    b_zeroDelimTopic[3] = 'y';
    b_zeroDelimTopic[4] = '\x00';
    Sub_PID_tuning_3.createSubscriber(&b_zeroDelimTopic[0], 1);
    PID_tuning_DW.obj_d.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S13>/In1' incorporates:
    //   Outport: '<S13>/Out1'

    PID_tuning_B.In1_i = PID_tuning_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem'

    // Start for MATLABSystem: '<S11>/SourceBlock'
    PID_tuning_DW.obj_el.matlabCodegenIsDeleted = false;
    PID_tuning_DW.obj_el.isInitialized = 1;
    for (int32_T i = 0; i < 8; i++) {
      b_zeroDelimTopic_0[i] = tmp[i];
    }

    b_zeroDelimTopic_0[8] = '\x00';
    Sub_PID_tuning_18.createSubscriber(&b_zeroDelimTopic_0[0], 1);
    PID_tuning_DW.obj_el.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    PID_tuning_DW.obj_j.matlabCodegenIsDeleted = false;
    PID_tuning_DW.obj_j.isInitialized = 1;
    for (int32_T i = 0; i < 5; i++) {
      b_zeroDelimTopic_1[i] = tmp_0[i];
    }

    b_zeroDelimTopic_1[5] = '\x00';
    Pub_PID_tuning_2.createPublisher(&b_zeroDelimTopic_1[0], 1);
    PID_tuning_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    PID_tuning_DW.obj_e.matlabCodegenIsDeleted = false;
    PID_tuning_DW.obj_e.isInitialized = 1;
    for (int32_T i = 0; i < 5; i++) {
      b_zeroDelimTopic_1[i] = tmp_1[i];
    }

    b_zeroDelimTopic_1[5] = '\x00';
    Pub_PID_tuning_34.createPublisher(&b_zeroDelimTopic_1[0], 1);
    PID_tuning_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    PID_tuning_DW.obj.matlabCodegenIsDeleted = false;
    PID_tuning_DW.obj.isInitialized = 1;
    for (int32_T i = 0; i < 7; i++) {
      b_zeroDelimTopic_2[i] = tmp_2[i];
    }

    b_zeroDelimTopic_2[7] = '\x00';
    Pub_PID_tuning_37.createPublisher(&b_zeroDelimTopic_2[0], 1);
    PID_tuning_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'
  }
}

// Model terminate function
void PID_tuning_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S10>/SourceBlock'
  if (!PID_tuning_DW.obj_d.matlabCodegenIsDeleted) {
    PID_tuning_DW.obj_d.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S10>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S11>/SourceBlock'
  if (!PID_tuning_DW.obj_el.matlabCodegenIsDeleted) {
    PID_tuning_DW.obj_el.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S11>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  if (!PID_tuning_DW.obj_j.matlabCodegenIsDeleted) {
    PID_tuning_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S7>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  if (!PID_tuning_DW.obj_e.matlabCodegenIsDeleted) {
    PID_tuning_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S8>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish1'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  if (!PID_tuning_DW.obj.matlabCodegenIsDeleted) {
    PID_tuning_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S9>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish2'
}

//
// File trailer for generated code.
//
// [EOF]
//
