//
// File: PID_identification.cpp
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
#include "PID_identification.h"
#include <math.h>

extern "C" {

#include "rt_nonfinite.h"

}
#include "rtwtypes.h"
#include "PID_identification_types.h"

// Block signals (default storage)
B_PID_identification_T PID_identification_B;

// Block states (default storage)
DW_PID_identification_T PID_identification_DW;

// Real-time model
RT_MODEL_PID_identification_T PID_identification_M_ =
  RT_MODEL_PID_identification_T();
RT_MODEL_PID_identification_T *const PID_identification_M =
  &PID_identification_M_;

// Model step function
void PID_identification_step(void)
{
  SL_Bus_PID_identification_std_msgs_Int16 rtb_BusAssignment;
  real32_T tmp;
  boolean_T b_varargout_1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S4>/SourceBlock'
  b_varargout_1 = Sub_PID_identification_3.getLatestMessage
    (&PID_identification_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S5>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S5>/In1'
    PID_identification_B.In1 = PID_identification_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S4>/SourceBlock'
  // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   UnitDelay: '<Root>/Unit Delay'

  PID_identification_DW.UnitDelay_DSTATE += PID_identification_B.In1.Axes[7] *
    10.0F;

  // Saturate: '<Root>/Saturation'
  if (PID_identification_DW.UnitDelay_DSTATE >
      PID_identification_P.Saturation_UpperSat) {
    // MATLAB Function: '<Root>/MATLAB Function'
    PID_identification_DW.UnitDelay_DSTATE =
      PID_identification_P.Saturation_UpperSat;
  } else if (PID_identification_DW.UnitDelay_DSTATE <
             PID_identification_P.Saturation_LowerSat) {
    // MATLAB Function: '<Root>/MATLAB Function'
    PID_identification_DW.UnitDelay_DSTATE =
      PID_identification_P.Saturation_LowerSat;
  }

  // End of Saturate: '<Root>/Saturation'

  // DataTypeConversion: '<Root>/Data Type Conversion' incorporates:
  //   UnitDelay: '<Root>/Unit Delay'

  tmp = static_cast<real32_T>(floor(static_cast<real_T>
    (PID_identification_DW.UnitDelay_DSTATE)));
  if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
    tmp = 0.0F;
  } else {
    tmp = static_cast<real32_T>(fmod(static_cast<real_T>(tmp), 65536.0));
  }

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion'

  rtb_BusAssignment.Data = static_cast<int16_T>(tmp < 0.0F ? static_cast<int32_T>
    (static_cast<int16_T>(-static_cast<int16_T>(static_cast<uint16_T>(-tmp)))) :
    static_cast<int32_T>(static_cast<int16_T>(static_cast<uint16_T>(tmp))));

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_PID_identification_2.publish(&rtb_BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void PID_identification_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    char_T b_zeroDelimTopic_0[6];
    char_T b_zeroDelimTopic[5];
    static const char_T tmp[5] = { '/', 'c', 't', 'r', 'l' };

    // InitializeConditions for MATLAB Function: '<Root>/MATLAB Function' incorporates:
    //   UnitDelay: '<Root>/Unit Delay'

    PID_identification_DW.UnitDelay_DSTATE =
      PID_identification_P.UnitDelay_InitialCondition;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S5>/In1' incorporates:
    //   Outport: '<S5>/Out1'

    PID_identification_B.In1 = PID_identification_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'

    // Start for MATLABSystem: '<S4>/SourceBlock'
    PID_identification_DW.obj_d.matlabCodegenIsDeleted = false;
    PID_identification_DW.obj_d.isInitialized = 1;
    b_zeroDelimTopic[0] = '/';
    b_zeroDelimTopic[1] = 'j';
    b_zeroDelimTopic[2] = 'o';
    b_zeroDelimTopic[3] = 'y';
    b_zeroDelimTopic[4] = '\x00';
    Sub_PID_identification_3.createSubscriber(&b_zeroDelimTopic[0], 1);
    PID_identification_DW.obj_d.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    PID_identification_DW.obj.matlabCodegenIsDeleted = false;
    PID_identification_DW.obj.isInitialized = 1;
    for (int32_T i = 0; i < 5; i++) {
      b_zeroDelimTopic_0[i] = tmp[i];
    }

    b_zeroDelimTopic_0[5] = '\x00';
    Pub_PID_identification_2.createPublisher(&b_zeroDelimTopic_0[0], 1);
    PID_identification_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'
  }
}

// Model terminate function
void PID_identification_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  if (!PID_identification_DW.obj_d.matlabCodegenIsDeleted) {
    PID_identification_DW.obj_d.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S4>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  if (!PID_identification_DW.obj.matlabCodegenIsDeleted) {
    PID_identification_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
