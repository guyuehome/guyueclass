/*
 * File: LPF.c
 *
 * Code generated for Simulink model 'LPF'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Sat Mar 27 00:44:46 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->8051 Compatible
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LPF.h"
#include "LPF_private.h"

/* Block states (default storage) */
DW_LPF_T LPF_DW;

/* External inputs (root inport signals with default storage) */
ExtU_LPF_T LPF_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_LPF_T LPF_Y;

/* Real-time model */
RT_MODEL_LPF_T LPF_M_;
RT_MODEL_LPF_T *const LPF_M = &LPF_M_;

/* Model step function */
void LPF_step(void)
{
  real_T rtb_Add1;
  real_T tmp;

  /* Sum: '<S1>/Add1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   *  Gain: '<S1>/Gain'
   *  Inport: '<Root>/In1'
   *  Sum: '<S1>/Add'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  rtb_Add1 = ((real_T)LPF_U.In1 - LPF_DW.UnitDelay_DSTATE) * LPF_P.g +
    LPF_DW.UnitDelay_DSTATE;

  /* DataTypeConversion: '<Root>/Data Type Conversion1' */
  tmp = floor(rtb_Add1);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  /* Outport: '<Root>/Out1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion1'
   */
  LPF_Y.Out1 = tmp < 0.0 ? (uint16_T)-(int16_T)(uint16_T)-tmp : (uint16_T)tmp;

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  LPF_DW.UnitDelay_DSTATE = rtb_Add1;
}

/* Model initialize function */
void LPF_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(LPF_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&LPF_DW, 0,
                sizeof(DW_LPF_T));

  /* external inputs */
  LPF_U.In1 = 0U;

  /* external outputs */
  LPF_Y.Out1 = 0U;

  /* InitializeConditions for UnitDelay: '<S1>/Unit Delay' */
  LPF_DW.UnitDelay_DSTATE = LPF_P.UnitDelay_InitialCondition;
}

/* Model terminate function */
void LPF_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
