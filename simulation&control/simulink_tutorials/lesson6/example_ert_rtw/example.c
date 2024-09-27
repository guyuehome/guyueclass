/*
 * File: example.c
 *
 * Code generated for Simulink model 'example'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Sat Mar 27 00:40:14 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->8051 Compatible
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "example.h"
#include "example_private.h"

/* External inputs (root inport signals with default storage) */
ExtU_example_T example_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_example_T example_Y;

/* Real-time model */
RT_MODEL_example_T example_M_;
RT_MODEL_example_T *const example_M = &example_M_;

/* Model step function */
void example_step(void)
{
  /* Outport: '<Root>/Out1' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   *  Inport: '<Root>/In3'
   *  Product: '<Root>/Product'
   *  Sum: '<Root>/Sum'
   */
  example_Y.Out1 = (example_P.k * example_U.In1 + example_U.In2) * example_U.In3;
}

/* Model initialize function */
void example_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(example_M, (NULL));

  /* external inputs */
  (void)memset(&example_U, 0, sizeof(ExtU_example_T));

  /* external outputs */
  example_Y.Out1 = 0.0;
}

/* Model terminate function */
void example_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
