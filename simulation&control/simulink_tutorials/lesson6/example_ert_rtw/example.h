/*
 * File: example.h
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

#ifndef RTW_HEADER_example_h_
#define RTW_HEADER_example_h_
#include <string.h>
#include <stddef.h>
#ifndef example_COMMON_INCLUDES_
# define example_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* example_COMMON_INCLUDES_ */

#include "example_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T In1;                          /* '<Root>/In1' */
  real_T In2;                          /* '<Root>/In2' */
  real_T In3;                          /* '<Root>/In3' */
} ExtU_example_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
} ExtY_example_T;

/* Parameters (default storage) */
struct P_example_T_ {
  real_T k;                            /* Variable: k
                                        * Referenced by: '<Root>/Gain'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_example_T {
  const char_T *errorStatus;
};

/* Block parameters (default storage) */
extern P_example_T example_P;

/* External inputs (root inport signals with default storage) */
extern ExtU_example_T example_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_example_T example_Y;

/* Model entry point functions */
extern void example_initialize(void);
extern void example_step(void);
extern void example_terminate(void);

/* Real-time Model object */
extern RT_MODEL_example_T *const example_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'example'
 */
#endif                                 /* RTW_HEADER_example_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
