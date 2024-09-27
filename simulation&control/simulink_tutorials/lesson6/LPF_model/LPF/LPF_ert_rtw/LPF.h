/*
 * File: LPF.h
 *
 * Code generated for Simulink model 'LPF'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
 * C/C++ source code generated on : Thu Feb 13 09:10:53 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->8051 Compatible
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_LPF_h_
#define RTW_HEADER_LPF_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef LPF_COMMON_INCLUDES_
# define LPF_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* LPF_COMMON_INCLUDES_ */

#include "LPF_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<S1>/Unit Delay' */
} DW_LPF_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  uint16_T In1;                        /* '<Root>/In1' */
} ExtU_LPF_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint16_T Out1;                       /* '<Root>/Out1' */
} ExtY_LPF_T;

/* Parameters (default storage) */
struct P_LPF_T_ {
  real_T g;                            /* Variable: g
                                        * Referenced by: '<S1>/Gain'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<S1>/Unit Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_LPF_T {
  const char_T *errorStatus;
};

/* Block parameters (default storage) */
extern P_LPF_T LPF_P;

/* Block states (default storage) */
extern DW_LPF_T LPF_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_LPF_T LPF_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_LPF_T LPF_Y;

/* Model entry point functions */
extern void LPF_initialize(void);
extern void LPF_step(void);
extern void LPF_terminate(void);

/* Real-time Model object */
extern RT_MODEL_LPF_T *const LPF_M;

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
 * '<Root>' : 'LPF'
 * '<S1>'   : 'LPF/LPF'
 */
#endif                                 /* RTW_HEADER_LPF_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
