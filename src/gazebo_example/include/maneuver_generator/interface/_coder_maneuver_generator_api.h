/*
 * _coder_maneuver_generator_api.h
 *
 * Code generation for function 'maneuver_generator'
 *
 */

#ifndef ___CODER_MANEUVER_GENERATOR_API_H__
#define ___CODER_MANEUVER_GENERATOR_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void maneuver_generator_initialize(emlrtContext *aContext);
extern void maneuver_generator_terminate(void);
extern void maneuver_generator_atexit(void);
extern void maneuver_generator_api(const mxArray *prhs[4], const mxArray *plhs[5]);
extern void maneuver_generator(double maneuver_type, double absolute_time, double p[3], double q[4], double *maneuver_switch, double qref[4], double pref[3], double *uref, double *distance);
extern void maneuver_generator_xil_terminate(void);

#endif
/* End of code generation (_coder_maneuver_generator_api.h) */
