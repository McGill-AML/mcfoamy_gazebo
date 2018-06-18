/* 
 * File: _coder_controller_eb_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 16-May-2018 15:06:37 
 */

#ifndef ___CODER_CONTROLLER_EB_API_H__
#define ___CODER_CONTROLLER_EB_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void controller_eb_initialize(emlrtContext *aContext);
extern void controller_eb_terminate(void);
extern void controller_eb_atexit(void);
extern void controller_eb_api(const mxArray *prhs[10], const mxArray *plhs[4]);
extern void controller_eb(double C_ba[9], double r_a[3], double v_b[3], double omega_ba_b[3], double C_ra[9], double r_ref_a[3], double v_ref_r[3], double omega_ra_r[3], double dt, double maneuver_switch, double *u1, double *u2, double *u3, double *u4);
extern void controller_eb_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_controller_eb_api.h 
 *  
 * [EOF] 
 */
