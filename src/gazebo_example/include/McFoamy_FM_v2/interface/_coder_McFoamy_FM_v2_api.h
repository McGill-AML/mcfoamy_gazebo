#ifndef ___CODER_MCFOAMY_FM_V2_API_H__
#define ___CODER_MCFOAMY_FM_V2_API_H__
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
extern void McFoamy_FM_v2_initialize(emlrtContext *aContext);
extern void McFoamy_FM_v2_terminate(void);
extern void McFoamy_FM_v2_atexit(void);
extern void McFoamy_FM_v2_api(const mxArray * const prhs[10], const mxArray *plhs[6]);
extern void McFoamy_FM_v2(double LAilDef, double ElevDef, double RudDef, double wIn, double u, double v, double w, double p, double q, double r, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz);
extern void McFoamy_FM_v2_xil_terminate(void);

#endif
