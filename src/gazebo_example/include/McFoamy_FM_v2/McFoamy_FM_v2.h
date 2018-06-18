#ifndef __MCFOAMY_FM_V2_H__
#define __MCFOAMY_FM_V2_H__
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "McFoamy_FM_v2_types.h"
#ifdef __cplusplus

extern "C" {

#endif

  extern void McFoamy_FM_v2(double LAilDef, double ElevDef, double RudDef,
    double wIn, double u, double v, double w, double p, double q, double r,
    double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz);
  extern void McFoamy_FM_v2_initialize(void);
  extern void McFoamy_FM_v2_terminate(void);

#ifdef __cplusplus

}
#endif
#endif
