/*
 * File: controller_eb.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 16-May-2018 15:06:37
 */

#ifndef __CONTROLLER_EB_H__
#define __CONTROLLER_EB_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "controller_eb_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void controller_eb(const double C_ba[9], const double r_a[3], const
    double v_b[3], const double omega_ba_b[3], const double C_ra[9], const
    double r_ref_a[3], const double v_ref_r[3], const double omega_ra_r[3],
    double dt, double maneuver_switch, double *u1, double *u2, double *u3,
    double *u4);
  extern void controller_eb_initialize(void);
  extern void controller_eb_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for controller_eb.h
 *
 * [EOF]
 */
