/*
 * maneuver_generator.h
 *
 * Code generation for function 'maneuver_generator'
 *
 */

#ifndef __MANEUVER_GENERATOR_H__
#define __MANEUVER_GENERATOR_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "maneuver_generator_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void maneuver_generator(double maneuver_type, double absolute_time,
    const double p[3], const double q[4], double *maneuver_switch, double qref[4],
    double pref[3], double *uref, double *distance);
  extern void maneuver_generator_initialize(void);
  extern void maneuver_generator_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/* End of code generation (maneuver_generator.h) */
