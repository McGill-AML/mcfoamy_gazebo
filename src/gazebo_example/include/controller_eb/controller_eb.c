/*
 * File: controller_eb.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 16-May-2018 15:06:37
 */

/* Include files */
#include "controller_eb.h"

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  double ro;
  double A_prop;
  double m;
  double Ix;
  double Iy;
  double Iz;
  double S;
  double b;
  double cbar;
  double Cl_delta_a;
  double Cm_delta_e;
  double Cn_delta_r;
  double delta_a_max;
  double delta_e_max;
  double delta_r_max;
  double omega_t_min;
  double omega_t_max;
  double Fhat[3];
  double Faero2;
  double Faero1;
  double Faero0;
} struct_T;

#endif                                 /*typedef_struct_T*/

/* Variable Definitions */
static double delta_h_i_a;
static double omega_t_old;
static double delay_element_1;
static double delay_element_2;

/* Function Declarations */
static void force_controller_init(void);
static void lpfilt_init(void);
static void mixer(const double M_b[3], double F, const double v[3], const
                  struct_T *parameters, double u[4]);
static void mixer_init(void);

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
static void force_controller_init(void)
{
  delta_h_i_a = 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void lpfilt_init(void)
{
  delay_element_1 = 0.0;
  delay_element_2 = 0.0;
}

/*
 * Arguments    : const double M_b[3]
 *                double F
 *                const double v[3]
 *                const struct_T *parameters
 *                double u[4]
 * Return Type  : void
 */
static void mixer(const double M_b[3], double F, const double v[3], const
                  struct_T *parameters, double u[4])
{
  double Vs;
  double Vs_min;
  double Vs_filt;
  double delta_a;
  double delta_e;
  double delta_r;
  if (F < 0.0) {
    F = 0.0;
  }

  Vs = sqrt(v[0] * v[0] + 2.0 * F / (parameters->ro * parameters->A_prop));

  /* Approximated Slipstream Velocity (m/s) */
  Vs_min = sqrt(2.0 * parameters->m * 9.81 / (parameters->ro *
    parameters->A_prop));

  /* Approximated minimum slipstream velocity-- Calculated from hover */
  /* Ensure a minimum slipstream velocity */
  if (Vs < Vs_min) {
    Vs = Vs_min;
  }

  /*  do the filtering */
  Vs = (Vs - delay_element_1 * -1.9111970674260732) - delay_element_2 *
    0.91497583480143374;
  Vs_filt = (Vs * 0.00094469184384015075 + delay_element_1 *
             0.0018893836876803015) + delay_element_2 * 0.00094469184384015075;
  delay_element_2 = delay_element_1;
  delay_element_1 = Vs;
  if (Vs_filt < Vs_min) {
    Vs_filt = Vs_min;
  }

  delta_a = M_b[0] / (0.5 * parameters->ro * (Vs_filt * Vs_filt) * parameters->S
                      * parameters->b * parameters->Cl_delta_a);

  /* Aileron Deflection (deg) */
  delta_e = M_b[1] / (0.5 * parameters->ro * (Vs_filt * Vs_filt) * parameters->S
                      * parameters->cbar * parameters->Cm_delta_e);

  /* Elevator Deflection (deg) */
  delta_r = M_b[2] / (0.5 * parameters->ro * (Vs_filt * Vs_filt) * parameters->S
                      * parameters->b * parameters->Cn_delta_r);

  /* Rudder Deflection (deg) */
  Vs = 0.0;
  Vs_min = 0.0;
  Vs_filt = 0.0;
  if (fabs(delta_a) > parameters->delta_a_max) {
    Vs = sqrt(fabs(M_b[0] / (0.5 * parameters->ro * parameters->S *
                parameters->b * parameters->Cl_delta_a * parameters->delta_a_max)));
  }

  if (fabs(delta_e) > parameters->delta_e_max) {
    Vs_min = sqrt(fabs(M_b[1] / (0.5 * parameters->ro * parameters->S *
      parameters->cbar * parameters->Cm_delta_e * parameters->delta_e_max)));
  }

  if (fabs(delta_r) > parameters->delta_r_max) {
    Vs_filt = sqrt(fabs(M_b[2] / (0.5 * parameters->ro * parameters->S *
      parameters->b * parameters->Cn_delta_r * parameters->delta_r_max)));
  }

  if (Vs < Vs_min) {
    Vs = Vs_min;
  }

  if (Vs < Vs_filt) {
    Vs = Vs_filt;
  }

  if ((Vs > v[0]) && (Vs > 0.0)) {
    Vs_min = F + parameters->ro * parameters->A_prop / 2.0 * (Vs * Vs - v[0] *
      v[0]);
  } else {
    Vs_min = F;
  }

  Vs = sqrt((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]) / (omega_t_old *
    0.0042333);

  /* Advance ratio = V/(omega/60 * D) */
  if (Vs < 0.0) {
    Vs = 0.0;
  }

  if (Vs > 0.5) {
    Vs = 0.5;
  }

  omega_t_old = sqrt(Vs_min / (((-1.43909969 * (Vs * Vs) - 2.21240323 * Vs) +
    2.24512051) * 1.0E-7));

  /* RPM command         */
  if (omega_t_old < parameters->omega_t_min) {
    omega_t_old = parameters->omega_t_min;
  } else {
    if (omega_t_old > parameters->omega_t_max) {
      omega_t_old = parameters->omega_t_max;
    }
  }

  u[0] = delta_a;
  u[1] = delta_e;
  u[2] = delta_r;
  u[3] = omega_t_old;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void mixer_init(void)
{
  omega_t_old = 1.0;
}

/*
 * Arguments    : const double C_ba[9]
 *                const double r_a[3]
 *                const double v_b[3]
 *                const double omega_ba_b[3]
 *                const double C_ra[9]
 *                const double r_ref_a[3]
 *                const double v_ref_r[3]
 *                const double omega_ra_r[3]
 *                double dt
 *                double maneuver_switch
 *                double *u1
 *                double *u2
 *                double *u3
 *                double *u4
 * Return Type  : void
 */
void controller_eb(const double C_ba[9], const double r_a[3], const double v_b[3],
                   const double omega_ba_b[3], const double C_ra[9], const
                   double r_ref_a[3], const double v_ref_r[3], const double
                   omega_ra_r[3], double dt, double maneuver_switch, double *u1,
                   double *u2, double *u3, double *u4)
{
  double b_r_ref_a[3];
  double b_C_ra[9];
  signed char parameters_Fhat[3];
  int i;
  static const signed char iv0[3] = { 1, 0, 0 };

  int i0;
  int i1;
  double b_v_ref_r[3];
  double V;
  double a[3];
  double theta_r[3];
  double dv0[9];
  static const signed char iv1[3] = { 1, 0, 0 };

  static const signed char iv2[3] = { 0, 1, 0 };

  double dv1[9];
  double dv2[9];
  static const signed char iv3[3] = { 0, 0, 1 };

  double F_aero;
  double C_db[9];
  double phi;
  double b_C_db[3];
  double v[3];
  double delta_h_a;
  double scale;
  double absxk;
  struct_T expl_temp;
  double b_C_ba[9];
  static const double b[3] = { 0.0, 0.0, 9.81 };

  double dv3[3];
  double dv4[3];
  double u[4];

  /* Air Density (kg/m^3) */
  /* Propeller Disk Area (m^2) */
  /* Mass (kg) */
  /* Moment of Inertia about x (kg m^2) */
  /* Moment of Inertia about y (kg m^2) */
  /* Moment of Inertia about z (kg m^2) */
  /* Wing Area (m^2) */
  /* Wing Span (m) */
  /* Mean Aerodynamic Chord (m) */
  /* Aileron Control Derivative Coefficient (/deg) */
  /* Elevator Control Derivative Coefficient (/deg) */
  /* Rudder Control Derivative Coefficient (/deg) */
  /* degrees */
  /* degrees */
  /* degrees */
  /* RPM */
  /* RPM */
  for (i = 0; i < 3; i++) {
    parameters_Fhat[i] = iv0[i];
    b_r_ref_a[i] = r_ref_a[i] - r_a[i];
    for (i0 = 0; i0 < 3; i0++) {
      b_C_ra[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        b_C_ra[i + 3 * i0] += C_ra[i + 3 * i1] * C_ba[i0 + 3 * i1];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    V = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      V += b_C_ra[i0 + 3 * i1] * v_b[i1];
    }

    b_v_ref_r[i0] = v_ref_r[i0] - V;
  }

  for (i0 = 0; i0 < 3; i0++) {
    V = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      V += 0.08 * C_ra[i0 + 3 * i1] * b_r_ref_a[i1];
    }

    a[i0] = V + 0.1 * b_v_ref_r[i0];
  }

  theta_r[0] = 0.0;
  theta_r[1] = 0.0 - a[2];
  theta_r[2] = a[1];

  /* Computes Desired Moments based on desired aircraft attitude */
  for (i = 0; i < 3; i++) {
    b_C_ra[3 * i] = iv1[i];
    if (theta_r[i] < -0.785375) {
      theta_r[i] = -0.785375;
    } else {
      if (theta_r[i] > 0.785375) {
        theta_r[i] = 0.785375;
      }
    }
  }

  b_C_ra[1] = 0.0;
  b_C_ra[4] = cos(theta_r[0]);
  b_C_ra[7] = sin(theta_r[0]);
  b_C_ra[2] = 0.0;
  b_C_ra[5] = -sin(theta_r[0]);
  b_C_ra[8] = cos(theta_r[0]);
  dv0[0] = cos(theta_r[1]);
  dv0[3] = 0.0;
  dv0[6] = -sin(theta_r[1]);
  for (i0 = 0; i0 < 3; i0++) {
    dv0[1 + 3 * i0] = iv2[i0];
  }

  dv0[2] = sin(theta_r[1]);
  dv0[5] = 0.0;
  dv0[8] = cos(theta_r[1]);
  dv2[0] = cos(theta_r[2]);
  dv2[3] = sin(theta_r[2]);
  dv2[6] = 0.0;
  dv2[1] = -sin(theta_r[2]);
  dv2[4] = cos(theta_r[2]);
  dv2[7] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      dv1[i0 + 3 * i1] = 0.0;
      for (i = 0; i < 3; i++) {
        dv1[i0 + 3 * i1] += b_C_ra[i0 + 3 * i] * dv0[i + 3 * i1];
      }
    }

    dv2[2 + 3 * i0] = iv3[i0];
  }

  F_aero = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_C_ra[i0 + 3 * i1] = 0.0;
      for (i = 0; i < 3; i++) {
        b_C_ra[i0 + 3 * i1] += dv1[i0 + 3 * i] * dv2[i + 3 * i1];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      dv0[i0 + 3 * i1] = 0.0;
      for (i = 0; i < 3; i++) {
        dv0[i0 + 3 * i1] += b_C_ra[i0 + 3 * i] * C_ra[i + 3 * i1];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      C_db[i0 + 3 * i1] = 0.0;
      for (i = 0; i < 3; i++) {
        C_db[i0 + 3 * i1] += dv0[i0 + 3 * i] * C_ba[i1 + 3 * i];
      }
    }

    F_aero += C_db[i0 + 3 * i0];
  }

  phi = acos((F_aero - 1.0) / 2.0);
  if (phi == 0.0) {
    for (i = 0; i < 3; i++) {
      a[i] = 0.0;
    }
  } else {
    F_aero = 2.0 * sin(phi);
    b_C_db[0] = C_db[7] - C_db[5];
    b_C_db[1] = C_db[2] - C_db[6];
    b_C_db[2] = C_db[3] - C_db[1];
    for (i0 = 0; i0 < 3; i0++) {
      a[i0] = b_C_db[i0] / F_aero;
    }
  }

  v[0] = 0.003922;
  v[1] = 0.01594;
  v[2] = 0.01934;
  memset(&C_db[0], 0, 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    C_db[i + 3 * i] = v[i];
  }

  delta_h_a = -(r_ref_a[2] - r_a[2]);

  /* Height error (m) */
  delta_h_i_a += delta_h_a * dt;

  /* Height error integral (ms) */
  if (maneuver_switch == 1.0) {
    delta_h_i_a = 0.0;
  }

  V = 0.0;
  scale = 2.2250738585072014E-308;
  for (i = 0; i < 3; i++) {
    absxk = fabs(v_b[i]);
    if (absxk > scale) {
      F_aero = scale / absxk;
      V = 1.0 + V * F_aero * F_aero;
      scale = absxk;
    } else {
      F_aero = absxk / scale;
      V += F_aero * F_aero;
    }
  }

  V = scale * sqrt(V);
  F_aero = 2.0 * ((-0.0157 * (V * V) + 0.0524 * V) + -0.5583);
  expl_temp.Faero0 = -0.5583;
  expl_temp.Faero1 = 0.0524;
  expl_temp.Faero2 = -0.0157;
  for (i = 0; i < 3; i++) {
    expl_temp.Fhat[i] = parameters_Fhat[i];
    for (i0 = 0; i0 < 3; i0++) {
      b_C_ba[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        b_C_ba[i + 3 * i0] += C_ba[i + 3 * i1] * C_ra[i0 + 3 * i1];
      }
    }
  }

  expl_temp.omega_t_max = 6710.0;
  expl_temp.omega_t_min = 1716.0;
  expl_temp.delta_r_max = 49.0;
  expl_temp.delta_e_max = 59.0;
  expl_temp.delta_a_max = 52.0;
  expl_temp.Cn_delta_r = -0.0035663;
  expl_temp.Cm_delta_e = -0.0117747;
  expl_temp.Cl_delta_a = -0.0006777;
  expl_temp.cbar = 0.21;
  expl_temp.b = 0.864;
  expl_temp.S = 0.14274;
  expl_temp.Iz = 0.01934;
  expl_temp.Iy = 0.01594;
  expl_temp.Ix = 0.003922;
  expl_temp.m = 0.45;
  expl_temp.A_prop = 0.0507;
  expl_temp.ro = 1.225;
  for (i0 = 0; i0 < 3; i0++) {
    V = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      V += b_C_ba[i0 + 3 * i1] * omega_ra_r[i1];
    }

    v[i0] = V - omega_ba_b[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    b_r_ref_a[i0] = 160.0 * (phi * a[i0]) + 8.0 * v[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    b_C_db[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      b_C_db[i0] += C_db[i0 + 3 * i1] * b_r_ref_a[i1];
    }

    v[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      v[i0] += C_ba[i0 + 3 * i1] * b[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      b_C_ba[i0 + 3 * i1] = 0.0;
      for (i = 0; i < 3; i++) {
        b_C_ba[i0 + 3 * i1] += C_ba[i0 + 3 * i] * C_ra[i1 + 3 * i];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    V = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      V += b_C_ba[i0 + 3 * i1] * v_ref_r[i1];
    }

    b_r_ref_a[i0] = V - v_b[i0];
  }

  dv3[0] = 0.0;
  dv3[1] = 0.0;
  dv3[2] = -delta_h_a;
  dv4[0] = 0.0;
  dv4[1] = 0.0;
  dv4[2] = -delta_h_i_a;
  V = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    b_v_ref_r[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      b_v_ref_r[i0] += C_ba[i0 + 3 * i1] * dv3[i1];
    }

    theta_r[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      theta_r[i0] += C_ba[i0 + 3 * i1] * dv4[i1];
    }

    V += ((-0.45 * v[i0] - F_aero) + 0.45 * ((3.0 * b_r_ref_a[i0] + 5.0 *
            b_v_ref_r[i0]) + 0.5 * theta_r[i0])) * (double)parameters_Fhat[i0];
  }

  mixer(b_C_db, V, v_b, &expl_temp, u);
  *u1 = u[0];
  *u2 = u[1];
  *u3 = u[2];
  *u4 = u[3];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void controller_eb_initialize(void)
{
  force_controller_init();
  mixer_init();
  lpfilt_init();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void controller_eb_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for controller_eb.c
 *
 * [EOF]
 */
