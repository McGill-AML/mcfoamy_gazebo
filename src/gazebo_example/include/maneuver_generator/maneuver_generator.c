/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * maneuver_generator.c
 *
 * Code generation for function 'maneuver_generator'
 *
 */

/* Include files */
#include <math.h>
#include "maneuver_generator.h"

/* Variable Definitions */
static double maneuver_type_old;
static double start_time;
static boolean_T start_time_not_empty;
static double q_ref[4];
static double p_ref[3];
static boolean_T p_ref_not_empty;
static double u_ref;
static double p_0[3];
static boolean_T p_0_not_empty;
static double theta_ref;
static double psi_ref;
static boolean_T psi_ref_not_empty;
static double transition1;
static double transition2;

/* Function Declarations */
static void EulToQuat(const double Eul[3], double Quat[4]);
static void QuatToEul(const double Quat[4], double Eul[3]);
static void maneuver_generator_init(void);

/* Function Definitions */
static void EulToQuat(const double Eul[3], double Quat[4])
{
  Quat[0] = cos(Eul[2] / 2.0) * cos(Eul[1] / 2.0) * cos(Eul[0] / 2.0) + sin(Eul
    [2] / 2.0) * sin(Eul[1] / 2.0) * sin(Eul[0] / 2.0);
  Quat[1] = cos(Eul[2] / 2.0) * cos(Eul[1] / 2.0) * sin(Eul[0] / 2.0) - sin(Eul
    [2] / 2.0) * sin(Eul[1] / 2.0) * cos(Eul[0] / 2.0);
  Quat[2] = cos(Eul[2] / 2.0) * sin(Eul[1] / 2.0) * cos(Eul[0] / 2.0) + sin(Eul
    [2] / 2.0) * cos(Eul[1] / 2.0) * sin(Eul[0] / 2.0);
  Quat[3] = sin(Eul[2] / 2.0) * cos(Eul[1] / 2.0) * cos(Eul[0] / 2.0) - cos(Eul
    [2] / 2.0) * sin(Eul[1] / 2.0) * sin(Eul[0] / 2.0);
}

static void QuatToEul(const double Quat[4], double Eul[3])
{
  Eul[0] = atan2(2.0 * (Quat[0] * Quat[1] + Quat[2] * Quat[3]), ((Quat[0] *
    Quat[0] + Quat[3] * Quat[3]) - Quat[1] * Quat[1]) - Quat[2] * Quat[2]);
  if (2.0 * (Quat[0] * Quat[2] - Quat[1] * Quat[3]) > 1.0) {
    Eul[1] = 1.5707963267948966;
  } else if (2.0 * (Quat[0] * Quat[2] - Quat[1] * Quat[3]) < -1.0) {
    Eul[1] = -1.5707963267948966;
  } else {
    Eul[1] = asin(2.0 * (Quat[0] * Quat[2] - Quat[1] * Quat[3]));
  }

  Eul[2] = atan2(2.0 * (Quat[0] * Quat[3] + Quat[1] * Quat[2]), ((Quat[0] *
    Quat[0] + Quat[1] * Quat[1]) - Quat[2] * Quat[2]) - Quat[3] * Quat[3]);
}

static void maneuver_generator_init(void)
{
  int i0;
  static const signed char iv0[4] = { 1, 0, 0, 0 };

  for (i0 = 0; i0 < 4; i0++) {
    q_ref[i0] = iv0[i0];
  }

  maneuver_type_old = -1.0;
  transition1 = 0.0;
  transition2 = 0.0;
  theta_ref = 0.0;
  u_ref = 0.0;
}

void maneuver_generator(double maneuver_type, double absolute_time, const double
  p[3], const double q[4], double *maneuver_switch, double qref[4], double pref
  [3], double *uref, double *distance)
{
  double eul[3];
  int k;
  double delta_yaw;
  double dv0[3];
  double absxk;
  double t;
  QuatToEul(q, eul);
  if (!psi_ref_not_empty) {
    psi_ref = eul[2];
    psi_ref_not_empty = true;
  }

  /*  Define maneuver switch */
  *maneuver_switch = (maneuver_type - maneuver_type_old != 0.0);
  if (!p_ref_not_empty) {
    for (k = 0; k < 3; k++) {
      p_ref[k] = p[k];
    }

    p_ref_not_empty = true;
  }

  if (!p_0_not_empty) {
    for (k = 0; k < 3; k++) {
      p_0[k] = p[k];
    }

    p_0_not_empty = true;
  }

  if (!start_time_not_empty) {
    start_time = absolute_time;
    start_time_not_empty = true;
  }

  /*      if absolute_time>-100 && u_ref < 4 */
  /*          u_ref = 4 */
  /*          maneuver_switch = true */
  /*      elseif absolute_time>200 && u_ref < 5 */
  /*          u_ref = 5 */
  /*          maneuver_switch = true */
  /*      elseif absolute_time>400 && u_ref < 6 */
  /*          u_ref = 6 */
  /*          maneuver_switch = true */
  /*      elseif absolute_time>600 && u_ref < 7 */
  /*          u_ref = 7 */
  /*          maneuver_switch = true; */
  /*      elseif absolute_time>800 && u_ref < 8 */
  /*          u_ref = 8 */
  /*          maneuver_switch = true; */
  /*      elseif absolute_time>1000 && u_ref < 9 */
  /*          u_ref = 9 */
  /*          maneuver_switch = true; */
  /*      end */
  /*  	1)level flight */
  /*  	2)Hover */
  /*  	3)Rolling Harrier */
  /*  	4)Knife-Edge */
  /*  	5)Flat-Spin */
  /*  	6)Aggressive Turnaround */
  /*  	7)Level turnaround to inverted */
  /*  	8)Inverted turnaround to level */
  /*  	9)Inverted flight */
  /*  	10)Take Off */
  /*  	11)L&& */
  if (maneuver_type == 1.0) {
    if (*maneuver_switch == 1.0) {
      for (k = 0; k < 3; k++) {
        p_0[k] = p[k];
        p_ref[k] = p_0[k];
      }

      u_ref = 5.0;
      theta_ref = (0.0191 * (u_ref * u_ref) - 0.3022 * u_ref) + 1.3262;
      dv0[0] = 0.0;
      dv0[1] = theta_ref;
      dv0[2] = psi_ref;
      EulToQuat(dv0, q_ref);
    }

    delta_yaw = cos(psi_ref);
    p_ref[0] = (delta_yaw * delta_yaw * (p[0] - p_0[0]) + sin(psi_ref) * cos
                (psi_ref) * (p[1] - p_0[1])) + p_0[0];
    delta_yaw = sin(psi_ref);
    p_ref[1] = (delta_yaw * delta_yaw * (p[1] - p_0[1]) + sin(psi_ref) * cos
                (psi_ref) * (p[0] - p_0[0])) + p_0[1];
  }

  if (maneuver_type == 2.0) {
    if (*maneuver_switch == 1.0) {
      for (k = 0; k < 3; k++) {
        p_0[k] = p[k];
        p_ref[k] = p_0[k];
      }

      dv0[0] = 0.0;
      dv0[1] = 1.5707963267948966;
      dv0[2] = psi_ref;
      EulToQuat(dv0, q_ref);
      u_ref = 0.0;
      transition1 = 0.0;
    }

    if (transition1 == 0.0) {
      delta_yaw = fabs(psi_ref - eul[2]);
      if (fabs((6.2831853071795862 + psi_ref) - eul[2]) < delta_yaw) {
        delta_yaw = fabs((6.2831853071795862 + psi_ref) - eul[2]);
      }

      if (fabs((-6.2831853071795862 + psi_ref) - eul[2]) < delta_yaw) {
        delta_yaw = fabs((-6.2831853071795862 + psi_ref) - eul[2]);
      }

      if (delta_yaw > 1.5707963267948966) {
        for (k = 0; k < 3; k++) {
          p_ref[k] = p[k];
        }

        transition1 = 1.0;
      }
    }

    /*          if (transition1 == false) */
    /*              delta_q_r = quatmultiply(quatconj(q),q_ref); */
    /*              E_yr = 2.0*acos(delta_q_r(1))*delta_q_r(3)/norm(delta_q_r(2:4)); */
    /*  			if E_yr<0 */
    /*                  p_ref=p; */
    /*                  transition1 = true; */
    /*              end */
    /*          end */
    /*  		if (transition1 == false && theta > 1.5) */
    /*  			p_ref=p; */
    /*  			transition1 = true; */
    /*          end */
  }

  if (maneuver_type == 3.0) {
    if (*maneuver_switch == 1.0) {
      for (k = 0; k < 3; k++) {
        p_0[k] = p[k];
        p_ref[k] = p_0[k];
      }

      u_ref = 5.0;
      theta_ref = (0.0082 * (u_ref * u_ref) - 0.1913 * u_ref) + 1.2408;
      start_time = absolute_time;
    }

    delta_yaw = 5.0 * (absolute_time - start_time);
    for (k = 0; k < 4; k++) {
      q_ref[k] = 0.0;
    }

    q_ref[0] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) * cos(delta_yaw / 2.0)
      + sin(psi_ref / 2.0) * sin(theta_ref / 2.0) * sin(delta_yaw / 2.0);
    q_ref[1] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) * sin(delta_yaw / 2.0)
      - sin(psi_ref / 2.0) * sin(theta_ref / 2.0) * cos(delta_yaw / 2.0);
    q_ref[2] = cos(psi_ref / 2.0) * sin(theta_ref / 2.0) * cos(delta_yaw / 2.0)
      + sin(psi_ref / 2.0) * cos(theta_ref / 2.0) * sin(delta_yaw / 2.0);
    q_ref[3] = sin(psi_ref / 2.0) * cos(theta_ref / 2.0) * cos(delta_yaw / 2.0)
      - cos(psi_ref / 2.0) * sin(theta_ref / 2.0) * sin(delta_yaw / 2.0);
    delta_yaw = cos(psi_ref);
    p_ref[0] = (delta_yaw * delta_yaw * (p[0] - p_0[0]) + sin(psi_ref) * cos
                (psi_ref) * (p[1] - p_0[1])) + p_0[0];
    delta_yaw = sin(psi_ref);
    p_ref[1] = (delta_yaw * delta_yaw * (p[1] - p_0[1]) + sin(psi_ref) * cos
                (psi_ref) * (p[0] - p_0[0])) + p_0[1];
  }

  if (maneuver_type == 4.0) {
    if (*maneuver_switch == 1.0) {
      for (k = 0; k < 3; k++) {
        p_0[k] = p[k];
        p_ref[k] = p_0[k];
      }

      u_ref = 5.0;
      theta_ref = (0.0018 * (u_ref * u_ref) - 0.0854 * u_ref) + 1.0929;
      for (k = 0; k < 4; k++) {
        q_ref[k] = 0.0;
      }

      q_ref[0] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) * 0.70710678118654757
        + sin(psi_ref / 2.0) * sin(theta_ref / 2.0) * 0.70710678118654746;
      q_ref[1] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) * 0.70710678118654746
        - sin(psi_ref / 2.0) * sin(theta_ref / 2.0) * 0.70710678118654757;
      q_ref[2] = cos(psi_ref / 2.0) * sin(theta_ref / 2.0) * 0.70710678118654757
        + sin(psi_ref / 2.0) * cos(theta_ref / 2.0) * 0.70710678118654746;
      q_ref[3] = sin(psi_ref / 2.0) * cos(theta_ref / 2.0) * 0.70710678118654757
        - cos(psi_ref / 2.0) * sin(theta_ref / 2.0) * 0.70710678118654746;
    }

    delta_yaw = cos(psi_ref);
    p_ref[0] = (delta_yaw * delta_yaw * (p[0] - p_0[0]) + sin(psi_ref) * cos
                (psi_ref) * (p[1] - p_0[1])) + p_0[0];
    delta_yaw = sin(psi_ref);
    p_ref[1] = (delta_yaw * delta_yaw * (p[1] - p_0[1]) + sin(psi_ref) * cos
                (psi_ref) * (p[0] - p_0[0])) + p_0[1];
  }

  /*  	if (maneuver_type==5) */
  /*  		if (maneuver_switch == true) */
  /*  			p_0 = p; */
  /*  			q_ref.from_euler(0.0, -pi/2.0, psi_ref); */
  /*  			u_ref = 0.0; */
  /*  			transition1 = false; */
  /*  			transition2 = false; */
  /*          end */
  /*   */
  /*  		if (transition1 == false) */
  /*  		 */
  /*  			p_ref = p; */
  /*          end */
  /*   */
  /*  		if (transition1 == false && theta < -80.0*(pi/180.0)) */
  /*  			transition1 = true; */
  /*  			start_time = absolute_time; */
  /*          end */
  /*   */
  /*  		if (transition1 == true && transition2 == false) */
  /*              t = absolute_time - start_time; */
  /*  			q_ref = EulToQuat([600.0*(pi/180.0)*t, -pi/2.0, psi_ref]); */
  /*  			p_ref = p; */
  /*  			if (t>1) */
  /*  				transition2 = true; */
  /*  				start_time = absolute_time; */
  /*              end */
  /*          end */
  /*   */
  /*  		if (transition2 == true) */
  /*              t = absolute_time - start_time; */
  /*  			q_ref = EulToQuat([20.0*(pi/180.0), 0.0, 400*(pi/180.0)*t + psi_ref + pi]); */
  /*          end */
  /*      end */
  if (maneuver_type == 6.0) {
    if (*maneuver_switch == 1.0) {
      for (k = 0; k < 3; k++) {
        p_0[k] = p[k];
        p_ref[k] = p_0[k];
      }

      for (k = 0; k < 4; k++) {
        q_ref[k] = 0.0;
      }

      q_ref[0] = cos(psi_ref / 2.0) * 0.70710678118654757 + sin(psi_ref / 2.0) *
        0.70710678118654746 * 0.0;
      q_ref[1] = cos(psi_ref / 2.0) * 0.70710678118654757 * 0.0 - sin(psi_ref /
        2.0) * 0.70710678118654746;
      q_ref[2] = cos(psi_ref / 2.0) * 0.70710678118654746 + sin(psi_ref / 2.0) *
        0.70710678118654757 * 0.0;
      q_ref[3] = sin(psi_ref / 2.0) * 0.70710678118654757 - cos(psi_ref / 2.0) *
        0.70710678118654746 * 0.0;
      u_ref = 5.0;
      transition1 = 0.0;
      transition2 = 0.0;
    }

    if ((transition1 == 0.0) && (eul[1] > 0.78539816339744828)) {
      psi_ref -= 3.1415926535897931;
      u_ref = 5.0;
      theta_ref = (0.0191 * (u_ref * u_ref) - 0.3022 * u_ref) + 1.3262;
      for (k = 0; k < 4; k++) {
        q_ref[k] = 0.0;
      }

      q_ref[0] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) *
        6.123233995736766E-17 + sin(psi_ref / 2.0) * sin(theta_ref / 2.0);
      q_ref[1] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) - sin(psi_ref / 2.0) *
        sin(theta_ref / 2.0) * 6.123233995736766E-17;
      q_ref[2] = cos(psi_ref / 2.0) * sin(theta_ref / 2.0) *
        6.123233995736766E-17 + sin(psi_ref / 2.0) * cos(theta_ref / 2.0);
      q_ref[3] = sin(psi_ref / 2.0) * cos(theta_ref / 2.0) *
        6.123233995736766E-17 - cos(psi_ref / 2.0) * sin(theta_ref / 2.0);
      transition1 = 1.0;
    }

    if ((transition1 == 1.0) && (transition2 == 0.0) && (eul[1] < theta_ref)) {
      for (k = 0; k < 4; k++) {
        q_ref[k] = 0.0;
      }

      q_ref[0] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) + sin(psi_ref / 2.0) *
        sin(theta_ref / 2.0) * 0.0;
      q_ref[1] = cos(psi_ref / 2.0) * cos(theta_ref / 2.0) * 0.0 - sin(psi_ref /
        2.0) * sin(theta_ref / 2.0);
      q_ref[2] = cos(psi_ref / 2.0) * sin(theta_ref / 2.0) + sin(psi_ref / 2.0) *
        cos(theta_ref / 2.0) * 0.0;
      q_ref[3] = sin(psi_ref / 2.0) * cos(theta_ref / 2.0) - cos(psi_ref / 2.0) *
        sin(theta_ref / 2.0) * 0.0;
      transition2 = 1.0;
    }

    if (transition1 == 1.0) {
      delta_yaw = cos(psi_ref);
      p_ref[0] = (delta_yaw * delta_yaw * (p[0] - p_0[0]) + sin(psi_ref) * cos
                  (psi_ref) * (p[1] - p_0[1])) + p_0[0];
      delta_yaw = sin(psi_ref);
      p_ref[1] = (delta_yaw * delta_yaw * (p[1] - p_0[1]) + sin(psi_ref) * cos
                  (psi_ref) * (p[0] - p_0[0])) + p_0[1];
    }
  }

  maneuver_type_old = maneuver_type;
  for (k = 0; k < 4; k++) {
    qref[k] = q_ref[k];
  }

  *uref = u_ref;
  *distance = 0.0;
  delta_yaw = 3.3121686421112381E-170;
  for (k = 0; k < 3; k++) {
    pref[k] = p_ref[k];
    absxk = fabs(p_ref[k] - p_0[k]);
    if (absxk > delta_yaw) {
      t = delta_yaw / absxk;
      *distance = 1.0 + *distance * t * t;
      delta_yaw = absxk;
    } else {
      t = absxk / delta_yaw;
      *distance += t * t;
    }
  }

  *distance = delta_yaw * sqrt(*distance);

  /*  	if (maneuver_type==7) */
  /*  	{ */
  /*  		if (maneuver_switch == true) */
  /*  		{ */
  /*  			p_0 = p; */
  /*  			p_ref = p_0; */
  /*  			q_ref.from_euler(0.0f, pi/2.0f, psi_ref); */
  /*  			u_ref = 5.0f; */
  /*  			transition1 = false; */
  /*  			L_0 = 0.0f; */
  /*  			M_0 = 0.0f; */
  /*  			N_0 = 0.0f; */
  /*  		} */
  /*   */
  /*  		if (transition1 == false && theta > pi/4.0f) */
  /*  		{ */
  /*  			psi_ref = psi_ref - pi; */
  /*  			u_ref = 5.0f; */
  /*  			theta_ref = 0.0118f*powf(u_ref,2.0f) - 0.2242f*u_ref + 1.1914f; */
  /*  			q_ref.from_euler(pi, theta_ref, psi_ref); */
  /*  			transition1 = true; */
  /*  		} */
  /*   */
  /*  		if (transition1 == true) */
  /*  		{ */
  /*  			        p_ref(1)=cos(psi_ref)^2*(p(1)-p_0(1)) + sin(psi_ref)*cos(psi_ref)*(p(2)-p_0(2)) + p_0(1);  */
  /*  			p_ref(2)=sin(psi_ref)^2*(p(2)-p_0(2)) + sin(psi_ref)*cos(psi_ref)*(p(1)-p_0(1)) + p_0(2);  */
  /*  		} */
  /*  	} */
  /*   */
  /*  	if (maneuver_type==8) */
  /*  	{ */
  /*  		if (maneuver_switch == true) */
  /*  		{ */
  /*  			p_0 = p; */
  /*  			p_ref = p_0; */
  /*  			q_ref.from_euler(pi, pi/2.0f, psi_ref); */
  /*  			u_ref = 5.0f; */
  /*  			transition1 = false; */
  /*  			L_0 = 0.0f; */
  /*  			M_0 = 0.0f; */
  /*  			N_0 = 0.0f; */
  /*  		} */
  /*   */
  /*  		if (transition1 == false && theta > pi/4.0f) */
  /*  		{ */
  /*  			psi_ref = psi_ref - pi; */
  /*  			u_ref = 5.0f; */
  /*  			theta_ref = 0.0118f*powf(u_ref,2.0f) - 0.2242f*u_ref + 1.1914f; */
  /*  			q_ref.from_euler(0.0f, theta_ref, psi_ref); */
  /*  			transition1 = true; */
  /*  		} */
  /*   */
  /*  		if (transition1 == true) */
  /*  		{ */
  /*  			        p_ref(1)=cos(psi_ref)^2*(p(1)-p_0(1)) + sin(psi_ref)*cos(psi_ref)*(p(2)-p_0(2)) + p_0(1);  */
  /*  			p_ref(2)=sin(psi_ref)^2*(p(2)-p_0(2)) + sin(psi_ref)*cos(psi_ref)*(p(1)-p_0(1)) + p_0(2);  */
  /*  		} */
  /*  	} */
  /*   */
  /*  	if (maneuver_type==9) */
  /*  	{ */
  /*  		if (maneuver_switch == true) */
  /*  		{ */
  /*  			p_0 = p; */
  /*  			p_ref = p_0; */
  /*  			u_ref = 5.0f; */
  /*  			theta_ref = 0.0118f*powf(u_ref,2.0f) - 0.2242f*u_ref + 1.1914f; */
  /*  			q_ref.from_euler(pi, theta_ref, psi_ref); */
  /*  			L_0 = 0.0f; */
  /*  			M_0 = -0.15f; */
  /*  			N_0 = 0.0f; */
  /*  		} */
  /*  		        p_ref(1)=cos(psi_ref)^2*(p(1)-p_0(1)) + sin(psi_ref)*cos(psi_ref)*(p(2)-p_0(2)) + p_0(1);  */
  /*  		p_ref(2)=sin(psi_ref)^2*(p(2)-p_0(2)) + sin(psi_ref)*cos(psi_ref)*(p(1)-p_0(1)) + p_0(2);  */
  /*  	} */
  /*   */
  /*  	if (maneuver_type==10) */
  /*  	{ */
  /*  		if (maneuver_switch == true) */
  /*  		{ */
  /*  			p_0 = p; */
  /*  			p_ref = p_0; */
  /*  			q_ref.from_euler(0.0f, pi/2.0f, psi_ref); */
  /*  			u_ref = 0.0f; */
  /*  			L_0 = 0.03f; */
  /*  			M_0 = 0.0f; */
  /*  			N_0 = 0.0f; */
  /*  			start_time = hrt_absolute_time(); */
  /*  		} */
  /*   */
  /*  		float t = float(hrt_absolute_time() - start_time)/1000000.0f; */
  /*  		if (p_0(2)-p_ref(2)<6.0f) */
  /*  		{ */
  /*  			p_ref(2)= p_0(2)-4.0f - 0.3f*t; */
  /*  		} */
  /*   */
  /*   */
  /*  	} */
  /*   */
  /*  	if (maneuver_type==11) */
  /*  	{ */
  /*  		if (maneuver_switch == true) */
  /*  		{ */
  /*  			p_0 = p; */
  /*  			p_ref = p_0; */
  /*  			u_ref = 0.0f; */
  /*  			L_0 = 0.03f; */
  /*  			M_0 = 0.0f; */
  /*  			N_0 = 0.0f; */
  /*  			start_time = hrt_absolute_time(); */
  /*  		} */
  /*  		float t = float(hrt_absolute_time() - start_time)/1000000.0f; */
  /*  		p_ref(2) = p_0(2) + 0.3f*t; */
  /*  	} */
  /*   */
  /*   */
  /*   */
  /*  } */
}

void maneuver_generator_initialize(void)
{
  psi_ref_not_empty = false;
  p_0_not_empty = false;
  p_ref_not_empty = false;
  start_time_not_empty = false;
  maneuver_generator_init();
}

void maneuver_generator_terminate(void)
{
  /* (no terminate code required) */
}

/* End of code generation (maneuver_generator.c) */
