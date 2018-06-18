#include "McFoamy_FM_v2.h"

static void McFoamy_Airfoil_Simplified(double alp, double cf, double c, double
  def, double AR, double Cd0, double Cd90, double alp0, double *CN, double *CL,
  double *CD, double *CM);
static void b_atan2(const double y[7], const double x[7], double r[7]);
static int b_bsearch(const double x[10], double xi);
static void b_cos(double x[7]);
static double b_interp1(const double varargin_1[10], const double varargin_2[10],
  double varargin_3);
static void b_power(const double a[3], double y[3]);
static void b_sign(double x[4]);
static void b_sin(double x[7]);
static void b_sqrt(const double x[7], double b_x[7]);
static double b_sum(const double x[3]);
static void c_atan2(const double y[3], const double x[3], double r[3]);
static void c_cos(double x[3]);
static double c_interp1(const double varargin_1[2], const double varargin_2[2],
  double varargin_3);
static void c_power(const double a[4], double y[4]);
static void c_sign(double *x);
static void c_sin(double x[3]);
static void c_sqrt(const double x[3], double b_x[3]);
static double c_sum(const double x[4]);
static void cross(const double a[3], const double b[3], double c[3]);
static void d_atan2(const double y[4], const double x[4], double r[4]);
static void d_cos(double x[4]);
static void d_sin(double x[4]);
static void d_sqrt(const double x[4], double b_x[4]);
static void e_sqrt(double x[7]);
static void f_sqrt(double x[3]);
static void g_sqrt(double x[4]);
static double interp1(const double varargin_1[10], const double varargin_2[10],
                      double varargin_3);
static double interp2_dispatch(const double V[110], double Xq, double Yq, double
  extrapval);
static double norm(const double x[3]);
static void power(const double a[7], double y[7]);
static double sum(const double x[7]);
static void McFoamy_Airfoil_Simplified(double alp, double cf, double c, double
  def, double AR, double Cd0, double Cd90, double alp0, double *CN, double *CL,
  double *CD, double *CM)
{
  static const double dv32[10] = { 0.55850536063818546, 0.62831853071795862,
    0.59341194567807209, 0.66322511575784515, 0.55850536063818546,
    0.36651914291880922, 0.27925268031909273, 0.19198621771937624,
    0.17453292519943295, 0.13962634015954636 };

  static const double dv33[10] = { 0.1666, 0.333, 0.4, 0.5, 1.0, 1.25, 2.0, 3.0,
    4.0, 6.0 };

  double LowAlpEnd;
  static const double dv34[10] = { 0.69813170079773179, 1.0471975511965976,
    0.95993108859688125, 0.97738438111682457, 0.69813170079773179,
    0.50614548307835561, 0.48869219055841229, 0.41887902047863906,
    0.38397243543875248, 0.3490658503988659 };

  double HighAlpStart;
  double CLAlp;
  double alp0_eff;
  double LowAlpEnd_N;
  double LowAlpStart_P;
  double LowAlpStart_N;
  double HighAlpEnd_P;
  double HighAlpStart_N;
  double HighAlpEnd_N;
  double b_gamma;
  double CT;
  double a_eff;
  double x;
  double b_x;
  double c_x;
  double d_x;
  double e_x;
  double f_x;
  double y;
  double CN2;
  double b_LowAlpStart_P[2];
  double CN1[2];
  double c_LowAlpStart_P[2];
  double b_CLAlp[2];
  double d_LowAlpStart_P[2];
  double b_Cd0[2];
  double e_LowAlpStart_P[2];
  double dv35[2];
  LowAlpEnd = b_interp1(dv33, dv32, AR);
  HighAlpStart = b_interp1(dv33, dv34, AR);
  CLAlp = 6.2831853071795862 * AR / (AR + 2.0 * (AR + 4.0) / (AR + 2.0));
  if (cf / c == 1.0) {
    alp0_eff = alp0;
    LowAlpEnd_N = -LowAlpEnd;
    LowAlpStart_P = -LowAlpEnd + 3.1415926535897931;
    LowAlpStart_N = LowAlpEnd - 3.1415926535897931;
    HighAlpEnd_P = 3.1415926535897931 - HighAlpStart;
    HighAlpStart_N = -HighAlpStart;
    HighAlpEnd_N = -(3.1415926535897931 - HighAlpStart);
    alp += def;
    b_gamma = 0.0;
  } else {
    if (fabs(AR - 2.092) < 0.0009) {
      alp0_eff = 0.1145 * pow(def, 3.0) + -0.3372 * def;
      LowAlpEnd = ((0.0678 * pow(def, 3.0) + 0.0009 * (def * def)) + -0.1636 *
                   def) + 0.2713;
      LowAlpEnd_N = ((0.0678 * pow(def, 3.0) + -0.0009 * (def * def)) + -0.1636 *
                     def) + -0.2713;
    } else if (fabs(AR - 0.227938) < 0.0009) {
      alp0_eff = 0.0;
      LowAlpEnd = 0.5842;
      LowAlpEnd_N = -0.5842;
    } else if (fabs(AR - 1.298412) < 0.0009) {
      alp0_eff = 0.1263 * pow(def, 3.0) + -0.3456 * def;
      LowAlpEnd = ((0.0951 * pow(def, 3.0) + 0.0015 * (def * def)) + -0.2331 *
                   def) + 0.361;
      LowAlpEnd_N = ((0.0951 * pow(def, 3.0) + -0.0015 * (def * def)) + -0.2331 *
                     def) + -0.361;
    } else {
      alp0_eff = 0.1381 * pow(def, 3.0) + -0.3734 * def;
      LowAlpEnd = ((0.1196 * pow(def, 3.0) + 0.0006 * (def * def)) + -0.3079 *
                   def) + 0.3726;
      LowAlpEnd_N = ((0.1196 * pow(def, 3.0) + -0.0006 * (def * def)) + -0.3079 *
                     def) + -0.3726;
    }

    LowAlpStart_P = LowAlpEnd_N + 3.1415926535897931;
    LowAlpStart_N = LowAlpEnd - 3.1415926535897931;
    CT = c - cf;
    b_gamma = asin(sin(def) * cf / sqrt((CT * CT + cf * cf) + 2.0 * (c - cf) *
      cf * cos(fabs(def))));
    HighAlpEnd_P = 3.1415926535897931 - HighAlpStart;
    HighAlpStart_N = -HighAlpStart;
    HighAlpEnd_N = -(3.1415926535897931 - HighAlpStart);
  }

  if ((alp >= -3.1415926535897931) && (alp <= LowAlpStart_N)) {
    a_eff = (alp - alp0_eff) + 3.1415926535897931;
    x = cos(a_eff);
    *CL = CLAlp * sin(a_eff) * (x * x) + 3.1415926535897931 * fabs(sin(a_eff)) *
      sin(a_eff) * cos(a_eff);
    *CD = Cd0 + fabs(CLAlp * fabs(sin(a_eff)) * sin(a_eff) * cos(a_eff) +
                     3.1415926535897931 * pow(sin(a_eff), 3.0));
    *CN = -(CLAlp * sin(a_eff) * fabs(cos(a_eff)) + 3.1415926535897931 * fabs
            (sin(a_eff)) * sin(a_eff));
    *CM = -(0.25 - 0.175 * (1.0 - fabs(a_eff - 3.1415926535897931) /
             1.5707963267948966)) * *CN;
  } else if ((alp > LowAlpStart_N) && (alp < HighAlpEnd_N)) {
    a_eff = (LowAlpStart_N - alp0_eff) + 3.1415926535897931;
    x = sin(a_eff);
    b_x = cos(a_eff);
    c_x = sin(a_eff);
    d_x = sin(a_eff);
    e_x = cos(a_eff);
    f_x = CLAlp * fabs(sin(a_eff)) * sin(a_eff) * cos(a_eff) +
      3.1415926535897931 * pow(sin(a_eff), 3.0);
    alp0_eff = -(CLAlp * sin(a_eff) * cos(a_eff) + 3.1415926535897931 * fabs(sin
                  (a_eff)) * sin(a_eff));
    y = fabs(a_eff - 3.1415926535897931);
    a_eff = HighAlpEnd_N + b_gamma;
    if ((cf / c == 1.0) || (cf / c == 0.0)) {
      b_gamma = Cd90;
    } else {
      b_gamma = HighAlpEnd_N;
      c_sign(&b_gamma);
      if (b_gamma == 1.0) {
        CT = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (CT * CT) + 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      } else {
        CT = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (CT * CT) - 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      }
    }

    CN2 = -b_gamma * (1.0 / (0.56 + 0.44 * sin(fabs(a_eff))) - 0.41 * (1.0 - exp
                       (-17.0 / AR))) * sin(fabs(a_eff));
    b_gamma = 0.5 * Cd0 * cos(a_eff);
    b_LowAlpStart_P[0] = LowAlpStart_N;
    b_LowAlpStart_P[1] = HighAlpEnd_N;
    CN1[0] = alp0_eff;
    CN1[1] = CN2;
    *CN = c_interp1(b_LowAlpStart_P, CN1, alp);
    c_LowAlpStart_P[0] = LowAlpStart_N;
    c_LowAlpStart_P[1] = HighAlpEnd_N;
    b_CLAlp[0] = CLAlp * x * (b_x * b_x) + 3.1415926535897931 * fabs(c_x) * d_x *
      e_x;
    b_CLAlp[1] = CN2 * cos(a_eff) - b_gamma * sin(a_eff);
    *CL = c_interp1(c_LowAlpStart_P, b_CLAlp, alp);
    d_LowAlpStart_P[0] = LowAlpStart_N;
    d_LowAlpStart_P[1] = HighAlpEnd_N;
    b_Cd0[0] = Cd0 + fabs(f_x);
    b_Cd0[1] = CN2 * sin(a_eff) + b_gamma * cos(a_eff);
    *CD = c_interp1(d_LowAlpStart_P, b_Cd0, alp);
    e_LowAlpStart_P[0] = LowAlpStart_N;
    e_LowAlpStart_P[1] = HighAlpEnd_N;
    dv35[0] = -(0.25 - 0.175 * (1.0 - y / 1.5707963267948966)) * alp0_eff;
    dv35[1] = -(0.25 - 0.175 * (1.0 - fabs(a_eff) / 1.5707963267948966)) * CN2;
    *CM = c_interp1(e_LowAlpStart_P, dv35, alp);
  } else if ((alp >= HighAlpEnd_N) && (alp <= HighAlpStart_N)) {
    a_eff = alp + b_gamma;
    if ((cf / c == 1.0) || (cf / c == 0.0)) {
      b_gamma = Cd90;
    } else {
      b_gamma = alp;
      c_sign(&b_gamma);
      if (b_gamma == 1.0) {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) + 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      } else {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) - 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      }
    }

    *CN = -b_gamma * (1.0 / (0.56 + 0.44 * sin(fabs(a_eff))) - 0.41 * (1.0 - exp
                       (-17.0 / AR))) * sin(fabs(a_eff));
    CT = 0.5 * Cd0 * cos(a_eff);
    *CL = *CN * cos(a_eff) - CT * sin(a_eff);
    *CD = *CN * sin(a_eff) + CT * cos(a_eff);
    *CM = -(0.25 - 0.175 * (1.0 - fabs(a_eff) / 1.5707963267948966)) * *CN;
  } else if ((alp > HighAlpStart_N) && (alp < LowAlpEnd_N)) {
    a_eff = LowAlpEnd_N - alp0_eff;
    x = sin(a_eff);
    b_x = cos(a_eff);
    c_x = sin(a_eff);
    d_x = sin(a_eff);
    e_x = cos(a_eff);
    f_x = CLAlp * fabs(sin(a_eff)) * sin(a_eff) * cos(a_eff) +
      3.1415926535897931 * pow(sin(a_eff), 3.0);
    alp0_eff = sin(a_eff);
    CT = cos(a_eff);
    HighAlpEnd_P = sin(a_eff);
    HighAlpEnd_N = sin(a_eff);
    LowAlpStart_P = sin(a_eff);
    LowAlpStart_N = sin(a_eff);
    a_eff = HighAlpStart_N + b_gamma;
    if ((cf / c == 1.0) || (cf / c == 0.0)) {
      b_gamma = Cd90;
    } else {
      b_gamma = HighAlpStart_N;
      c_sign(&b_gamma);
      if (b_gamma == 1.0) {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) + 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      } else {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) - 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      }
    }

    CN2 = -b_gamma * (1.0 / (0.56 + 0.44 * sin(fabs(a_eff))) - 0.41 * (1.0 - exp
                       (-17.0 / AR))) * sin(fabs(a_eff));
    b_gamma = 0.5 * Cd0 * cos(a_eff);
    b_LowAlpStart_P[0] = LowAlpEnd_N;
    b_LowAlpStart_P[1] = HighAlpStart_N;
    b_CLAlp[0] = CLAlp * alp0_eff * CT + 3.1415926535897931 * fabs(HighAlpEnd_P)
      * HighAlpEnd_N;
    b_CLAlp[1] = CN2;
    *CN = c_interp1(b_LowAlpStart_P, b_CLAlp, alp);
    c_LowAlpStart_P[0] = LowAlpEnd_N;
    c_LowAlpStart_P[1] = HighAlpStart_N;
    CN1[0] = CLAlp * x * (b_x * b_x) + 3.1415926535897931 * fabs(c_x) * d_x *
      e_x;
    CN1[1] = CN2 * cos(a_eff) - b_gamma * sin(a_eff);
    *CL = c_interp1(c_LowAlpStart_P, CN1, alp);
    d_LowAlpStart_P[0] = LowAlpEnd_N;
    d_LowAlpStart_P[1] = HighAlpStart_N;
    b_Cd0[0] = Cd0 + fabs(f_x);
    b_Cd0[1] = CN2 * sin(a_eff) + b_gamma * cos(a_eff);
    *CD = c_interp1(d_LowAlpStart_P, b_Cd0, alp);
    e_LowAlpStart_P[0] = LowAlpEnd_N;
    e_LowAlpStart_P[1] = HighAlpStart_N;
    dv35[0] = -0.53407075111026481 * fabs(LowAlpStart_P) * LowAlpStart_N;
    dv35[1] = -(0.25 - 0.175 * (1.0 - fabs(a_eff) / 1.5707963267948966)) * CN2;
    *CM = c_interp1(e_LowAlpStart_P, dv35, alp);
  } else if ((alp >= LowAlpEnd_N) && (alp <= LowAlpEnd)) {
    a_eff = alp - alp0_eff;
    x = cos(a_eff);
    *CL = CLAlp * sin(a_eff) * (x * x) + 3.1415926535897931 * fabs(sin(a_eff)) *
      sin(a_eff) * cos(a_eff);
    *CD = (Cd0 + fabs(CLAlp * fabs(sin(a_eff)) * sin(a_eff) * cos(a_eff))) +
      3.1415926535897931 * fabs(pow(sin(a_eff), 3.0));
    *CN = CLAlp * sin(a_eff) * cos(a_eff) + 3.1415926535897931 * fabs(sin(a_eff))
      * sin(a_eff);
    *CM = -0.53407075111026481 * fabs(sin(a_eff)) * sin(a_eff);
  } else if ((alp > LowAlpEnd) && (alp < HighAlpStart)) {
    a_eff = LowAlpEnd - alp0_eff;
    x = sin(a_eff);
    b_x = cos(a_eff);
    c_x = sin(a_eff);
    d_x = sin(a_eff);
    e_x = cos(a_eff);
    f_x = CLAlp * fabs(sin(a_eff)) * sin(a_eff) * cos(a_eff) +
      3.1415926535897931 * pow(sin(a_eff), 3.0);
    alp0_eff = sin(a_eff);
    CT = cos(a_eff);
    HighAlpEnd_P = sin(a_eff);
    HighAlpEnd_N = sin(a_eff);
    LowAlpStart_P = sin(a_eff);
    LowAlpStart_N = sin(a_eff);
    a_eff = HighAlpStart + b_gamma;
    if ((cf / c == 1.0) || (cf / c == 0.0)) {
      b_gamma = Cd90;
    } else {
      b_gamma = HighAlpStart;
      c_sign(&b_gamma);
      if (b_gamma == 1.0) {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) + 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      } else {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) - 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      }
    }

    CN2 = b_gamma * (1.0 / (0.56 + 0.44 * sin(fabs(a_eff))) - 0.41 * (1.0 - exp(
      -17.0 / AR))) * sin(fabs(a_eff));
    b_gamma = 0.5 * Cd0 * cos(a_eff);
    b_LowAlpStart_P[0] = LowAlpEnd;
    b_LowAlpStart_P[1] = HighAlpStart;
    b_CLAlp[0] = CLAlp * alp0_eff * CT + 3.1415926535897931 * fabs(HighAlpEnd_P)
      * HighAlpEnd_N;
    b_CLAlp[1] = CN2;
    *CN = c_interp1(b_LowAlpStart_P, b_CLAlp, alp);
    c_LowAlpStart_P[0] = LowAlpEnd;
    c_LowAlpStart_P[1] = HighAlpStart;
    CN1[0] = CLAlp * x * (b_x * b_x) + 3.1415926535897931 * fabs(c_x) * d_x *
      e_x;
    CN1[1] = CN2 * cos(a_eff) - b_gamma * sin(a_eff);
    *CL = c_interp1(c_LowAlpStart_P, CN1, alp);
    d_LowAlpStart_P[0] = LowAlpEnd;
    d_LowAlpStart_P[1] = HighAlpStart;
    b_Cd0[0] = Cd0 + fabs(f_x);
    b_Cd0[1] = CN2 * sin(a_eff) + b_gamma * cos(a_eff);
    *CD = c_interp1(d_LowAlpStart_P, b_Cd0, alp);
    e_LowAlpStart_P[0] = LowAlpEnd;
    e_LowAlpStart_P[1] = HighAlpStart;
    dv35[0] = -0.53407075111026481 * fabs(LowAlpStart_P) * LowAlpStart_N;
    dv35[1] = -(0.25 - 0.175 * (1.0 - fabs(a_eff) / 1.5707963267948966)) * CN2;
    *CM = c_interp1(e_LowAlpStart_P, dv35, alp);
  } else if ((alp >= HighAlpStart) && (alp <= HighAlpEnd_P)) {
    a_eff = alp + b_gamma;
    if ((cf / c == 1.0) || (cf / c == 0.0)) {
      b_gamma = Cd90;
    } else {
      b_gamma = alp;
      c_sign(&b_gamma);
      if (b_gamma == 1.0) {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) + 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      } else {
        y = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (y * y) - 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      }
    }

    *CN = b_gamma * (1.0 / (0.56 + 0.44 * sin(fabs(a_eff))) - 0.41 * (1.0 - exp(
      -17.0 / AR))) * sin(fabs(a_eff));
    CT = 0.5 * Cd0 * cos(a_eff);
    *CL = *CN * cos(a_eff) - CT * sin(a_eff);
    *CD = *CN * sin(a_eff) + CT * cos(a_eff);
    *CM = -(0.25 - 0.175 * (1.0 - fabs(a_eff) / 1.5707963267948966)) * *CN;
  } else if ((alp > HighAlpEnd_P) && (alp < LowAlpStart_P)) {
    a_eff = (LowAlpStart_P - alp0_eff) - 3.1415926535897931;
    x = sin(a_eff);
    b_x = cos(a_eff);
    c_x = sin(a_eff);
    d_x = sin(a_eff);
    e_x = cos(a_eff);
    f_x = CLAlp * fabs(sin(a_eff)) * sin(a_eff) * cos(a_eff) +
      3.1415926535897931 * pow(sin(a_eff), 3.0);
    alp0_eff = -(CLAlp * sin(a_eff) * cos(a_eff) + 3.1415926535897931 * fabs(sin
                  (a_eff)) * sin(a_eff));
    y = fabs(a_eff - 3.1415926535897931);
    a_eff = HighAlpEnd_P + b_gamma;
    if ((cf / c == 1.0) || (cf / c == 0.0)) {
      b_gamma = Cd90;
    } else {
      b_gamma = HighAlpEnd_P;
      c_sign(&b_gamma);
      if (b_gamma == 1.0) {
        CT = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (CT * CT) + 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      } else {
        CT = def * 180.0 / 3.1415926535897931;
        b_gamma = (-1.2963E-5 * (CT * CT) - 0.00361111 * (def * 180.0 /
                    3.1415926535897931)) + Cd90;
      }
    }

    CN2 = b_gamma * (1.0 / (0.56 + 0.44 * sin(fabs(a_eff))) - 0.41 * (1.0 - exp(
      -17.0 / AR))) * sin(fabs(a_eff));
    CT = 0.5 * Cd0 * cos(a_eff);
    b_LowAlpStart_P[0] = LowAlpStart_P;
    b_LowAlpStart_P[1] = HighAlpEnd_P;
    CN1[0] = alp0_eff;
    CN1[1] = CN2;
    *CN = c_interp1(b_LowAlpStart_P, CN1, alp);
    c_LowAlpStart_P[0] = LowAlpStart_P;
    c_LowAlpStart_P[1] = HighAlpEnd_P;
    b_CLAlp[0] = CLAlp * x * (b_x * b_x) + 3.1415926535897931 * fabs(c_x) * d_x *
      e_x;
    b_CLAlp[1] = CN2 * cos(a_eff) - CT * sin(a_eff);
    *CL = c_interp1(c_LowAlpStart_P, b_CLAlp, alp);
    d_LowAlpStart_P[0] = LowAlpStart_P;
    d_LowAlpStart_P[1] = HighAlpEnd_P;
    b_Cd0[0] = Cd0 + fabs(f_x);
    b_Cd0[1] = CN2 * sin(a_eff) + CT * cos(a_eff);
    *CD = c_interp1(d_LowAlpStart_P, b_Cd0, alp);
    e_LowAlpStart_P[0] = LowAlpStart_P;
    e_LowAlpStart_P[1] = HighAlpEnd_P;
    dv35[0] = -(0.25 - 0.175 * (1.0 - y / 1.5707963267948966)) * alp0_eff;
    dv35[1] = -(0.25 - 0.175 * (1.0 - fabs(a_eff) / 1.5707963267948966)) * CN2;
    *CM = c_interp1(e_LowAlpStart_P, dv35, alp);
  } else {
    a_eff = (alp - alp0_eff) - 3.1415926535897931;
    x = cos(a_eff);
    *CL = CLAlp * sin(a_eff) * (x * x) + 3.1415926535897931 * fabs(sin(a_eff)) *
      sin(a_eff) * cos(a_eff);
    *CD = Cd0 + fabs(CLAlp * fabs(sin(a_eff)) * sin(a_eff) * cos(a_eff) +
                     3.1415926535897931 * pow(sin(a_eff), 3.0));
    *CN = -(CLAlp * sin(a_eff) * cos(a_eff) + 3.1415926535897931 * fabs(sin
             (a_eff)) * sin(a_eff));
    *CM = -(0.25 - 0.175 * (1.0 - fabs(a_eff - 3.1415926535897931) /
             1.5707963267948966)) * *CN;
  }
}

static void b_atan2(const double y[7], const double x[7], double r[7])
{
  int k;
  for (k = 0; k < 7; k++) {
    r[k] = atan2(y[k], x[k]);
  }
}

static int b_bsearch(const double x[10], double xi)
{
  int n;
  int low_ip1;
  int high_i;
  int mid_i;
  n = 1;
  low_ip1 = 2;
  high_i = 10;
  while (high_i > low_ip1) {
    mid_i = (n + high_i) >> 1;
    if (xi >= x[mid_i - 1]) {
      n = mid_i;
      low_ip1 = mid_i + 1;
    } else {
      high_i = mid_i;
    }
  }

  return n;
}

static void b_cos(double x[7])
{
  int k;
  for (k = 0; k < 7; k++) {
    x[k] = cos(x[k]);
  }
}

static double b_interp1(const double varargin_1[10], const double varargin_2[10],
  double varargin_3)
{
  double Vq;
  double y[10];
  double x[10];
  int n;
  double r;
  for (n = 0; n < 10; n++) {
    y[n] = varargin_2[n];
    x[n] = varargin_1[n];
  }

  if (varargin_1[1] < varargin_1[0]) {
    for (n = 0; n < 5; n++) {
      r = x[n];
      x[n] = x[9 - n];
      x[9 - n] = r;
    }

    for (n = 0; n < 5; n++) {
      r = y[n];
      y[n] = y[9 - n];
      y[9 - n] = r;
    }
  }

  Vq = 0.0;
  if ((varargin_3 > x[9]) || (varargin_3 < x[0])) {
  } else {
    n = b_bsearch(x, varargin_3) - 1;
    r = (varargin_3 - x[n]) / (x[n + 1] - x[n]);
    if (y[n] == y[n + 1]) {
      Vq = y[n];
    } else {
      Vq = (1.0 - r) * y[n] + r * y[n + 1];
    }
  }

  return Vq;
}

static void b_power(const double a[3], double y[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    y[k] = a[k] * a[k];
  }
}

static void b_sign(double x[4])
{
  int k;
  double b_x;
  for (k = 0; k < 4; k++) {
    if (x[k] < 0.0) {
      b_x = -1.0;
    } else if (x[k] > 0.0) {
      b_x = 1.0;
    } else {
      b_x = x[k];
    }

    x[k] = b_x;
  }
}

static void b_sin(double x[7])
{
  int k;
  for (k = 0; k < 7; k++) {
    x[k] = sin(x[k]);
  }
}

static void b_sqrt(const double x[7], double b_x[7])
{
  int i0;
  for (i0 = 0; i0 < 7; i0++) {
    b_x[i0] = x[i0];
  }

  e_sqrt(b_x);
}

static double b_sum(const double x[3])
{
  double y;
  int k;
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  return y;
}

static void c_atan2(const double y[3], const double x[3], double r[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    r[k] = atan2(y[k], x[k]);
  }
}

static void c_cos(double x[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    x[k] = cos(x[k]);
  }
}

static double c_interp1(const double varargin_1[2], const double varargin_2[2],
  double varargin_3)
{
  double Vq;
  double y[2];
  double x[2];
  int i3;
  double r;
  for (i3 = 0; i3 < 2; i3++) {
    y[i3] = varargin_2[i3];
    x[i3] = varargin_1[i3];
  }

  if (varargin_1[1] < varargin_1[0]) {
    x[0] = varargin_1[1];
    x[1] = varargin_1[0];
    y[0] = varargin_2[1];
    y[1] = varargin_2[0];
  }

  Vq = 0.0;
  if ((varargin_3 > x[1]) || (varargin_3 < x[0])) {
  } else {
    r = (varargin_3 - x[0]) / (x[1] - x[0]);
    if (y[0] == y[1]) {
      Vq = y[0];
    } else {
      Vq = (1.0 - r) * y[0] + r * y[1];
    }
  }

  return Vq;
}

static void c_power(const double a[4], double y[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    y[k] = a[k] * a[k];
  }
}

static void c_sign(double *x)
{
  if (*x < 0.0) {
    *x = -1.0;
  } else {
    if (*x > 0.0) {
      *x = 1.0;
    }
  }
}

static void c_sin(double x[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    x[k] = sin(x[k]);
  }
}

static void c_sqrt(const double x[3], double b_x[3])
{
  int i1;
  for (i1 = 0; i1 < 3; i1++) {
    b_x[i1] = x[i1];
  }

  f_sqrt(b_x);
}

static double c_sum(const double x[4])
{
  double y;
  int k;
  y = x[0];
  for (k = 0; k < 3; k++) {
    y += x[k + 1];
  }

  return y;
}

static void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

static void d_atan2(const double y[4], const double x[4], double r[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    r[k] = atan2(y[k], x[k]);
  }
}

static void d_cos(double x[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    x[k] = cos(x[k]);
  }
}

static void d_sin(double x[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    x[k] = sin(x[k]);
  }
}

static void d_sqrt(const double x[4], double b_x[4])
{
  int i2;
  for (i2 = 0; i2 < 4; i2++) {
    b_x[i2] = x[i2];
  }

  g_sqrt(b_x);
}

static void e_sqrt(double x[7])
{
  int k;
  for (k = 0; k < 7; k++) {
    x[k] = sqrt(x[k]);
  }
}

static void f_sqrt(double x[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    x[k] = sqrt(x[k]);
  }
}

static void g_sqrt(double x[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    x[k] = sqrt(x[k]);
  }
}

static double interp1(const double varargin_1[10], const double varargin_2[10],
                      double varargin_3)
{
  double Vq;
  double y[10];
  double x[10];
  int i;
  double r;
  int low_ip1;
  int high_i;
  int mid_i;
  for (i = 0; i < 10; i++) {
    y[i] = varargin_2[i];
    x[i] = varargin_1[i];
  }

  if (varargin_1[1] < varargin_1[0]) {
    for (i = 0; i < 5; i++) {
      r = x[i];
      x[i] = x[9 - i];
      x[9 - i] = r;
    }

    for (i = 0; i < 5; i++) {
      r = y[i];
      y[i] = y[9 - i];
      y[9 - i] = r;
    }
  }

  Vq = 0.0;
  if ((varargin_3 > x[9]) || (varargin_3 < x[0])) {
  } else {
    i = 1;
    low_ip1 = 2;
    high_i = 10;
    while (high_i > low_ip1) {
      mid_i = (i + high_i) >> 1;
      if (varargin_3 >= x[mid_i - 1]) {
        i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    r = (varargin_3 - x[i - 1]) / (x[i] - x[i - 1]);
    if (y[i - 1] == y[i]) {
      Vq = y[i - 1];
    } else {
      Vq = (1.0 - r) * y[i - 1] + r * y[i];
    }
  }

  return Vq;
}

static double interp2_dispatch(const double V[110], double Xq, double Yq, double
  extrapval)
{
  double Vq;
  int low_i;
  int low_ip1;
  int high_i;
  int mid_i;
  static const double dv30[11] = { 0.0, 0.1, 0.2, 0.30000000000000004, 0.4, 0.5,
    0.6, 0.7, 0.8, 0.9, 1.0 };

  double dv31[10];
  double rx;
  double ry;
  if ((Xq >= 0.0) && (Xq <= 1.0) && (Yq >= 0.0) && (Yq <= 1.5707963267948966)) {
    low_i = 1;
    low_ip1 = 2;
    high_i = 11;
    while (high_i > low_ip1) {
      mid_i = (low_i + high_i) >> 1;
      if (Xq >= dv30[mid_i - 1]) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    for (low_ip1 = 0; low_ip1 < 10; low_ip1++) {
      dv31[low_ip1] = 0.17453292519943295 * (double)low_ip1;
    }

    low_ip1 = b_bsearch(dv31, Yq);
    rx = (Xq - dv30[low_i - 1]) / (dv30[low_i] - dv30[low_i - 1]);
    if (V[(low_ip1 + 10 * (low_i - 1)) - 1] == V[(low_ip1 + 10 * low_i) - 1]) {
      Vq = V[(low_ip1 + 10 * (low_i - 1)) - 1];
    } else {
      Vq = (1.0 - rx) * V[(low_ip1 + 10 * (low_i - 1)) - 1] + rx * V[(low_ip1 +
        10 * low_i) - 1];
    }

    if (V[low_ip1 + 10 * (low_i - 1)] == V[low_ip1 + 10 * low_i]) {
      rx = V[low_ip1 + 10 * (low_i - 1)];
    } else {
      rx = (1.0 - rx) * V[low_ip1 + 10 * (low_i - 1)] + rx * V[low_ip1 + 10 *
        low_i];
    }

    if (Vq == rx) {
    } else {
      ry = (Yq - 0.17453292519943295 * (double)(low_ip1 - 1)) /
        (0.17453292519943295 * (double)low_ip1 - 0.17453292519943295 * (double)
         (low_ip1 - 1));
      Vq = (1.0 - ry) * Vq + ry * rx;
    }
  } else {
    Vq = extrapval;
  }

  return Vq;
}

static double norm(const double x[3])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 3; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static void power(const double a[7], double y[7])
{
  int k;
  for (k = 0; k < 7; k++) {
    y[k] = a[k] * a[k];
  }
}

static double sum(const double x[7])
{
  double y;
  int k;
  y = x[0];
  for (k = 0; k < 6; k++) {
    y += x[k + 1];
  }

  return y;
}

void McFoamy_FM_v2(double LAilDef, double ElevDef, double RudDef, double wIn,
                   double u, double v, double w, double p, double q, double r,
                   double *Fx, double *Fy, double *Fz, double *Mx, double *My,
                   double *Mz)
{
  double Vel[3];
  double B_rate[3];
  double rods[224];
  static const double b_rods[224] = { -80.0, -255.0, -328.0, -425.0, -540.0,
    -645.0, -745.0, -80.0, -230.0, -230.0, -230.0, -330.0, -445.0, -540.0,
    -655.0, -720.0, -80.0, -255.0, -328.0, -425.0, -540.0, -645.0, -745.0, -80.0,
    -230.0, -230.0, -230.0, -330.0, -445.0, -540.0, -655.0, -720.0, 48.0, 0.0,
    54.0, 0.0, 38.0, 0.0, 0.0, 48.0, 0.0, 0.0, 0.0, 55.0, 0.0, 35.0, 0.0, 115.0,
    -48.0, 0.0, -54.0, 0.0, -38.0, 0.0, 0.0, -48.0, 0.0, 0.0, 0.0, -55.0, 0.0,
    -35.0, 0.0, -115.0, 0.0, -80.0, 0.0, -85.0, 0.0, -60.0, -93.0, 0.0, 75.0,
    75.0, 75.0, 0.0, 65.0, 0.0, 55.0, 0.0, 0.0, -80.0, 0.0, -85.0, 0.0, -60.0,
    -93.0, 0.0, 75.0, 75.0, 75.0, 0.0, 65.0, 0.0, 55.0, 0.0, -255.0, -328.0,
    -425.0, -540.0, -645.0, -695.0, -720.0, -230.0, -185.0, -265.0, -330.0,
    -445.0, -540.0, -655.0, -695.0, -745.0, -255.0, -328.0, -425.0, -540.0,
    -645.0, -695.0, -720.0, -230.0, -185.0, -265.0, -330.0, -445.0, -540.0,
    -655.0, -695.0, -745.0, 0.0, 54.0, 0.0, 38.0, 0.0, 115.0, 115.0, 0.0, 210.0,
    210.0, 55.0, 0.0, 35.0, 0.0, 115.0, 0.0, 0.0, -54.0, 0.0, -38.0, 0.0, -115.0,
    -115.0, 0.0, -210.0, -210.0, -55.0, 0.0, -35.0, 0.0, -115.0, 0.0, -80.0, 0.0,
    -85.0, 0.0, -60.0, 0.0, 0.0, 75.0, 0.0, 0.0, 0.0, 65.0, 0.0, 55.0, 0.0, 65.0,
    -80.0, 0.0, -85.0, 0.0, -60.0, 0.0, 0.0, 75.0, 0.0, 0.0, 0.0, 65.0, 0.0,
    55.0, 0.0, 65.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    2.0, 2.0, 2.0, 2.0, 2.0 };

  int i;
  static const double dv0[192] = { 160.0, -15.0, -88.0, -185.0, -300.0, -405.0,
    -505.0, 160.0, 10.0, 10.0, 10.0, -90.0, -205.0, -300.0, -415.0, -480.0,
    160.0, -15.0, -88.0, -185.0, -300.0, -405.0, -505.0, 160.0, 10.0, 10.0, 10.0,
    -90.0, -205.0, -300.0, -415.0, -480.0, 48.0, 0.0, 54.0, 0.0, 38.0, 0.0, 0.0,
    48.0, 0.0, 0.0, 0.0, 55.0, 0.0, 35.0, 0.0, 115.0, -48.0, 0.0, -54.0, 0.0,
    -38.0, 0.0, 0.0, -48.0, 0.0, 0.0, 0.0, -55.0, 0.0, -35.0, 0.0, -115.0, -6.61,
    -86.61, -6.61, -91.61, -6.61, -66.61, -99.61, -6.61, 68.39, 68.39, 68.39,
    -6.61, 58.39, -6.61, 48.39, -6.61, -6.61, -86.61, -6.61, -91.61, -6.61,
    -66.61, -99.61, -6.61, 68.39, 68.39, 68.39, -6.61, 58.39, -6.61, 48.39,
    -6.61, -15.0, -88.0, -185.0, -300.0, -405.0, -455.0, -480.0, 10.0, 55.0,
    -25.0, -90.0, -205.0, -300.0, -415.0, -455.0, -505.0, -15.0, -88.0, -185.0,
    -300.0, -405.0, -455.0, -480.0, 10.0, 55.0, -25.0, -90.0, -205.0, -300.0,
    -415.0, -455.0, -505.0, 0.0, 54.0, 0.0, 38.0, 0.0, 115.0, 115.0, 0.0, 210.0,
    210.0, 55.0, 0.0, 35.0, 0.0, 115.0, 0.0, 0.0, -54.0, 0.0, -38.0, 0.0, -115.0,
    -115.0, 0.0, -210.0, -210.0, -55.0, 0.0, -35.0, 0.0, -115.0, 0.0, -86.61,
    -6.61, -91.61, -6.61, -66.61, -6.61, -6.61, 68.39, -6.61, -6.61, -6.61,
    58.39, -6.61, 48.39, -6.61, 58.39, -86.61, -6.61, -91.61, -6.61, -66.61,
    -6.61, -6.61, 68.39, -6.61, -6.61, -6.61, 58.39, -6.61, 48.39, -6.61, 58.39
  };

  double vthr_x;
  double vthr_y;
  double vthr_z;
  double psiT;
  double deltaT;
  double wOut;
  double CFx;
  double CFy;
  double CMy;
  double CMz;
  static const double dv1[110] = { 0.156336454, 0.156336454, 0.156336454,
    0.156336454, 0.156336454, 0.156336454, 0.156336454, 0.156336454, 0.156336454,
    0.156336454, 0.142939755, 0.143274298, 0.144236696, 0.145715204, 0.147592844,
    0.149730239, 0.151960378, 0.154120349, 0.156099101, 0.157836404, 0.12449467,
    0.125294853, 0.127662714, 0.131491476, 0.136533063, 0.142227959, 0.148100411,
    0.153647038, 0.158176177, 0.161723393, 0.103654093, 0.104978752, 0.10890636,
    0.115283154, 0.123846679, 0.133924798, 0.144287776, 0.153875413, 0.161071479,
    0.163852071, 0.080437634, 0.082320505, 0.087900516, 0.096990697, 0.109258684,
    0.124116893, 0.139693524, 0.153953862, 0.163349211, 0.165797264, 0.054966543,
    0.057426888, 0.064729471, 0.076657686, 0.092816061, 0.112598271, 0.133992265,
    0.153533548, 0.165846339, 0.168474477, 0.027444178, 0.03049664, 0.039588216,
    0.054473185, 0.074693054, 0.099537509, 0.127172755, 0.152570626, 0.168638349,
    0.171699702, -0.001874328, 0.001785855, 0.012726044, 0.030695378, 0.05514811,
    0.085260174, 0.119365045, 0.151139537, 0.171659606, 0.175259993,
    -0.032728784, -0.02842523, -0.015539759, 0.005645211, 0.034482474,
    0.070037437, 0.11072865, 0.149362748, 0.174874022, 0.179027738, -0.064814458,
    -0.059769732, -0.044818343, -0.020318541, 0.013005062, 0.054122603,
    0.101419857, 0.147360112, 0.17827724, 0.182929631, -0.097502535,
    -0.091725529, -0.074619748, -0.046776089, -0.008957689, 0.037754768,
    0.091641937, 0.145232828, 0.181871525, 0.18696977 };

  static const double dv2[110] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.000304035, 0.000593774, 0.000855707, 0.001081293, 0.001265997,
    0.001410344, 0.001515466, 0.001581189, 0.00160638, 0.0, 0.000672049,
    0.001308186, 0.001875984, 0.002343891, 0.002685232, 0.002915524, 0.003073033,
    0.003203695, 0.003296003, 0.0, 0.001105525, 0.002139399, 0.003046099,
    0.003777072, 0.004271303, 0.004529052, 0.004674445, 0.004870829, 0.005159087,
    0.0, 0.001600104, 0.003086457, 0.004372415, 0.005383801, 0.006042688,
    0.006286839, 0.006333996, 0.006596278, 0.007149694, 0.0, 0.00214306,
    0.004124254, 0.005829393, 0.007152135, 0.007991019, 0.008217389, 0.008079318,
    0.008340642, 0.00921631, 0.0, 0.002695169, 0.005206331, 0.007370573,
    0.009046337, 0.010092937, 0.010327588, 0.009939438, 0.010095901, 0.01134185,
    0.0, 0.003243617, 0.0062742, 0.008920992, 0.010988567, 0.012301024,
    0.012588497, 0.01193205, 0.011869291, 0.013520997, 0.0, 0.003737499,
    0.007197636, 0.010310416, 0.012837704, 0.014518175, 0.014938663, 0.014050094,
    0.013673189, 0.015752967, 0.0, 0.004001698, 0.007738064, 0.011301405,
    0.014405295, 0.016623841, 0.017317246, 0.016267158, 0.015518142, 0.018038805,
    0.0, 0.003755281, 0.007554928, 0.011576152, 0.015455208, 0.018472368,
    0.019651635, 0.018549771, 0.017413651, 0.020375896 };

  static const double dv3[110] = { -0.009379255, -0.009379255, -0.009379255,
    -0.009379255, -0.009379255, -0.009379255, -0.009379255, -0.009379255,
    -0.009379255, -0.009379255, -0.009414602, -0.009414258, -0.009412543,
    -0.009408081, -0.009399845, -0.009386461, -0.009367587, -0.009343565,
    -0.009317827, -0.009292775, -0.00922517, -0.009234779, -0.009259831,
    -0.009292432, -0.0093192, -0.009327092, -0.009306845, -0.009256742,
    -0.009186392, -0.009135945, -0.008715215, -0.008749876, -0.008846651,
    -0.008982547, -0.009121875, -0.009224484, -0.009256742, -0.009206982,
    -0.009150015, -0.009359351, -0.007778011, -0.007856254, -0.008075885,
    -0.008395035, -0.008745071, -0.009041916, -0.009209727, -0.009213159,
    -0.00927596, -0.009773903, -0.006305113, -0.0064465, -0.006846983,
    -0.007438956, -0.008111231, -0.008720706, -0.009127023, -0.00925331,
    -0.009427299, -0.01020081, -0.004198032, -0.004423154, -0.005068319,
    -0.006031948, -0.007148975, -0.008203888, -0.008967104, -0.009300668,
    -0.009571088, -0.010605068, -0.001374064, -0.001706255, -0.002664393,
    -0.004110867, -0.005807856, -0.007449251, -0.00869943, -0.009331211,
    -0.009705269, -0.010994226, 0.002229593, 0.001762192, 0.00040563,
    -0.001648602, -0.004067627, -0.006438264, -0.008307869, -0.009330867,
    -0.009834645, -0.011382011, 0.006644511, 0.00599214, 0.004131457,
    0.001338374, -0.001941328, -0.005174359, -0.007785904, -0.009294491,
    -0.009963678, -0.011781807, 0.011790387, 0.01091461, 0.008431755,
    0.004772847, 0.000516475, -0.003684646, -0.007140396, -0.009222768,
    -0.010097172, -0.012199449 };

  static const double dv4[110] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.000624918, 0.001228902, 0.001786557, 0.002280383, 0.002695622,
    0.003025754, 0.003265632, 0.003407706, 0.003447857, 0.0, 0.00118875,
    0.002355195, 0.003475311, 0.004508604, 0.005373056, 0.006031948, 0.006465718,
    0.006690153, 0.006735452, 0.0, 0.001686008, 0.003352112, 0.004976005,
    0.006530234, 0.007916309, 0.008970193, 0.009615701, 0.009827095, 0.009570745,
    0.0, 0.00210811, 0.004209014, 0.006291729, 0.008331891, 0.010270131,
    0.01181784, 0.012740975, 0.012792794, 0.012191899, 0.0, 0.002447508,
    0.004910802, 0.007398462, 0.009900534, 0.012377212, 0.01452513, 0.015837766,
    0.015769475, 0.014745447, 0.0, 0.002695622, 0.005441004, 0.008275611,
    0.011210425, 0.014212843, 0.017039214, 0.018875875, 0.018797288, 0.017259188,
    0.0, 0.002844902, 0.005784863, 0.008902245, 0.012237884, 0.015771877,
    0.019318224, 0.021818238, 0.021878979, 0.019738611, 0.0, 0.002886426,
    0.005919044, 0.009246104, 0.012952713, 0.017029948, 0.021326814, 0.024630882,
    0.025002195, 0.022188521, 0.0, 0.00279274, 0.005797904, 0.009258801,
    0.013314074, 0.017960633, 0.023040962, 0.027285666, 0.028151834, 0.024615439,
    0.0, 0.002507907, 0.005340455, 0.008857975, 0.013262255, 0.018530643,
    0.024451059, 0.029763374, 0.031305935, 0.027038239 };

  static const double dv5[110] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.000466715, 0.000928969, 0.001379898, 0.001812295, 0.002217926,
    0.002588209, 0.002912507, 0.003173662, 0.003355543, 0.0, 0.000650999,
    0.001316754, 0.002009277, 0.002733028, 0.003466731, 0.004186021, 0.004849031,
    0.005419041, 0.005744369, 0.0, 0.000651342, 0.001337687, 0.002091295,
    0.002939961, 0.003887461, 0.004884035, 0.005825014, 0.00627663, 0.005094056,
    0.0, 0.000542556, 0.001141049, 0.001847642, 0.002706947, 0.003752251,
    0.004944433, 0.006105387, 0.005895365, 0.003495558, 0.0, 0.000378176,
    0.000833567, 0.001435835, 0.002244007, 0.003306126, 0.004618762, 0.005966058,
    0.005365163, 0.001968096, 0.0, 0.000193549, 0.00048456, 0.000959511,
    0.001687037, 0.002723076, 0.004095081, 0.005597491, 0.004922813, 0.000671589,
    0.0, 7.5498E-6, 0.000133151, 0.000477353, 0.001115998, 0.002106738,
    0.003494871, 0.005116363, 0.004597142, -0.000392246, 0.0, -0.00017193,
    -0.000201786, 1.9904E-5, 0.000571383, 0.001510303, 0.002888485, 0.004593367,
    0.004365844, -0.001249149, 0.0, -0.00033528, -0.000507209, -0.000396021,
    7.41253E-5, 0.000958825, 0.002311955, 0.004069343, 0.004200435, -0.001923827,
    0.0, -0.00047049, -0.000764589, -0.000754637, -0.000362734, 0.000464313,
    0.001782782, 0.003567967, 0.004069686, -0.002446135 };

  double dv6[10];
  static const double dv7[10] = { -0.097502535, -0.091725529, -0.074619748,
    -0.046776089, -0.008957689, 0.037754768, 0.091641937, 0.145232828,
    0.181871525, 0.18696977 };

  static const double dv8[10] = { 0.0, 0.003755281, 0.007554928, 0.011576152,
    0.015455208, 0.018472368, 0.019651635, 0.018549771, 0.017413651, 0.020375896
  };

  static const double dv9[10] = { 0.011790387, 0.01091461, 0.008431755,
    0.004772847, 0.000516475, -0.003684646, -0.007140396, -0.009222768,
    -0.010097172, -0.012199449 };

  static const double dv10[10] = { 0.0, 0.002507907, 0.005340455, 0.008857975,
    0.013262255, 0.018530643, 0.024451059, 0.029763374, 0.031305935, 0.027038239
  };

  static const double dv11[10] = { 0.0, -0.00047049, -0.000764589, -0.000754637,
    -0.000362734, 0.000464313, 0.001782782, 0.003567967, 0.004069686,
    -0.002446135 };

  double y;
  double b_y;
  double MX;
  double c_y;
  double ThrForce_idx_0;
  double ThrForce_idx_1;
  double VProp_Axial[50];
  static const double dv12[50] = { 0.21706999999999999, 0.21139, 0.21543,
    0.21218, 0.20885, 0.20550000000000002, 0.20226, 0.71127, 0.71972, 0.71515,
    0.73394, 0.73581, 0.74414, 0.76076, 0.20509, 0.20509, 0.20509, 0.20509,
    0.1675, 0.2915, 0.3765, 0.4825, 0.5925, 0.67, 0.7325, 0.155,
    0.20750000000000002, 0.2475, 0.28, 0.3875, 0.4925, 0.5975, 0.675, 0.7325,
    0.1675, 0.2915, 0.3765, 0.4825, 0.5925, 0.67, 0.7325, 0.155,
    0.20750000000000002, 0.2475, 0.28, 0.3875, 0.4925, 0.5975, 0.675, 0.7325 };

  static const double dv13[50] = { 0.03338, 0.091760000000000008, 0.15114,
    0.21201, 0.27455, 0.33742, 0.40021, 0.03953, 0.09333, 0.15156999999999998,
    0.02354, 0.02996, 0.094629999999999992, 0.14856, 0.053810000000000004,
    0.01794, 0.01863, 0.05589, 0.046647615158762409, 0.048259714048054618,
    0.050351266121121521, 0.0465537323960174, 0.035510561809129405,
    0.064855608855364233, 0.0739493069609175, 0.044522466238967488,
    0.1114955156048888, 0.1114955156048888, 0.046502688094345684,
    0.042573465914816006, 0.0369120576505835, 0.032596012026013241,
    0.063737743919909814, 0.066049224068114532, 0.046647615158762409,
    0.048259714048054618, 0.050351266121121521, 0.0465537323960174,
    0.035510561809129405, 0.064855608855364233, 0.0739493069609175,
    0.044522466238967488, 0.1114955156048888, 0.1114955156048888,
    0.046502688094345684, 0.042573465914816006, 0.0369120576505835,
    0.032596012026013241, 0.063737743919909814, 0.066049224068114532 };

  double vws_xcomp[7];
  double vws_zcomp[7];
  static const double b[7] = { 0.03338, 0.091760000000000008, 0.15114, 0.21201,
    0.27455, 0.33742, 0.40021 };

  static const double b_b[7] = { 0.022930000000000006, 0.028609999999999997,
    0.024569999999999981, 0.027819999999999984, 0.031149999999999983,
    0.034499999999999975, 0.037739999999999996 };

  double dv14[7];
  double CM_ws[7];
  double Fz_ws[7];
  double CL_ws[7];
  double unusedExpr[7];
  double vws_xz[7];
  double vwp_xcomp[7];
  double vwp_zcomp[7];
  static const double c_b[7] = { -0.03338, -0.091760000000000008, -0.15114,
    -0.21201, -0.27455, -0.33742, -0.40021 };

  double b_unusedExpr[7];
  double vwp_xz[7];
  double vts_xcomp[3];
  double vts_zcomp[3];
  static const double d_b[3] = { 0.03953, 0.09333, 0.15156999999999998 };

  static const double e_b[3] = { -0.47126999999999997, -0.47972000000000004,
    -0.47514999999999996 };

  double li[3];
  double ri[3];
  double V_l_l[3];
  double CL_ts[3];
  double c_unusedExpr[3];
  double vts_xz[3];
  double vtp_xcomp[3];
  double vtp_zcomp[3];
  static const double f_b[3] = { -0.03953, -0.09333, -0.15156999999999998 };

  double d_unusedExpr[3];
  double vtp_xz[3];
  double vr_xcomp[4];
  double dv15[4];
  static const double g_b[4] = { 0.016929999999999997, -0.03657, -0.10124,
    -0.15517 };

  static const double dv16[4] = { 0.02354, -0.029959999999999997,
    -0.094629999999999992, -0.14856 };

  double vr_ycomp[4];
  static const double h_b[4] = { -0.49394000000000005, -0.49581, -0.50414,
    -0.52076 };

  double dv17[4];
  double a_B[4];
  double CL_r[4];
  double e_unusedExpr[4];
  double vr_xy[4];
  double vB_xcomp[4];
  static const double i_b[4] = { 0.047200000000000006, 0.01133,
    -0.025240000000000002, -0.0625 };

  static const double dv18[4] = { 0.053810000000000004, 0.01794, -0.01863,
    -0.05589 };

  double vB_ycomp[4];
  double f_unusedExpr[4];
  double vB_xy[4];
  double CD_wp[7];
  double CM_wp[7];
  int b_i;
  static const double dv19[7] = { 0.2534, 0.23758, 0.22269, 0.20742,
    0.19174000000000002, 0.17597000000000002, 0.16028 };

  static const double dv20[7] = { 0.0, 0.10204, 0.099230000000000013, 0.09636,
    0.093409999999999993, 0.09044, 0.08749 };

  double CM_ts[3];
  double CL_tp[3];
  double CD_tp[3];
  double CM_tp[3];
  static const double dv21[3] = { 0.11741, 0.14564, 0.14316 };

  static const double dv22[3] = { 0.081870000000000012, 0.11148999999999999,
    0.14316 };

  double CM_r[4];
  static const double dv23[4] = { 0.20062000000000002, 0.19294999999999998,
    0.16949, 0.14114 };

  static const double dv24[4] = { 0.12036000000000001, 0.11269,
    0.10343000000000001, 0.14114 };

  double CM_B[4];
  double D_rods[3];
  double M_rods[3];
  double b_VProp_Axial[3];
  double b_vthr_y;
  double dv25[3];
  double b_V_l_l[3];
  double Fx_ws[7];
  double Fx_wp[7];
  static const double dv26[7] = { 0.00888718145, 0.0085448022800000013,
    0.008087015186250001, 0.007950460455, 0.0073494421350000011,
    0.006817187781250001, 0.0060768158500000011 };

  static const double dv27[3] = { 0.0047858517437500006, 0.004557457905,
    0.0054382547100000015 };

  double Fx_r[4];
  double Fx_B[4];
  static const double dv28[4] = { 0.0056893324250000014, 0.00734973080625,
    0.007195253038750001, 0.003293678325 };

  double CtrlDef[4];
  static const double dv29[4] = { 0.01409860934125, 0.01409860934125,
    0.014644945192500002, 0.014644945192500002 };

  static const double Cw[7] = { 0.2534, 0.23758, 0.22269, 0.20742,
    0.19174000000000002, 0.17597000000000002, 0.16028 };

  static const double Ct[3] = { 0.11741, 0.14564, 0.14316 };

  static const double Cr[4] = { 0.20062000000000002, 0.19294999999999998,
    0.16949, 0.14114 };

  Vel[0] = u;
  Vel[1] = v;
  Vel[2] = w;
  B_rate[0] = p;
  B_rate[1] = q;
  B_rate[2] = r;
  memcpy(&rods[0], &b_rods[0], 224U * sizeof(double));
  for (i = 0; i < 6; i++) {
    memcpy(&rods[i << 5], &dv0[i << 5], sizeof(double) << 5);
  }

  for (i = 0; i < 224; i++) {
    rods[i] *= 0.001;
  }

  vthr_x = u + q * -0.00661;
  vthr_y = (v + r * 0.24) - p * -0.00661;
  vthr_z = w - q * 0.24;
  psiT = atan2(sqrt(vthr_y * vthr_y + vthr_z * vthr_z), fabs(vthr_x));
  deltaT = atan2(vthr_z, vthr_y);
  if (wIn < 1716.0) {
    wOut = 0.0;
    CFx = 0.0;
    CFy = 0.0;
    vthr_z = 0.0;
    CMy = 0.0;
    CMz = 0.0;
  } else {
    wOut = wIn;
    vthr_y = sqrt((vthr_x * vthr_x + vthr_y * vthr_y) + vthr_z * vthr_z) / (wIn /
      60.0 * 0.254);
    if (vthr_x >= 0.0) {
      if (vthr_y <= 1.0) {
        CFx = interp2_dispatch(dv1, vthr_y, psiT, 0.0);
        CFy = interp2_dispatch(dv2, vthr_y, psiT, 0.0);
        vthr_z = interp2_dispatch(dv3, vthr_y, psiT, 0.0);
        CMy = interp2_dispatch(dv4, vthr_y, psiT, 0.0);
        CMz = interp2_dispatch(dv5, vthr_y, psiT, 0.0);
      } else {
        for (i = 0; i < 10; i++) {
          dv6[i] = 0.17453292519943295 * (double)i;
        }

        CFx = interp1(dv6, dv7, psiT);
        for (i = 0; i < 10; i++) {
          dv6[i] = 0.17453292519943295 * (double)i;
        }

        CFy = interp1(dv6, dv8, psiT);
        for (i = 0; i < 10; i++) {
          dv6[i] = 0.17453292519943295 * (double)i;
        }

        vthr_z = interp1(dv6, dv9, psiT);
        for (i = 0; i < 10; i++) {
          dv6[i] = 0.17453292519943295 * (double)i;
        }

        CMy = interp1(dv6, dv10, psiT);
        for (i = 0; i < 10; i++) {
          dv6[i] = 0.17453292519943295 * (double)i;
        }

        CMz = interp1(dv6, dv11, psiT);
      }
    } else {
      CFx = 0.15633;
      CFy = 0.0;
      vthr_z = -0.00938;
      CMy = interp2_dispatch(dv4, vthr_y, psiT, 0.0);
      CMz = interp2_dispatch(dv5, vthr_y, psiT, 0.0);
    }
  }

  y = wOut / 60.0;
  vthr_y = wOut / 60.0;
  psiT = wOut / 60.0;
  b_y = wOut / 60.0;
  MX = 1.225 * (b_y * b_y) * 0.001057227821024 * vthr_z;
  b_y = wOut / 60.0;
  c_y = wOut / 60.0;
  if ((vthr_x >= -0.2 * (0.5 * (1.59 * (wOut / 60.0) * 0.254 *
         0.39538588745679831))) && (CFx > 0.0)) {
    vthr_x = 0.5 * (1.59 * (wOut / 60.0) * 0.254 * sqrt(CFx));
    MX *= 0.2;
  } else {
    vthr_x = 0.0;
  }

  MX *= 0.4;
  ThrForce_idx_0 = 1.225 * (y * y) * 0.004162314256 * CFx;
  ThrForce_idx_1 = 0.5 * (1.225 * (vthr_y * vthr_y) * 0.004162314256 * (-CFy *
    cos(deltaT) + 0.0 * sin(deltaT)));
  CFy = 0.5 * (1.225 * (psiT * psiT) * 0.004162314256 * (-CFy * sin(deltaT) -
    0.0 * cos(deltaT)));
  vthr_z = 2.0 * vthr_x * 0.91823899371069173;
  for (i = 0; i < 50; i++) {
    if (dv12[i] < 0.194056) {
      y = dv12[i] / 0.127;
      vthr_y = vthr_x * (1.0 + dv12[i] / 0.127 / sqrt(1.0 + y * y));
    } else if (dv12[i] < 0.319532) {
      y = (dv13[i] - 0.058946599999999995 * (1.0 - 0.1294 * (dv12[i] - 0.194056)
            / 0.18796)) / (0.05210289974 + 0.1326 * ((dv12[i] - 0.194056) -
        0.09398));
      vthr_y = vthr_z * (1.24 - 0.0765 * (dv12[i] - 0.194056) / 0.18796) * exp
        (-(y * y));
    } else {
      y = (dv13[i] - 0.058946599999999995 * (1.3 - 0.3059 * (dv12[i] - 0.194056)
            / 0.18796)) / (0.030510760159999994 + 0.2295 * ((dv12[i] - 0.194056)
        - 0.09398));
      vthr_y = vthr_z * (1.37 - 0.1529 * (dv12[i] - 0.194056) / 0.18796) * exp
        (-(y * y));
    }

    if (vthr_y < 0.01) {
      vthr_y = 0.0;
    }

    VProp_Axial[i] = vthr_y;
  }

  for (i = 0; i < 7; i++) {
    vws_xcomp[i] = ((u + q * -0.00661) - r * b[i]) + VProp_Axial[i];
    vws_zcomp[i] = (w + p * b[i]) - q * b_b[i];
  }

  power(vws_xcomp, dv14);
  for (i = 0; i < 7; i++) {
    CM_ws[i] = (v + r * b_b[i]) - p * -0.00661;
  }

  power(CM_ws, Fz_ws);
  power(vws_zcomp, CL_ws);
  for (i = 0; i < 7; i++) {
    CM_ws[i] = (dv14[i] + Fz_ws[i]) + CL_ws[i];
  }

  b_sqrt(CM_ws, unusedExpr);
  power(vws_xcomp, vws_xz);
  power(vws_zcomp, dv14);
  for (i = 0; i < 7; i++) {
    vws_xz[i] += dv14[i];
  }

  e_sqrt(vws_xz);
  for (i = 0; i < 7; i++) {
    vwp_xcomp[i] = ((u + q * -0.00661) - r * c_b[i]) + VProp_Axial[i];
    vwp_zcomp[i] = (w + p * c_b[i]) - q * b_b[i];
  }

  power(vwp_xcomp, dv14);
  for (i = 0; i < 7; i++) {
    CM_ws[i] = (v + r * b_b[i]) - p * -0.00661;
  }

  power(CM_ws, Fz_ws);
  power(vwp_zcomp, CL_ws);
  for (i = 0; i < 7; i++) {
    CM_ws[i] = (dv14[i] + Fz_ws[i]) + CL_ws[i];
  }

  b_sqrt(CM_ws, b_unusedExpr);
  power(vwp_xcomp, vwp_xz);
  power(vwp_zcomp, dv14);
  for (i = 0; i < 7; i++) {
    vwp_xz[i] += dv14[i];
  }

  e_sqrt(vwp_xz);
  for (i = 0; i < 3; i++) {
    vts_xcomp[i] = ((u + q * -0.00661) - r * d_b[i]) + VProp_Axial[7 + i];
    vts_zcomp[i] = (w + p * d_b[i]) - q * e_b[i];
  }

  b_power(vts_xcomp, li);
  for (i = 0; i < 3; i++) {
    ri[i] = (v + r * e_b[i]) - p * -0.00661;
  }

  b_power(ri, V_l_l);
  b_power(vts_zcomp, CL_ts);
  for (i = 0; i < 3; i++) {
    ri[i] = (li[i] + V_l_l[i]) + CL_ts[i];
  }

  c_sqrt(ri, c_unusedExpr);
  b_power(vts_xcomp, vts_xz);
  b_power(vts_zcomp, li);
  for (i = 0; i < 3; i++) {
    vts_xz[i] += li[i];
  }

  f_sqrt(vts_xz);
  for (i = 0; i < 3; i++) {
    vtp_xcomp[i] = ((u + q * -0.00661) - r * f_b[i]) + VProp_Axial[7 + i];
    vtp_zcomp[i] = (w + p * f_b[i]) - q * e_b[i];
  }

  b_power(vtp_xcomp, li);
  for (i = 0; i < 3; i++) {
    ri[i] = (v + r * e_b[i]) - p * -0.00661;
  }

  b_power(ri, V_l_l);
  b_power(vtp_zcomp, CL_ts);
  for (i = 0; i < 3; i++) {
    ri[i] = (li[i] + V_l_l[i]) + CL_ts[i];
  }

  c_sqrt(ri, d_unusedExpr);
  b_power(vtp_xcomp, vtp_xz);
  b_power(vtp_zcomp, li);
  for (i = 0; i < 3; i++) {
    vtp_xz[i] += li[i];
  }

  f_sqrt(vtp_xz);
  for (i = 0; i < 4; i++) {
    vr_xcomp[i] = (u + q * g_b[i]) + VProp_Axial[10 + i];
    dv15[i] = dv16[i];
  }

  b_sign(dv15);
  for (i = 0; i < 4; i++) {
    vr_ycomp[i] = (v + r * h_b[i]) - p * g_b[i];
  }

  c_power(vr_xcomp, dv15);
  c_power(vr_ycomp, dv17);
  for (i = 0; i < 4; i++) {
    a_B[i] = w - q * h_b[i];
  }

  c_power(a_B, CL_r);
  for (i = 0; i < 4; i++) {
    a_B[i] = (dv15[i] + dv17[i]) + CL_r[i];
  }

  d_sqrt(a_B, e_unusedExpr);
  c_power(vr_xcomp, vr_xy);
  c_power(vr_ycomp, dv15);
  for (i = 0; i < 4; i++) {
    vr_xy[i] += dv15[i];
  }

  g_sqrt(vr_xy);
  for (i = 0; i < 4; i++) {
    vB_xcomp[i] = (u + q * i_b[i]) + VProp_Axial[14 + i];
    dv15[i] = dv18[i];
  }

  b_sign(dv15);
  for (i = 0; i < 4; i++) {
    vB_ycomp[i] = (v + r * 0.03491) - p * i_b[i];
  }

  c_power(vB_xcomp, dv15);
  c_power(vB_ycomp, dv17);
  for (i = 0; i < 4; i++) {
    a_B[i] = w - q * 0.03491;
  }

  c_power(a_B, CL_r);
  for (i = 0; i < 4; i++) {
    a_B[i] = (dv15[i] + dv17[i]) + CL_r[i];
  }

  d_sqrt(a_B, f_unusedExpr);
  c_power(vB_xcomp, vB_xy);
  c_power(vB_ycomp, dv15);
  for (i = 0; i < 4; i++) {
    vB_xy[i] += dv15[i];
  }

  g_sqrt(vB_xy);
  for (i = 0; i < 7; i++) {
    CM_ws[i] = vws_zcomp[i];
    CD_wp[i] = vwp_xcomp[i];
  }

  b_atan2(CM_ws, vws_xcomp, vws_zcomp);
  b_atan2(vwp_zcomp, CD_wp, vwp_xcomp);
  for (i = 0; i < 3; i++) {
    li[i] = vts_zcomp[i];
    ri[i] = vtp_zcomp[i];
  }

  c_atan2(li, vts_xcomp, vts_zcomp);
  c_atan2(ri, vtp_xcomp, vtp_zcomp);
  for (i = 0; i < 4; i++) {
    a_B[i] = vr_ycomp[i];
  }

  d_atan2(a_B, vr_xcomp, vr_ycomp);
  d_atan2(vB_ycomp, vB_xcomp, a_B);
  for (b_i = 0; b_i < 7; b_i++) {
    McFoamy_Airfoil_Simplified(vws_zcomp[b_i], dv20[b_i], dv19[b_i], -LAilDef,
      2.0920035851844676, 0.02, 1.98, 0.0, &CFx, &vthr_y, &vthr_z, &psiT);
    CL_ws[b_i] = vthr_y;
    vws_xcomp[b_i] = vthr_z;
    CM_ws[b_i] = psiT;
    McFoamy_Airfoil_Simplified(vwp_xcomp[b_i], dv20[b_i], dv19[b_i], -(-LAilDef),
      2.0920035851844676, 0.02, 1.98, 0.0, &CFx, &vthr_y, &vthr_z, &psiT);
    vwp_zcomp[b_i] = vthr_y;
    CD_wp[b_i] = vthr_z;
    CM_wp[b_i] = psiT;
  }

  for (b_i = 0; b_i < 3; b_i++) {
    McFoamy_Airfoil_Simplified(vts_zcomp[b_i], dv22[b_i], dv21[b_i], ElevDef,
      1.2420750265020692, 0.02, 1.98, 0.0, &CFx, &vthr_y, &vthr_z, &psiT);
    CL_ts[b_i] = vthr_y;
    vts_xcomp[b_i] = vthr_z;
    CM_ts[b_i] = psiT;
    McFoamy_Airfoil_Simplified(vtp_zcomp[b_i], dv22[b_i], dv21[b_i], ElevDef,
      1.2420750265020692, 0.02, 1.98, 0.0, &CFx, &vthr_y, &vthr_z, &psiT);
    CL_tp[b_i] = vthr_y;
    CD_tp[b_i] = vthr_z;
    CM_tp[b_i] = psiT;
  }

  for (b_i = 0; b_i < 4; b_i++) {
    McFoamy_Airfoil_Simplified(vr_ycomp[b_i], dv24[b_i], dv23[b_i], -RudDef,
      1.2984123165744528, 0.02, 1.98, 0.0, &CFx, &vthr_y, &vthr_z, &psiT);
    CL_r[b_i] = vthr_y;
    vr_xcomp[b_i] = vthr_z;
    CM_r[b_i] = psiT;
  }

  for (b_i = 0; b_i < 4; b_i++) {
    McFoamy_Airfoil_Simplified(a_B[b_i], 0.0, 0.64171, 0.0, 0.22793785354755264,
      0.02, 1.98, 0.0, &CFx, &vthr_y, &vthr_z, &psiT);
    vB_xcomp[b_i] = vthr_y;
    vB_ycomp[b_i] = vthr_z;
    CM_B[b_i] = psiT;
  }

  for (b_i = 0; b_i < 3; b_i++) {
    D_rods[b_i] = 0.0;
    M_rods[b_i] = 0.0;
  }

  for (b_i = 0; b_i < 32; b_i++) {
    for (i = 0; i < 3; i++) {
      ri[i] = (rods[b_i + (i << 5)] + rods[b_i + ((3 + i) << 5)]) / 2.0;
      li[i] = rods[b_i + ((3 + i) << 5)] - rods[b_i + (i << 5)];
    }

    cross(B_rate, ri, V_l_l);
    b_VProp_Axial[0] = VProp_Axial[b_i + 18];
    b_VProp_Axial[1] = 0.0;
    b_VProp_Axial[2] = 0.0;
    for (i = 0; i < 3; i++) {
      vtp_xcomp[i] = (Vel[i] + V_l_l[i]) + b_VProp_Axial[i];
    }

    vthr_y = norm(vtp_xcomp);
    if (vthr_y >= 1.0E-6) {
      b_vthr_y = vthr_y;
    } else {
      b_vthr_y = 1.0E-6;
    }

    cross(vtp_xcomp, li, dv25);
    psiT = norm(dv25) / b_vthr_y / norm(li);
    cross(vtp_xcomp, li, V_l_l);
    for (i = 0; i < 3; i++) {
      b_V_l_l[i] = V_l_l[i];
    }

    cross(b_V_l_l, li, V_l_l);
    CFx = 0.0;
    for (i = 0; i < 3; i++) {
      CFx += vtp_xcomp[i] * V_l_l[i];
    }

    if (CFx < 0.0) {
      vthr_z = norm(li);
      vthr_y = norm(vtp_xcomp) * (vthr_z * vthr_z);
      if (vthr_y >= 1.0E-6) {
      } else {
        vthr_y = 1.0E-6;
      }

      for (i = 0; i < 3; i++) {
        V_l_l[i] /= vthr_y;
      }
    } else {
      vthr_z = norm(li);
      vthr_y = norm(vtp_xcomp) * (vthr_z * vthr_z);
      if (vthr_y >= 1.0E-6) {
      } else {
        vthr_y = 1.0E-6;
      }

      for (i = 0; i < 3; i++) {
        V_l_l[i] = -V_l_l[i] / vthr_y;
      }
    }

    vthr_z = norm(vtp_xcomp);
    vthr_z = 0.6125 * (vthr_z * vthr_z) * norm(li) * rods[192 + b_i] * 1.1 *
      (psiT * psiT);
    for (i = 0; i < 3; i++) {
      vthr_y = vthr_z * V_l_l[i];
      b_V_l_l[i] = vthr_y;
      D_rods[i] += vthr_y;
    }

    cross(ri, b_V_l_l, V_l_l);
    for (i = 0; i < 3; i++) {
      M_rods[i] += V_l_l[i];
    }
  }

  power(vws_xz, Fx_ws);
  for (i = 0; i < 7; i++) {
    dv14[i] = vws_zcomp[i];
  }

  b_sin(dv14);
  for (i = 0; i < 7; i++) {
    Fz_ws[i] = vws_zcomp[i];
  }

  b_cos(Fz_ws);
  power(vwp_xz, Fx_wp);
  for (i = 0; i < 7; i++) {
    Fx_ws[i] = dv26[i] * Fx_ws[i] * (CL_ws[i] * dv14[i] - vws_xcomp[i] * Fz_ws[i]);
    dv14[i] = vwp_xcomp[i];
  }

  b_sin(dv14);
  for (i = 0; i < 7; i++) {
    Fz_ws[i] = vwp_xcomp[i];
  }

  b_cos(Fz_ws);
  for (i = 0; i < 7; i++) {
    Fx_wp[i] = dv26[i] * Fx_wp[i] * (vwp_zcomp[i] * dv14[i] - CD_wp[i] * Fz_ws[i]);
  }

  b_power(vts_xz, Vel);
  for (i = 0; i < 3; i++) {
    li[i] = vts_zcomp[i];
  }

  c_sin(li);
  for (i = 0; i < 3; i++) {
    V_l_l[i] = vts_zcomp[i];
  }

  c_cos(V_l_l);
  b_power(vtp_xz, B_rate);
  for (i = 0; i < 3; i++) {
    Vel[i] = dv27[i] * Vel[i] * (CL_ts[i] * li[i] - vts_xcomp[i] * V_l_l[i]);
    li[i] = vtp_zcomp[i];
  }

  c_sin(li);
  for (i = 0; i < 3; i++) {
    V_l_l[i] = vtp_zcomp[i];
  }

  c_cos(V_l_l);
  for (i = 0; i < 3; i++) {
    B_rate[i] = dv27[i] * B_rate[i] * (CL_tp[i] * li[i] - CD_tp[i] * V_l_l[i]);
  }

  c_power(vr_xy, Fx_r);
  for (i = 0; i < 4; i++) {
    dv15[i] = vr_ycomp[i];
  }

  d_sin(dv15);
  for (i = 0; i < 4; i++) {
    dv17[i] = vr_ycomp[i];
  }

  d_cos(dv17);
  c_power(vB_xy, Fx_B);
  for (i = 0; i < 4; i++) {
    Fx_r[i] = dv28[i] * Fx_r[i] * (CL_r[i] * dv15[i] - vr_xcomp[i] * dv17[i]);
    dv15[i] = a_B[i];
  }

  d_sin(dv15);
  for (i = 0; i < 4; i++) {
    dv17[i] = a_B[i];
  }

  d_cos(dv17);
  c_power(vr_xy, CtrlDef);
  for (i = 0; i < 4; i++) {
    Fx_B[i] = dv29[i] * Fx_B[i] * (vB_xcomp[i] * dv15[i] - vB_ycomp[i] * dv17[i]);
    dv15[i] = vr_ycomp[i];
  }

  d_cos(dv15);
  d_sin(vr_ycomp);
  for (i = 0; i < 4; i++) {
    CtrlDef[i] = dv28[i] * CtrlDef[i] * (-CL_r[i] * dv15[i] - vr_xcomp[i] *
      vr_ycomp[i]);
    dv15[i] = a_B[i];
  }

  c_power(vB_xy, vr_ycomp);
  d_cos(dv15);
  d_sin(a_B);
  for (i = 0; i < 4; i++) {
    vr_ycomp[i] = dv29[i] * vr_ycomp[i] * (-vB_xcomp[i] * dv15[i] - vB_ycomp[i] *
      a_B[i]);
  }

  power(vws_xz, Fz_ws);
  for (i = 0; i < 7; i++) {
    dv14[i] = vws_zcomp[i];
  }

  b_cos(dv14);
  b_sin(vws_zcomp);
  for (i = 0; i < 7; i++) {
    Fz_ws[i] = dv26[i] * Fz_ws[i] * (-CL_ws[i] * dv14[i] - vws_xcomp[i] *
      vws_zcomp[i]);
    dv14[i] = vwp_xcomp[i];
  }

  power(vwp_xz, vws_zcomp);
  b_cos(dv14);
  b_sin(vwp_xcomp);
  for (i = 0; i < 7; i++) {
    vws_zcomp[i] = dv26[i] * vws_zcomp[i] * (-vwp_zcomp[i] * dv14[i] - CD_wp[i] *
      vwp_xcomp[i]);
  }

  b_power(vts_xz, vtp_xcomp);
  for (i = 0; i < 3; i++) {
    li[i] = vts_zcomp[i];
  }

  c_cos(li);
  c_sin(vts_zcomp);
  for (i = 0; i < 3; i++) {
    vtp_xcomp[i] = dv27[i] * vtp_xcomp[i] * (-CL_ts[i] * li[i] - vts_xcomp[i] *
      vts_zcomp[i]);
    li[i] = vtp_zcomp[i];
  }

  b_power(vtp_xz, vts_zcomp);
  c_cos(li);
  c_sin(vtp_zcomp);
  for (i = 0; i < 3; i++) {
    vts_zcomp[i] = dv27[i] * vts_zcomp[i] * (-CL_tp[i] * li[i] - CD_tp[i] *
      vtp_zcomp[i]);
  }

  power(vws_xz, vws_xcomp);
  power(vwp_xz, CL_ws);
  for (i = 0; i < 7; i++) {
    vws_xcomp[i] = (-0.00661 * Fx_ws[i] - b_b[i] * Fz_ws[i]) + dv26[i] *
      vws_xcomp[i] * Cw[i] * CM_ws[i];
    CL_ws[i] = (-0.00661 * Fx_wp[i] - b_b[i] * vws_zcomp[i]) + dv26[i] * CL_ws[i]
      * Cw[i] * CM_wp[i];
  }

  b_power(vts_xz, vts_xcomp);
  b_power(vtp_xz, CL_ts);
  for (i = 0; i < 3; i++) {
    vts_xcomp[i] = (-0.00661 * Vel[i] - e_b[i] * vtp_xcomp[i]) + dv27[i] *
      vts_xcomp[i] * Ct[i] * CM_ts[i];
    CL_ts[i] = (-0.00661 * B_rate[i] - e_b[i] * vts_zcomp[i]) + dv27[i] *
      CL_ts[i] * Ct[i] * CM_tp[i];
  }

  c_power(vr_xy, vr_xcomp);
  c_power(vB_xy, CL_r);
  for (i = 0; i < 4; i++) {
    vr_xcomp[i] = h_b[i] * CtrlDef[i] - dv28[i] * vr_xcomp[i] * Cr[i] * CM_r[i];
    CL_r[i] = 0.03491 * vr_ycomp[i] - dv29[i] * CL_r[i] * 0.64171 * CM_B[i];
  }

  *Fx = ((((((sum(Fx_ws) + sum(Fx_wp)) + b_sum(Vel)) + b_sum(B_rate)) + c_sum
           (Fx_r)) + c_sum(Fx_B)) + ThrForce_idx_0) + D_rods[0];
  for (i = 0; i < 7; i++) {
    dv14[i] = 0.0;
  }

  for (i = 0; i < 7; i++) {
    CM_ws[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    li[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    V_l_l[i] = 0.0;
  }

  *Fy = ((((((sum(dv14) + sum(CM_ws)) + b_sum(li)) + b_sum(V_l_l)) + c_sum
           (CtrlDef)) + c_sum(vr_ycomp)) + ThrForce_idx_1) + D_rods[1];
  for (i = 0; i < 4; i++) {
    dv15[i] = 0.0;
  }

  for (i = 0; i < 4; i++) {
    dv17[i] = 0.0;
  }

  *Fz = ((((((sum(Fz_ws) + sum(vws_zcomp)) + b_sum(vtp_xcomp)) + b_sum(vts_zcomp))
           + c_sum(dv15)) + c_sum(dv17)) + CFy) + D_rods[2];
  for (i = 0; i < 7; i++) {
    CM_ws[i] = b[i] * Fz_ws[i];
    CD_wp[i] = c_b[i] * vws_zcomp[i];
  }

  for (i = 0; i < 3; i++) {
    ri[i] = d_b[i] * vtp_xcomp[i];
    li[i] = f_b[i] * vts_zcomp[i];
  }

  for (i = 0; i < 4; i++) {
    dv15[i] = 0.0 - g_b[i] * CtrlDef[i];
    dv17[i] = 0.0 - i_b[i] * vr_ycomp[i];
    a_B[i] = g_b[i] * Fx_r[i];
    CM_r[i] = i_b[i] * Fx_B[i];
  }

  *Mx = (((((((sum(CM_ws) + sum(CD_wp)) + b_sum(ri)) + b_sum(li)) + c_sum(dv15))
           + c_sum(dv17)) + (0.0 - -0.00661 * ThrForce_idx_1)) + MX) + M_rods[0];
  *My = ((((((((sum(vws_xcomp) + sum(CL_ws)) + b_sum(vts_xcomp)) + b_sum(CL_ts))
             + c_sum(a_B)) + c_sum(CM_r)) + (-0.00661 * ThrForce_idx_0 - 0.24 *
            CFy)) + 0.5 * (1.225 * (b_y * b_y) * 0.001057227821024 * (-CMy * cos
            (deltaT) + CMz * sin(deltaT)))) + -5.0E-5 * (wOut * 2.0 *
          3.1415926535897931 / 60.0) * r) + M_rods[1];
  for (i = 0; i < 7; i++) {
    dv14[i] = 0.0 - b[i] * Fx_ws[i];
    CM_ws[i] = 0.0 - c_b[i] * Fx_wp[i];
  }

  for (i = 0; i < 3; i++) {
    li[i] = -0.0 - d_b[i] * Vel[i];
    V_l_l[i] = -0.0 - f_b[i] * B_rate[i];
  }

  *Mz = ((((((((sum(dv14) + sum(CM_ws)) + b_sum(li)) + b_sum(V_l_l)) + c_sum
             (vr_xcomp)) + c_sum(CL_r)) + 0.24 * ThrForce_idx_1) + 0.5 * (1.225 *
           (c_y * c_y) * 0.001057227821024 * (-CMy * sin(deltaT) - CMz * cos
            (deltaT)))) + 5.0E-5 * (wOut * 2.0 * 3.1415926535897931 / 60.0) * q)
    + M_rods[2];
}

void McFoamy_FM_v2_initialize(void)
{
}

void McFoamy_FM_v2_terminate(void)
{
}
