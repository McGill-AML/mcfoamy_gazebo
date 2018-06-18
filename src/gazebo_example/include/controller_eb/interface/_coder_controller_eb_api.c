/*
 * File: _coder_controller_eb_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 16-May-2018 15:06:37
 */

/* Include files */
#include "_coder_controller_eb_api.h"

/* Function Declarations */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *C_ba,
  const char *identifier))[9];
static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9];
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *r_a,
  const char *identifier))[3];
static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static double e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char *identifier);
static double f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *emlrt_marshallOut(const double u);
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9];
static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static double i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void controller_eb_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void controller_eb_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void controller_eb_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  controller_eb_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[10]
 *                const mxArray *plhs[4]
 * Return Type  : void
 */
void controller_eb_api(const mxArray *prhs[10], const mxArray *plhs[4])
{
  double (*C_ba)[9];
  double (*r_a)[3];
  double (*v_b)[3];
  double (*omega_ba_b)[3];
  double (*C_ra)[9];
  double (*r_ref_a)[3];
  double (*v_ref_r)[3];
  double (*omega_ra_r)[3];
  double dt;
  double maneuver_switch;
  double u4;
  double u3;
  double u2;
  double u1;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[3] = emlrtProtectR2012b(prhs[3], 3, false, -1);
  prhs[4] = emlrtProtectR2012b(prhs[4], 4, false, -1);
  prhs[5] = emlrtProtectR2012b(prhs[5], 5, false, -1);
  prhs[6] = emlrtProtectR2012b(prhs[6], 6, false, -1);
  prhs[7] = emlrtProtectR2012b(prhs[7], 7, false, -1);

  /* Marshall function inputs */
  C_ba = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "C_ba");
  r_a = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "r_a");
  v_b = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "v_b");
  omega_ba_b = c_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "omega_ba_b");
  C_ra = emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "C_ra");
  r_ref_a = c_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "r_ref_a");
  v_ref_r = c_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "v_ref_r");
  omega_ra_r = c_emlrt_marshallIn(&st, emlrtAlias(prhs[7]), "omega_ra_r");
  dt = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "dt");
  maneuver_switch = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]),
    "maneuver_switch");

  /* Invoke the target function */
  controller_eb(*C_ba, *r_a, *v_b, *omega_ba_b, *C_ra, *r_ref_a, *v_ref_r,
                *omega_ra_r, dt, maneuver_switch, &u1, &u2, &u3, &u4);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(u1);
  plhs[1] = emlrt_marshallOut(u2);
  plhs[2] = emlrt_marshallOut(u3);
  plhs[3] = emlrt_marshallOut(u4);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *C_ba
 *                const char *identifier
 * Return Type  : double (*)[9]
 */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *C_ba,
  const char *identifier))[9]
{
  double (*y)[9];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(C_ba), &thisId);
  emlrtDestroyArray(&C_ba);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[9]
 */
  static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[9]
{
  double (*y)[9];
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *r_a
 *                const char *identifier
 * Return Type  : double (*)[3]
 */
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *r_a,
  const char *identifier))[3]
{
  double (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(r_a), &thisId);
  emlrtDestroyArray(&r_a);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[3]
 */
  static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3]
{
  double (*y)[3];
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *dt
 *                const char *identifier
 * Return Type  : double
 */
static double e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char *identifier)
{
  double y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(dt), &thisId);
  emlrtDestroyArray(&dt);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double
 */
static double f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  double y;
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const double u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const double u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[9]
 */
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9]
{
  double (*ret)[9];
  int iv0[2];
  int i;
  for (i = 0; i < 2; i++) {
    iv0[i] = 3;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv0);
  ret = (double (*)[9])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[3]
 */
  static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  double (*ret)[3];
  int iv1[1];
  iv1[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv1);
  ret = (double (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double
 */
static double i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  double ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(double *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_controller_eb_api.c
 *
 * [EOF]
 */
