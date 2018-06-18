/*
 * _coder_maneuver_generator_api.c
 *
 * Code generation for function 'maneuver_generator'
 *
 */

/* Include files */
#include "_coder_maneuver_generator_api.h"

/* Function Declarations */
static double emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *maneuver_type, const char *identifier);
static double b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *p, const
  char *identifier))[3];
static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char *identifier))[4];
static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static const mxArray *emlrt_marshallOut(const double u);
static const mxArray *b_emlrt_marshallOut(const double u[4]);
static const mxArray *c_emlrt_marshallOut(const double u[3]);
static double g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static double (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];

/* Function Definitions */
void maneuver_generator_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void maneuver_generator_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void maneuver_generator_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  maneuver_generator_xil_terminate();
}

void maneuver_generator_api(const mxArray *prhs[4], const mxArray *plhs[5])
{
  double (*qref)[4];
  double (*pref)[3];
  double maneuver_type;
  double absolute_time;
  double (*p)[3];
  double (*q)[4];
  double distance;
  double uref;
  double maneuver_switch;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  qref = (double (*)[4])mxMalloc(sizeof(double [4]));
  pref = (double (*)[3])mxMalloc(sizeof(double [3]));
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[3] = emlrtProtectR2012b(prhs[3], 3, false, -1);

  /* Marshall function inputs */
  maneuver_type = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "maneuver_type");
  absolute_time = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "absolute_time");
  p = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "p");
  q = e_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "q");

  /* Invoke the target function */
  maneuver_generator(maneuver_type, absolute_time, *p, *q, &maneuver_switch,
                     *qref, *pref, &uref, &distance);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(maneuver_switch);
  plhs[1] = b_emlrt_marshallOut(*qref);
  plhs[2] = c_emlrt_marshallOut(*pref);
  plhs[3] = emlrt_marshallOut(uref);
  plhs[4] = emlrt_marshallOut(distance);
}

static double emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *maneuver_type, const char *identifier)
{
  double y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(maneuver_type), &thisId);
  emlrtDestroyArray(&maneuver_type);
  return y;
}

static double b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  double y;
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *p, const
  char *identifier))[3]
{
  double (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(p), &thisId);
  emlrtDestroyArray(&p);
  return y;
}
  static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3]
{
  double (*y)[3];
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char *identifier))[4]
{
  double (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(q), &thisId);
  emlrtDestroyArray(&q);
  return y;
}
  static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[4]
{
  double (*y)[4];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const double u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m0);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const double u[4])
{
  const mxArray *y;
  static const int iv0[2] = { 0, 0 };

  const mxArray *m1;
  static const int iv1[2] = { 1, 4 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, iv1, 2);
  emlrtAssign(&y, m1);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const double u[3])
{
  const mxArray *y;
  static const int iv2[2] = { 0, 0 };

  const mxArray *m2;
  static const int iv3[2] = { 1, 3 };

  y = NULL;
  m2 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u);
  emlrtSetDimensions((mxArray *)m2, iv3, 2);
  emlrtAssign(&y, m2);
  return y;
}

static double g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  double ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(double *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  double (*ret)[3];
  int iv4[2];
  int i0;
  for (i0 = 0; i0 < 2; i0++) {
    iv4[i0] = 1 + (i0 << 1);
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv4);
  ret = (double (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static double (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  double (*ret)[4];
  int iv5[2];
  int i1;
  for (i1 = 0; i1 < 2; i1++) {
    iv5[i1] = 1 + 3 * i1;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv5);
  ret = (double (*)[4])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/* End of code generation (_coder_maneuver_generator_api.c) */
