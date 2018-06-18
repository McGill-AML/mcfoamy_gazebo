#include "_coder_McFoamy_FM_v2_api.h"

static double emlrt_marshallIn(const emlrtStack *sp, const mxArray *LAilDef,
  const char *identifier);
static double b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *emlrt_marshallOut(const double u);
static double c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
void McFoamy_FM_v2_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void McFoamy_FM_v2_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void McFoamy_FM_v2_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  McFoamy_FM_v2_xil_terminate();
}

void McFoamy_FM_v2_api(const mxArray * const prhs[10], const mxArray *plhs[6])
{
  double LAilDef;
  double ElevDef;
  double RudDef;
  double wIn;
  double u;
  double v;
  double w;
  double p;
  double q;
  double r;
  double Mz;
  double My;
  double Mx;
  double Fz;
  double Fy;
  double Fx;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  LAilDef = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "LAilDef");
  ElevDef = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "ElevDef");
  RudDef = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "RudDef");
  wIn = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "wIn");
  u = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "u");
  v = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "v");
  w = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "w");
  p = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "p");
  q = emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "q");
  r = emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "r");
  McFoamy_FM_v2(LAilDef, ElevDef, RudDef, wIn, u, v, w, p, q, r, &Fx, &Fy, &Fz,
                &Mx, &My, &Mz);
  plhs[0] = emlrt_marshallOut(Fx);
  plhs[1] = emlrt_marshallOut(Fy);
  plhs[2] = emlrt_marshallOut(Fz);
  plhs[3] = emlrt_marshallOut(Mx);
  plhs[4] = emlrt_marshallOut(My);
  plhs[5] = emlrt_marshallOut(Mz);
}

static double emlrt_marshallIn(const emlrtStack *sp, const mxArray *LAilDef,
  const char *identifier)
{
  double y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(LAilDef), &thisId);
  emlrtDestroyArray(&LAilDef);
  return y;
}

static double b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  double y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
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

static double c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  double ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(double *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
