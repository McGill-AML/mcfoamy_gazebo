/*
 * _coder_McFoamy_FM_v2_mex.c
 *
 * Code generation for function 'McFoamy_FM_v2'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_McFoamy_FM_v2_api.h"

/* Function Declarations */
static void McFoamy_FM_v2_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "McFoamy_FM_v2", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void McFoamy_FM_v2_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *outputs[6];
  const mxArray *inputs[10];
  int n = 0;
  int nOutputs = (nlhs < 1 ? 1 : nlhs);
  int nInputs = nrhs;
  emlrtStack st = { NULL, NULL, NULL };
  /* Module initialization. */
  McFoamy_FM_v2_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 10) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 10, mxCHAR_CLASS, 13, "McFoamy_FM_v2");
  } else if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 13, "McFoamy_FM_v2");
  }
  /* Temporary copy for mex inputs. */
  for (n = 0; n < nInputs; ++n) {
    inputs[n] = prhs[n];
  }
  /* Call the function. */
  McFoamy_FM_v2_api(inputs, outputs);
  /* Copy over outputs to the caller. */
  for (n = 0; n < nOutputs; ++n) {
    plhs[n] = emlrtReturnArrayR2009a(outputs[n]);
  }
  /* Module finalization. */
  McFoamy_FM_v2_terminate();
}

void McFoamy_FM_v2_atexit_wrapper(void)
{
   McFoamy_FM_v2_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(McFoamy_FM_v2_atexit_wrapper);
  /* Dispatch the entry-point. */
  McFoamy_FM_v2_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_McFoamy_FM_v2_mex.c) */
