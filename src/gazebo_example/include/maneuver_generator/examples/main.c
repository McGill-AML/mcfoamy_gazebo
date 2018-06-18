/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "maneuver_generator.h"
#include "main.h"

/* Function Declarations */
static void argInit_1x3_real_T(double result[3]);
static void argInit_1x4_real_T(double result[4]);
static double argInit_real_T(void);
static void main_maneuver_generator(void);

/* Function Definitions */
static void argInit_1x3_real_T(double result[3])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 3; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

static void argInit_1x4_real_T(double result[4])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 4; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

static double argInit_real_T(void)
{
  return 0.0;
}

static void main_maneuver_generator(void)
{
  double dv1[3];
  double dv2[4];
  double maneuver_switch;
  double qref[4];
  double pref[3];
  double uref;
  double distance;

  /* Initialize function 'maneuver_generator' input arguments. */
  /* Initialize function input argument 'p'. */
  /* Initialize function input argument 'q'. */
  /* Call the entry-point 'maneuver_generator'. */
  argInit_1x3_real_T(dv1);
  argInit_1x4_real_T(dv2);
  maneuver_generator(argInit_real_T(), argInit_real_T(), dv1, dv2,
                     &maneuver_switch, qref, pref, &uref, &distance);
}

int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* Initialize the application.
     You do not need to do this more than one time. */
  maneuver_generator_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_maneuver_generator();

  /* Terminate the application.
     You do not need to do this more than one time. */
  maneuver_generator_terminate();
  return 0;
}

/* End of code generation (main.c) */
