/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: idivide.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Aug-2020 15:40:02
 */

/* Include Files */
#include <math.h>
#include "maze_init.h"
#include "maze_solve.h"
#include "idivide.h"
#include "maze_init_emxutil.h"

/* Function Declarations */
static double rt_roundd_snf(double u);

/* Function Definitions */

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : const emxArray_uint16_T *a
 *                double b
 *                emxArray_uint16_T *c
 * Return Type  : void
 */
void idivide(const emxArray_uint16_T *a, double b, emxArray_uint16_T *c)
{
  emxArray_real_T *x;
  int nx;
  int loop_ub;
  double d0;
  unsigned short u1;
  emxInit_real_T(&x, 1);
  nx = x->size[0];
  x->size[0] = a->size[0];
  emxEnsureCapacity_real_T(x, nx);
  loop_ub = a->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    x->data[nx] = (double)a->data[nx] / b;
  }

  nx = x->size[0];
  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    x->data[loop_ub] = trunc(x->data[loop_ub]);
  }

  nx = c->size[0];
  c->size[0] = x->size[0];
  emxEnsureCapacity_uint16_T(c, nx);
  loop_ub = x->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    d0 = rt_roundd_snf(x->data[nx]);
    if (d0 < 65536.0) {
      if (d0 >= 0.0) {
        u1 = (unsigned short)d0;
      } else {
        u1 = 0U;
      }
    } else if (d0 >= 65536.0) {
      u1 = MAX_uint16_T;
    } else {
      u1 = 0U;
    }

    c->data[nx] = u1;
  }

  emxFree_real_T(&x);
}

/*
 * File trailer for idivide.c
 *
 * [EOF]
 */
