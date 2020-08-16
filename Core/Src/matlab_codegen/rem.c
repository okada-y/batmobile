/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: rem.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Aug-2020 15:40:02
 */

/* Include Files */
#include "maze_init.h"
#include "maze_solve.h"
#include "rem.h"
#include "maze_init_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : unsigned char x
 * Return Type  : unsigned char
 */
unsigned char b_rem(unsigned char x)
{
  return (unsigned char)(x % 4);
}

/*
 * Arguments    : const emxArray_uint16_T *x
 *                emxArray_uint16_T *r
 * Return Type  : void
 */
void c_rem(const emxArray_uint16_T *x, emxArray_uint16_T *r)
{
  unsigned int x_idx_0;
  int nx;
  int k;
  x_idx_0 = (unsigned int)x->size[0];
  nx = r->size[0];
  r->size[0] = (int)x_idx_0;
  emxEnsureCapacity_uint16_T(r, nx);
  x_idx_0 = (unsigned int)x->size[0];
  nx = (int)x_idx_0;
  for (k = 0; k < nx; k++) {
    r->data[k] = (unsigned short)(x->data[k] % 32);
  }
}

/*
 * Arguments    : const emxArray_uint16_T *x
 *                emxArray_uint16_T *r
 * Return Type  : void
 */
void d_rem(const emxArray_uint16_T *x, emxArray_uint16_T *r)
{
  unsigned int x_idx_0;
  int nx;
  int k;
  x_idx_0 = (unsigned int)x->size[0];
  nx = r->size[0];
  r->size[0] = (int)x_idx_0;
  emxEnsureCapacity_uint16_T(r, nx);
  x_idx_0 = (unsigned int)x->size[0];
  nx = (int)x_idx_0;
  for (k = 0; k < nx; k++) {
    r->data[k] = (unsigned short)(x->data[k] % 33);
  }
}

/*
 * File trailer for rem.c
 *
 * [EOF]
 */
