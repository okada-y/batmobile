/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: isequal.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Aug-2020 15:40:02
 */

/* Include Files */
#include "maze_init.h"
#include "maze_solve.h"
#include "isequal.h"

/* Function Definitions */

/*
 * Arguments    : const unsigned char varargin_1[2]
 *                const unsigned char varargin_2[2]
 * Return Type  : bool
 */
bool isequal(const unsigned char varargin_1[2], const unsigned char varargin_2[2])
{
  bool p;
  bool b_p;
  int k;
  bool exitg1;
  p = false;
  b_p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (varargin_1[k] != varargin_2[k]) {
      b_p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = true;
  }

  return p;
}

/*
 * File trailer for isequal.c
 *
 * [EOF]
 */
