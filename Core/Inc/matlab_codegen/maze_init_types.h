/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_init_types.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 08-Jan-2021 11:16:43
 */

#ifndef MAZE_INIT_TYPES_H
#define MAZE_INIT_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
  unsigned char Row;
  unsigned char Col;
  unsigned char section;
} b_struct_T;

#endif                                 /*typedef_b_struct_T*/

#ifndef typedef_c_struct_T
#define typedef_c_struct_T

typedef struct {
  unsigned char North;
  unsigned char North_East;
  unsigned char East;
  unsigned char South_East;
  unsigned char South;
  unsigned char South_West;
  unsigned char West;
  unsigned char North_West;
} c_struct_T;

#endif                                 /*typedef_c_struct_T*/

#ifndef typedef_d_struct_T
#define typedef_d_struct_T

typedef struct {
  unsigned char front;
  unsigned char right;
  unsigned char back;
  unsigned char left;
} d_struct_T;

#endif                                 /*typedef_d_struct_T*/

#ifndef typedef_e_struct_T
#define typedef_e_struct_T

typedef struct {
  unsigned char straight;
  unsigned char diagonal;
} e_struct_T;

#endif                                 /*typedef_e_struct_T*/

#ifndef typedef_i_struct_T
#define typedef_i_struct_T

typedef struct {
  unsigned char b_default;
  unsigned char r_45;
  unsigned char l_45;
  unsigned char r_90;
  unsigned char l_90;
  unsigned char r_135;
  unsigned char l_135;
  unsigned char r_180;
  unsigned char l_180;
} i_struct_T;

#endif                                 /*typedef_i_struct_T*/

#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  unsigned char North;
  unsigned char East;
  unsigned char South;
  unsigned char West;
} struct_T;

#endif                                 /*typedef_struct_T*/
#endif

/*
 * File trailer for maze_init_types.h
 *
 * [EOF]
 */
