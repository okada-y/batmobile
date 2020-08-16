/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_init_types.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Aug-2020 15:40:02
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

#ifndef struct_emxArray_uint16_T
#define struct_emxArray_uint16_T

struct emxArray_uint16_T
{
  unsigned short *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_emxArray_uint16_T*/

#ifndef typedef_emxArray_uint16_T
#define typedef_emxArray_uint16_T

typedef struct emxArray_uint16_T emxArray_uint16_T;

#endif                                 /*typedef_emxArray_uint16_T*/

#ifndef typedef_coder_internal_ref_3
#define typedef_coder_internal_ref_3

typedef struct {
  emxArray_uint16_T *contents;
} coder_internal_ref_3;

#endif                                 /*typedef_coder_internal_ref_3*/

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

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T

struct emxArray_uint8_T
{
  unsigned char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_emxArray_uint8_T*/

#ifndef typedef_emxArray_uint8_T
#define typedef_emxArray_uint8_T

typedef struct emxArray_uint8_T emxArray_uint8_T;

#endif                                 /*typedef_emxArray_uint8_T*/

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

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
} h_struct_T;

#endif                                 /*typedef_h_struct_T*/

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
