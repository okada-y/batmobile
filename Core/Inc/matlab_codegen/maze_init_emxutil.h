/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_init_emxutil.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Aug-2020 15:40:02
 */

#ifndef MAZE_INIT_EMXUTIL_H
#define MAZE_INIT_EMXUTIL_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "maze_init_types.h"

/* Function Declarations */
extern void c_emxFreeStruct_coder_internal_(coder_internal_ref_3 *pStruct);
extern void c_emxInitStruct_coder_internal_(coder_internal_ref_3 *pStruct);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_uint16_T(emxArray_uint16_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_uint8_T(emxArray_uint8_T *emxArray, int oldNumel);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxFree_uint16_T(emxArray_uint16_T **pEmxArray);
extern void emxFree_uint8_T(emxArray_uint8_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_uint16_T(emxArray_uint16_T **pEmxArray, int numDimensions);
extern void emxInit_uint8_T(emxArray_uint8_T **pEmxArray, int numDimensions);

#endif

/*
 * File trailer for maze_init_emxutil.h
 *
 * [EOF]
 */
