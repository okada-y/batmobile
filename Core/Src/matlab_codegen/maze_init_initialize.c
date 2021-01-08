/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_init_initialize.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 08-Jan-2021 11:16:43
 */

/* Include Files */
#include "maze_init.h"
#include "maze_solve.h"
#include "maze_init_initialize.h"
#include "maze_init_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void maze_init_initialize(void)
{
  turn_pattern.b_default = 0U;
  turn_pattern.r_45 = 1U;
  turn_pattern.l_45 = 2U;
  turn_pattern.r_90 = 3U;
  turn_pattern.l_90 = 4U;
  turn_pattern.r_135 = 5U;
  turn_pattern.l_135 = 6U;
  turn_pattern.r_180 = 7U;
  turn_pattern.l_180 = 8U;
  move_dir_property.straight = 0U;
  move_dir_property.diagonal = 1U;
  l_direction.front = 0U;
  l_direction.right = 1U;
  l_direction.back = 2U;
  l_direction.left = 3U;
  g_d_direction.North = 0U;
  g_d_direction.North_East = 1U;
  g_d_direction.East = 2U;
  g_d_direction.South_East = 3U;
  g_d_direction.South = 4U;
  g_d_direction.South_West = 5U;
  g_d_direction.West = 6U;
  g_d_direction.North_West = 7U;
  g_direction.North = 0U;
  g_direction.East = 1U;
  g_direction.South = 2U;
  g_direction.West = 3U;
  matrix_dir.Row = 0U;
  matrix_dir.Col = 1U;
  matrix_dir.section = 2U;
  wall_sensor_left_th = 0;
  wall_sensor_right_th = 0;
  wall_sensor_front_th = 0;
}

/*
 * File trailer for maze_init_initialize.c
 *
 * [EOF]
 */
