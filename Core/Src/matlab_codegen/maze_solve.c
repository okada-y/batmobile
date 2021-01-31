/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_solve.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 29-Jan-2021 00:40:38
 */

/* Include Files */
#include <string.h>
#include <math.h>
#include "maze_init.h"
#include "maze_solve.h"
#include "rem.h"
#include "maze_init_data.h"
#include "C:\work\matlab\maze_sim_git\src\C_src\matlab_movement.h"
#include "C:\work\matlab\maze_sim_git\src\C_src\matlab_IR_sensor.h"
#include "C:\work\matlab\maze_sim_git\src\C_src\matlab_rom.h"

/* Type Definitions */
#ifndef typedef_coder_internal_ref
#define typedef_coder_internal_ref

typedef struct {
  unsigned char contents;
} coder_internal_ref;

#endif                                 /*typedef_coder_internal_ref*/

#ifndef typedef_coder_internal_ref_1
#define typedef_coder_internal_ref_1

typedef struct {
  unsigned char contents[1024];
} coder_internal_ref_1;

#endif                                 /*typedef_coder_internal_ref_1*/

#ifndef typedef_coder_internal_ref_2
#define typedef_coder_internal_ref_2

typedef struct {
  unsigned char contents[18];
} coder_internal_ref_2;

#endif                                 /*typedef_coder_internal_ref_2*/

#ifndef typedef_coder_internal_ref_3
#define typedef_coder_internal_ref_3

typedef struct {
  unsigned short contents;
} coder_internal_ref_3;

#endif                                 /*typedef_coder_internal_ref_3*/

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

typedef struct {
  unsigned char goal;
  unsigned char search;
} h_struct_T;

#endif                                 /*typedef_h_struct_T*/

#ifndef typedef_coder_internal_ref_4
#define typedef_coder_internal_ref_4

typedef struct {
  h_struct_T contents;
} coder_internal_ref_4;

#endif                                 /*typedef_coder_internal_ref_4*/

#ifndef typedef_g_struct_T
#define typedef_g_struct_T

typedef struct {
  unsigned char unknown;
  unsigned char known;
} g_struct_T;

#endif                                 /*typedef_g_struct_T*/

#ifndef typedef_coder_internal_ref_5
#define typedef_coder_internal_ref_5

typedef struct {
  g_struct_T contents;
} coder_internal_ref_5;

#endif                                 /*typedef_coder_internal_ref_5*/

#ifndef typedef_f_struct_T
#define typedef_f_struct_T

typedef struct {
  unsigned char nowall;
  unsigned char wall;
} f_struct_T;

#endif                                 /*typedef_f_struct_T*/

#ifndef typedef_coder_internal_ref_6
#define typedef_coder_internal_ref_6

typedef struct {
  f_struct_T contents;
} coder_internal_ref_6;

#endif                                 /*typedef_coder_internal_ref_6*/

/* Function Declarations */
static void b_fust_run(const coder_internal_ref_1 *maze_wall, const
  coder_internal_ref *goal_size, coder_internal_ref *wall_flg, const
  coder_internal_ref_6 *wall, const unsigned char b_maze_wall[1024], const
  unsigned short contour_map[1024], const unsigned char maze_goal[18], unsigned
  short max_length, unsigned char start_x, unsigned char start_y, unsigned char
  start_flg);
static void b_make_map_find(const coder_internal_ref_6 *wall, const unsigned
  char maze_goal[2], const unsigned char maze_wall[1024], unsigned char
  current_x, unsigned char current_y, unsigned short contour_map[1024]);
static void b_make_map_fustrun_diagonal(coder_internal_ref_3 *max_length, const
  coder_internal_ref_6 *wall, const coder_internal_ref_5 *search, const unsigned
  char maze_goal[2], const unsigned char maze_wall[1024], const unsigned char
  maze_wall_search[1024], unsigned short row_num_node[1056], unsigned short
  col_num_node[1056], unsigned short *start_num);
static void b_search_adachi(const coder_internal_ref_6 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_5 *search, const coder_internal_ref_2
  *maze_goal, const coder_internal_ref_1 *maze_wall, const coder_internal_ref_4 *
  adachi_search_mode, unsigned char *current_x, unsigned char *current_y,
  unsigned char *current_dir, unsigned char maze_row_size, unsigned char
  maze_col_size, unsigned char b_maze_wall[1024], unsigned char
  maze_wall_search[1024], const unsigned char exploration_goal[2], unsigned char
  *start_flg, unsigned char stop_flg, unsigned char goal_after_flg, unsigned
  char adachi_s_mode, unsigned short contour_map[1024]);
static void decide_goal_node_dir(const unsigned char maze_goal[18], unsigned
  char goal_size, const unsigned short row_num_node[1056], const unsigned short
  col_num_node[1056], unsigned char goal_node[2], unsigned char *goal_matrix_dir,
  unsigned char *goal_dir);
static void decide_goal_section(const unsigned char maze_goal[18], const
  unsigned char goal_node[2], unsigned char goal_matrix_dir, unsigned char
  goal_dir, unsigned char goal_section[2], unsigned char goal_node2[2], unsigned
  char *goal_matrix_dir2);
static void fust_run(const coder_internal_ref_1 *maze_wall, const
                     coder_internal_ref *goal_size, coder_internal_ref *wall_flg,
                     const coder_internal_ref_6 *wall, const unsigned char
                     b_maze_wall[1024], const unsigned char maze_wall_search
                     [1024], const unsigned short contour_map[1024], const
                     unsigned char maze_goal[18], unsigned short max_length,
                     unsigned char unexp_square[1024], unsigned char
                     *unexp_square_idx);
static unsigned char fust_run_wallset(const coder_internal_ref_1 *maze_wall,
  unsigned char temp_y, unsigned char temp_x, unsigned char temp_dir);
static void get_next_dir_diagonal(const unsigned short row_num_node[1056], const
  unsigned short col_num_node[1056], unsigned char current_move_dir, const
  unsigned char current_node[2], unsigned char current_matrix_dir, const
  unsigned char goal_node2[2], unsigned char goal_matrix_dir2, const unsigned
  char goal_section[2], unsigned char *next_dir, unsigned char next_node[2],
  unsigned char *next_node_property);
static void get_nextdir2(unsigned char current_x, unsigned char current_y, const
  unsigned char maze_wall[1024], const unsigned short contour_map[1024],
  unsigned char *next_dir, unsigned char *error_flg);
static unsigned char get_turn_pattern_num(const double move_dir_buffer[3],
  unsigned char ref_move_mode);
static double goal_judge(const unsigned char maze_goal[18], unsigned char x,
  unsigned char y);
static void make_map_find(const coder_internal_ref_6 *wall, const unsigned char
  maze_goal[18], unsigned char l_goal_size, const unsigned char maze_wall[1024],
  unsigned char current_x, unsigned char current_y, unsigned short contour_map
  [1024]);
static void make_map_fustrun(const coder_internal_ref *goal_size, const
  coder_internal_ref_6 *wall, const coder_internal_ref_5 *search, const unsigned
  char maze_goal[18], const unsigned char maze_wall[1024], const unsigned char
  maze_wall_search[1024], unsigned char unknown_wall_flg, unsigned short
  contour_map[1024]);
static void make_map_fustrun_diagonal(coder_internal_ref_3 *max_length, const
  coder_internal_ref_6 *wall, const coder_internal_ref_5 *search, const unsigned
  char maze_goal[18], unsigned char goal_size, const unsigned char maze_wall
  [1024], const unsigned char maze_wall_search[1024], unsigned short
  row_num_node[1056], unsigned short col_num_node[1056], unsigned short
  *start_num);
static void make_new_goal_all(const coder_internal_ref_6 *wall, const unsigned
  char maze_wall[1024], const unsigned char maze_wall_search[1024], unsigned
  char current_x, unsigned char current_y, unsigned short contour_map[1024],
  unsigned char new_goal[2]);
static void make_new_goal_sh(const coder_internal_ref_6 *wall, const unsigned
  char maze_wall[1024], unsigned char current_x, unsigned char current_y, const
  unsigned char unexp_square[1024], unsigned char unexp_square_idx, unsigned
  short contour_map[1024], unsigned char new_goal[2]);
static void make_route_diagonal(const coder_internal_ref *start_flg, const
  coder_internal_ref *wall_flg, const unsigned short row_num_node[1056], const
  unsigned short col_num_node[1056], const unsigned char goal_section[2], const
  unsigned char goal_node2[2], unsigned char goal_node_property);
static void move_step(unsigned char *temp_x, unsigned char *temp_y, unsigned
                      char temp_dir);
static void move_straight(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode, unsigned char straight_count);
static void search_adachi(const coder_internal_ref_6 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_5 *search, const coder_internal_ref_2
  *maze_goal, const coder_internal_ref_1 *maze_wall, const coder_internal_ref_4 *
  adachi_search_mode, unsigned char *current_x, unsigned char *current_y,
  unsigned char *current_dir, unsigned char maze_row_size, unsigned char
  maze_col_size, unsigned char b_maze_wall[1024], unsigned char
  maze_wall_search[1024], const unsigned char exploration_goal[18], unsigned
  char l_goal_size, unsigned char *start_flg, unsigned char adachi_s_mode,
  unsigned short contour_map[1024]);
static unsigned char sh_route_unexp_sq_jud(const unsigned char
  temp_unexp_square[1024], unsigned char temp_unexp_square_idx, unsigned char
  temp_y, unsigned char temp_x);
static void turn_180deg(unsigned char *current_dir);
static void turn_clk_90deg(unsigned char *current_dir);
static void turn_conclk_90deg(unsigned char *current_dir);
static void turn_l_135(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode);
static void turn_l_180(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode);
static void turn_l_45(unsigned char current_node[2], unsigned char
                      *current_node_property, unsigned char *current_move_dir,
                      unsigned char *current_move_mode);
static void turn_l_90(unsigned char current_node[2], unsigned char
                      *current_node_property, unsigned char *current_move_dir,
                      unsigned char *current_move_mode);
static void turn_r_135(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode);
static void turn_r_45(unsigned char current_node[2], unsigned char
                      *current_node_property, unsigned char *current_move_dir,
                      unsigned char *current_move_mode);
static void wall_set(const coder_internal_ref_6 *wall, coder_internal_ref
                     *wall_flg, const coder_internal_ref_5 *search, const
                     coder_internal_ref_2 *maze_goal, unsigned char
                     maze_row_size, unsigned char maze_col_size, unsigned char
                     current_x, unsigned char current_y, unsigned char
                     current_dir, unsigned char maze_wall[1024], unsigned char
                     maze_wall_search[1024]);

/* Function Definitions */

/*
 * 入力　壁情報,壁探索情報,等高線MAP,ゴール座標,最大経路長
 * 出力   最短経路上の未探索マスの座標、未探索マスの数
 * Arguments    : const coder_internal_ref_1 *maze_wall
 *                const coder_internal_ref *goal_size
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_6 *wall
 *                const unsigned char b_maze_wall[1024]
 *                const unsigned short contour_map[1024]
 *                const unsigned char maze_goal[18]
 *                unsigned short max_length
 *                unsigned char start_x
 *                unsigned char start_y
 *                unsigned char start_flg
 * Return Type  : void
 */
static void b_fust_run(const coder_internal_ref_1 *maze_wall, const
  coder_internal_ref *goal_size, coder_internal_ref *wall_flg, const
  coder_internal_ref_6 *wall, const unsigned char b_maze_wall[1024], const
  unsigned short contour_map[1024], const unsigned char maze_goal[18], unsigned
  short max_length, unsigned char start_x, unsigned char start_y, unsigned char
  start_flg)
{
  unsigned char goal_flag;
  unsigned short little;
  unsigned char straight_count;
  unsigned char temp_x;
  unsigned char temp_y;
  unsigned char temp_dir;
  unsigned char next_dir;
  int tempk;
  bool exitg1;
  int i71;
  int tempi;
  int i72;
  int i73;
  unsigned short u5;
  int i74;
  int i75;
  unsigned int qY;
  unsigned char switch_expression;

  /*     %% fust_run 最短経路走行 */
  /* 最短経路表示用ax */
  /*          global sh_route_ax */
  /* local変数宣言 */
  goal_flag = 0U;

  /* ゴール判定フラグ */
  little = max_length;

  /* 進行方向選定用閾値 */
  straight_count = 0U;

  /*          %マウス位置表示用オブジェクト */
  /*          if coder.target('MATLAB') */
  /*              ax = gca; */
  /*              h = hgtransform('Parent',ax); */
  /*          end */
  /* マウスの初期位置設定 */
  temp_x = start_x;
  temp_y = start_y;

  /* マウスの初期方向定義 */
  temp_dir = g_direction.North;
  next_dir = g_direction.North;

  /* 探索開始時x */
  /* 探索開始時y */
  /* 実行時、既存の最短ルート表示を削除する(MATLABのみ) */
  tempk = 0;
  exitg1 = false;
  while ((!exitg1) && (tempk <= max_length - 1)) {
    /* 非走行モードのとき、非探索マスを記録する */
    /* 現在位置がゴールか判定 */
    i71 = goal_size->contents;
    for (tempi = 0; tempi < i71; tempi++) {
      if ((temp_x == maze_goal[tempi]) && (temp_y == maze_goal[tempi + 9])) {
        goal_flag = 1U;
      }
    }

    if (goal_flag == 1) {
      /* ゴールかつ走行時、停止処理を実施 */
      if (straight_count > 0) {
        /* 直進カウンタが立っている場合 */
        /* 走行モード時、Cの動作関数を呼び出し */
        m_move_front_long(straight_count, start_flg, wall_flg->contents,
                          move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        start_flg = 0U;
        wall_flg->contents = 0U;

        /* 移動後、ストレートカウンタをクリア */
      }

      /* 参照マス、参照方向を更新 */
      /* 走行モード時、Cの動作関数を呼び出し */
      m_goal_movement(start_flg, wall_flg->contents, move_dir_property.straight);
      exitg1 = true;
    } else {
      /*         %%進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      /* 北側の壁のありなし */
      i71 = temp_y + ((temp_x - 1) << 5);
      tempi = b_maze_wall[i71 - 1];
      if (g_direction.North <= 7) {
        i72 = (unsigned char)(1 << g_direction.North);
      } else {
        i72 = 0;
      }

      if (((tempi & i72) == wall->contents.nowall) && (contour_map[i71] < little))
      {
        /* 北側の等高線mapが閾値より低ければ、 */
        /* 閾値を北側の等高map値に変更 */
        little = contour_map[temp_y + ((temp_x - 1) << 5)];

        /* 北側を進行方向に変更y */
        next_dir = g_direction.North;
      }

      /* 東側 */
      if (g_direction.East <= 7) {
        i73 = (unsigned char)(1 << g_direction.East);
      } else {
        i73 = 0;
      }

      if ((tempi & i73) == wall->contents.nowall) {
        u5 = contour_map[(temp_y + (temp_x << 5)) - 1];
        if (u5 < little) {
          little = u5;
          next_dir = g_direction.East;
        }
      }

      /* 南側 */
      if (g_direction.South <= 7) {
        i74 = (unsigned char)(1 << g_direction.South);
      } else {
        i74 = 0;
      }

      if ((tempi & i74) == wall->contents.nowall) {
        u5 = contour_map[i71 - 2];
        if (u5 < little) {
          little = u5;
          next_dir = g_direction.South;
        }
      }

      /* 西側 */
      if (g_direction.West <= 7) {
        i75 = (unsigned char)(1 << g_direction.West);
      } else {
        i75 = 0;
      }

      if ((tempi & i75) == wall->contents.nowall) {
        u5 = contour_map[(temp_y + ((temp_x - 2) << 5)) - 1];
        if (u5 < little) {
          little = u5;
          next_dir = g_direction.West;
        }
      }

      /*          %走行時、探索壁情報に応じて、壁フラグをセット */
      /*          if (Sh_r_mode) */
      /*              %前 */
      /*              if bitand(maze_wall(temp_y,temp_x),rem(bitshift(uint8(1),temp_dir),15)) */
      /*                  wall_flg = bitor(wall_flg,1,'uint8'); */
      /*              end */
      /*              %右 */
      /*              if bitand(maze_wall(temp_y,temp_x),rem(bitshift(uint8(1),temp_dir+1),15)) */
      /*                  wall_flg = bitor(wall_flg,2,'uint8'); */
      /*              end */
      /*              %左 */
      /*              if bitand(maze_wall(temp_y,temp_x),rem(bitshift(uint8(1),temp_dir+3),15)) */
      /*                  wall_flg = bitor(wall_flg,8,'uint8'); */
      /*              end */
      /*          end */
      /*         %%現在方向と進行方向に応じた処理 */
      tempi = (int)(4U + next_dir);
      if ((unsigned int)tempi > 255U) {
        tempi = 255;
      }

      qY = (unsigned int)tempi - temp_dir;
      if (qY > (unsigned int)tempi) {
        qY = 0U;
      }

      switch_expression = (unsigned char)((int)qY % 4);
      if (l_direction.front == switch_expression) {
        tempi = 0;
      } else if (l_direction.right == switch_expression) {
        tempi = 1;
      } else if (l_direction.back == switch_expression) {
        tempi = 2;
      } else if (l_direction.left == switch_expression) {
        tempi = 3;
      } else {
        tempi = -1;
      }

      switch (tempi) {
       case 0:
        /* 直進の場合、直進カウンタをインクリメント */
        i71 = (int)(straight_count + 1U);
        if ((unsigned int)i71 > 255U) {
          i71 = 255;
        }

        straight_count = (unsigned char)i71;
        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }

        /* 参照マスを移動 */
        /*                  %disp("front") */
        /*                  if (Sh_r_mode) %走行モード時、Cの動作関数を呼び出し */
        /*                      if ~coder.target('MATLAB') */
        /*                          coder.ceval('m_move_front',start_flg,wall_flg,uint8(move_dir_property.straight)); */
        /*                      end */
        /*                      %スタート直後フラグと壁フラグをクリア */
        /*                      start_flg = uint8(0); */
        /*                      wall_flg = uint8(0); */
        /*                  end */
        break;

       case 1:
        if (straight_count > 0) {
          /* 直進カウンタが立っている場合 */
          /* 走行モード時、Cの動作関数を呼び出し */
          m_move_front_long(straight_count, start_flg, wall_flg->contents,
                            move_dir_property.straight);

          /* スタート直後フラグと壁フラグをクリア */
          start_flg = 0U;
          wall_flg->contents = 0U;

          /* 移動後、ストレートカウンタをクリア */
          straight_count = 0U;
        }

        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 参照マス、参照方向を更新 */
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_clk_90deg 時計周りに90度ターンする関数 */
        i71 = (int)(4U + temp_dir);
        if ((unsigned int)i71 > 255U) {
          i71 = 255;
        }

        i71++;
        if ((unsigned int)i71 > 255U) {
          i71 = 255;
        }

        temp_dir = (unsigned char)(i71 % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }

        /* 走行モード時、Cの動作関数を呼び出し */
        m_move_right(start_flg, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        start_flg = 0U;
        wall_flg->contents = 0U;
        break;

       case 2:
        if (straight_count > 0) {
          /* 直進カウンタが立っている場合 */
          /* 走行モード時、Cの動作関数を呼び出し */
          m_move_front_long(straight_count, start_flg, wall_flg->contents,
                            move_dir_property.straight);

          /* スタート直後フラグと壁フラグをクリア */
          start_flg = 0U;
          wall_flg->contents = 0U;

          /* 移動後、ストレートカウンタをクリア */
          straight_count = 0U;
        }

        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 参照マス、参照方向を更新 */
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_180deg 180度ターンする関数 */
        i71 = (int)(4U + temp_dir);
        if ((unsigned int)i71 > 255U) {
          i71 = 255;
        }

        temp_dir = (unsigned char)((i71 - 2) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }

        /* 走行モード時、Cの動作関数を呼び出し */
        m_move_back(start_flg, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        start_flg = 0U;
        wall_flg->contents = 0U;
        break;

       case 3:
        if (straight_count > 0) {
          /* 直進カウンタが立っている場合 */
          /* 走行モード時、Cの動作関数を呼び出し */
          m_move_front_long(straight_count, start_flg, wall_flg->contents,
                            move_dir_property.straight);

          /* スタート直後フラグと壁フラグをクリア */
          start_flg = 0U;
          wall_flg->contents = 0U;

          /* 移動後、ストレートカウンタをクリア */
          straight_count = 0U;
        }

        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 参照マス、参照方向を更新 */
        /* 入力　現在方向 */
        /* 出力　現在方向 */
        /*     %% turn_conclk_90deg 反時計周りに90度回る関数 */
        i71 = (int)(4U + temp_dir);
        if ((unsigned int)i71 > 255U) {
          i71 = 255;
        }

        temp_dir = (unsigned char)((i71 - 1) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }

        /* disp("left") */
        /* 走行モード時、Cの動作関数を呼び出し */
        m_move_left(start_flg, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        start_flg = 0U;
        wall_flg->contents = 0U;
        break;
      }

      /* for code generation */
      tempk++;
    }
  }
}

/*
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : const coder_internal_ref_6 *wall
 *                const unsigned char maze_goal[2]
 *                const unsigned char maze_wall[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void b_make_map_find(const coder_internal_ref_6 *wall, const unsigned
  char maze_goal[2], const unsigned char maze_wall[1024], unsigned char
  current_x, unsigned char current_y, unsigned short contour_map[1024])
{
  unsigned char contor_renew_square[2048];
  unsigned char contor_renew_square_temp[2048];
  unsigned char contor_renew_square_idx;
  unsigned char contor_renew_square_idx_temp;
  int i21;
  unsigned short tempi;
  bool exitg1;
  unsigned char change_flag;
  int tempn;
  int i22;
  int i23;
  int i24;
  int i25;
  int i26;
  int i27;
  int i28;
  unsigned int qY;
  int i29;

  /*     %%  make_map_find 壁情報から等高線MAPを生成 */
  /*  迷路パラメータ設定 */
  /* コンター更新マス保管用 */
  /* 更新座標 */
  memset(&contor_renew_square[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_square_temp[0], 0, sizeof(unsigned char) << 11);

  /* 更新座標更新用 */
  contor_renew_square_idx = 1U;

  /* 更新座標 */
  contor_renew_square_idx_temp = 1U;

  /* 更新座標更新用 */
  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* 32マス分mapを保持 */
  /* 16bitにすべき */
  for (i21 = 0; i21 < 1024; i21++) {
    contour_map[i21] = MAX_uint16_T;
  }

  /* ゴール座標に0を入力 */
  contour_map[(maze_goal[1] + ((maze_goal[0] - 1) << 5)) - 1] = 0U;
  contor_renew_square[0] = maze_goal[1];
  contor_renew_square[1024] = maze_goal[0];
  tempi = 0U;
  exitg1 = false;
  while ((!exitg1) && (tempi < 65535)) {
    /* 歩数カウントは0~max_length */
    /* map更新確認用フラグ */
    change_flag = 0U;

    /* 更新された座標に対し、歩数mapを更新 */
    i21 = contor_renew_square_idx;
    for (tempn = 0; tempn < i21; tempn++) {
      /* 北側 */
      /* if (bitand(maze_wall(row(tempn),col(tempn)),bitshift(uint8(1),g_direction.North)) == wall.nowall) */
      if (g_direction.North <= 7) {
        i22 = (unsigned char)(1 << g_direction.North);
      } else {
        i22 = 0;
      }

      if ((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn +
              1024] - 1) << 5)) - 1] & i22) == wall->contents.nowall) {
        /* 北側のMAPが更新されているか判断、されていなければ書き込み */
        i23 = (int)(contor_renew_square[tempn] + 1U);
        i24 = i23;
        if ((unsigned int)i23 > 255U) {
          i24 = 255;
        }

        if (contour_map[(i24 + ((contor_renew_square[tempn + 1024] - 1) << 5)) -
            1] == 65535) {
          i24 = i23;
          if ((unsigned int)i23 > 255U) {
            i24 = 255;
          }

          contour_map[(i24 + ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
            = (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          if ((unsigned int)i23 > 255U) {
            i23 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)i23;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square[tempn + 1024];

          /* 更新マス用インデックスを増加 */
          i23 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i23 > 255U) {
            i23 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i23;
        }
      }

      /* 東側 */
      contor_renew_square_idx = contor_renew_square[tempn + 1024];
      i23 = (contor_renew_square_idx - 1) << 5;
      i24 = maze_wall[(contor_renew_square[tempn] + i23) - 1];
      if (g_direction.East <= 7) {
        i25 = (unsigned char)(1 << g_direction.East);
      } else {
        i25 = 0;
      }

      if ((i24 & i25) == wall->contents.nowall) {
        /* 東側のMAPが更新されているか判断、されていなければ書き込み */
        i26 = (int)(contor_renew_square[tempn + 1024] + 1U);
        i27 = i26;
        if ((unsigned int)i26 > 255U) {
          i27 = 255;
        }

        if (contour_map[(contor_renew_square[tempn] + ((i27 - 1) << 5)) - 1] ==
            65535) {
          i27 = i26;
          if ((unsigned int)i26 > 255U) {
            i27 = 255;
          }

          contour_map[(contor_renew_square[tempn] + ((i27 - 1) << 5)) - 1] =
            (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
            contor_renew_square[tempn];
          if ((unsigned int)i26 > 255U) {
            i26 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            (unsigned char)i26;

          /* 更新マス用インデックスを増加 */
          i26 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i26 > 255U) {
            i26 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i26;
        }
      }

      /* 南側 */
      if (g_direction.South <= 7) {
        i28 = (unsigned char)(1 << g_direction.South);
      } else {
        i28 = 0;
      }

      if ((i24 & i28) == wall->contents.nowall) {
        /* 南側のMAPが更新されているか判断、されていなければ書き込み */
        qY = contor_renew_square[tempn] - 1U;
        if (qY > contor_renew_square[tempn]) {
          qY = 0U;
        }

        if (contour_map[((int)qY + i23) - 1] == 65535) {
          qY = contor_renew_square[tempn] - 1U;
          if (qY > contor_renew_square[tempn]) {
            qY = 0U;
          }

          contour_map[((int)qY + i23) - 1] = (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          qY = contor_renew_square[tempn] - 1U;
          if (qY > contor_renew_square[tempn]) {
            qY = 0U;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)qY;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square_idx;

          /* 更新マス用インデックスを増加 */
          i23 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i23 > 255U) {
            i23 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i23;
        }
      }

      /* 西側 */
      if (g_direction.West <= 7) {
        i29 = (unsigned char)(1 << g_direction.West);
      } else {
        i29 = 0;
      }

      if ((i24 & i29) == wall->contents.nowall) {
        /* 西側のMAPが更新されているか判断、されていなければ書き込み */
        qY = contor_renew_square_idx - 1U;
        if (qY > contor_renew_square_idx) {
          qY = 0U;
        }

        if (contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) - 1]
            == 65535) {
          qY = contor_renew_square_idx - 1U;
          if (qY > contor_renew_square_idx) {
            qY = 0U;
          }

          contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) - 1] =
            (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
            contor_renew_square[tempn];
          qY = contor_renew_square_idx - 1U;
          if (qY > contor_renew_square_idx) {
            qY = 0U;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            (unsigned char)qY;

          /* 更新マス用インデックスを増加 */
          i23 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i23 > 255U) {
            i23 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i23;
        }
      }
    }

    /* ゴール更新マスの更新とインデックスのクリア */
    for (i21 = 0; i21 < 2048; i21++) {
      contor_renew_square[i21] = contor_renew_square_temp[i21];
      contor_renew_square_temp[i21] = 0U;
    }

    contor_renew_square_idx = (unsigned char)(contor_renew_square_idx_temp - 1);
    contor_renew_square_idx_temp = 1U;

    /* 更新がない、もしくは現在位置が更新されていれば終了 */
    if ((change_flag == 0) || (contour_map[(current_y + ((current_x - 1) << 5))
         - 1] != 65535)) {
      exitg1 = true;
    } else {
      tempi++;
    }
  }
}

/*
 * 未知壁の領域は仮想壁をおいて侵入しない。
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数),迷路探索情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : coder_internal_ref_3 *max_length
 *                const coder_internal_ref_6 *wall
 *                const coder_internal_ref_5 *search
 *                const unsigned char maze_goal[2]
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned short row_num_node[1056]
 *                unsigned short col_num_node[1056]
 *                unsigned short *start_num
 * Return Type  : void
 */
static void b_make_map_fustrun_diagonal(coder_internal_ref_3 *max_length, const
  coder_internal_ref_6 *wall, const coder_internal_ref_5 *search, const unsigned
  char maze_goal[2], const unsigned char maze_wall[1024], const unsigned char
  maze_wall_search[1024], unsigned short row_num_node[1056], unsigned short
  col_num_node[1056], unsigned short *start_num)
{
  unsigned char contor_renew_node_row_idx;
  unsigned char contor_renew_node_row_idx_temp;
  unsigned char contor_renew_node_row[2048];
  unsigned char contor_renew_node_row_temp[2048];
  unsigned char contor_renew_node_col[2048];
  unsigned char contor_renew_node_col_temp[2048];
  unsigned char contor_renew_node_col_idx;
  unsigned char contor_renew_node_col_idx_temp;
  int i154;
  unsigned char row_dir_node[1056];
  unsigned char col_dir_node[1056];
  int i155;
  int i156;
  int i157;
  int i158;
  int i159;
  int row_num_node_tmp;
  int i160;
  int i161;
  int i162;
  unsigned int qY;
  int i163;
  unsigned short i;
  bool exitg1;
  unsigned char change_flag;
  int i164;
  int i165;
  int i166;
  int i167;
  unsigned int b_qY;
  int i168;
  int i169;
  int i170;
  int i171;
  int i172;
  unsigned int c_qY;
  int i173;
  int i174;
  int i175;
  int i176;
  unsigned int u9;
  int i177;
  int i178;
  int i179;
  int i180;
  int i181;
  int i182;
  int i183;
  int i184;
  int i185;
  int i186;
  int i187;
  int i188;
  int i189;
  int i190;
  int i191;
  int i192;
  int i193;
  int i194;
  int i195;
  int i196;
  int i197;
  int i198;
  int i199;
  int i200;
  int i201;
  int i202;
  int i203;
  int i204;
  int i205;
  int i206;
  int i207;
  int i208;
  int i209;
  int i210;
  int i211;
  int i212;
  int i213;
  int i214;
  int i215;
  int i216;
  int i217;
  int i218;
  int i219;
  int i220;
  int i221;
  int i222;
  int i223;

  /*     %% make_map_fustrun_diagonal 最短走行用等高線MAPを生成 */
  /* ローカル変数設定 */
  /* コンター更新ノード(行)保管用 */
  /* 更新座標 */
  /* 更新座標更新用 */
  contor_renew_node_row_idx = 1U;

  /* 更新座標 */
  contor_renew_node_row_idx_temp = 1U;

  /* 更新座標更新用 */
  /* コンター更新ノード（列）保管用 */
  /* 更新座標 */
  memset(&contor_renew_node_row[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_node_row_temp[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_node_col[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_node_col_temp[0], 0, sizeof(unsigned char) << 11);

  /* 更新座標更新用 */
  contor_renew_node_col_idx = 1U;

  /* 更新座標 */
  contor_renew_node_col_idx_temp = 1U;

  /* 更新座標更新用 */
  /* パラメータ設定 */
  /*  迷路パラメータ設定 */
  max_length->contents = 1024U;

  /*  ルートの重み設定 */
  /* MAPの初期化(すべてのノードにmax_lengthを入力) */
  /* 歩数MAP */
  /*  %更新用MAP */
  /*  row_num_node_temp = ones(33,32,'uint16')*uint16(65535); */
  /*  col_num_node_temp = ones(32,33,'uint16')*uint16(65535); */
  /* 進行方向保持用ノード作成 */
  for (i154 = 0; i154 < 1056; i154++) {
    row_num_node[i154] = MAX_uint16_T;
    col_num_node[i154] = MAX_uint16_T;
    row_dir_node[i154] = 0U;
    col_dir_node[i154] = 0U;
  }

  /* ゴールセクションが確定している場合 */
  *start_num = MAX_uint16_T;

  /* ゴールマスから、東西南北にマップを展開 */
  /* 北壁 */
  i154 = maze_goal[0] - 1;
  i155 = (maze_goal[1] + (i154 << 5)) - 1;
  if (g_direction.North <= 7) {
    i156 = (unsigned char)(1 << g_direction.North);
  } else {
    i156 = 0;
  }

  if ((maze_wall[i155] & i156) == 0) {
    /* 歩数更新 */
    i157 = (int)(maze_goal[1] + 1U);
    i158 = i157;
    if ((unsigned int)i157 > 255U) {
      i158 = 255;
    }

    row_num_node_tmp = 33 * i154;
    row_num_node[(i158 + row_num_node_tmp) - 1] = 3U;

    /* 方向追加 */
    i154 = i157;
    if ((unsigned int)i157 > 255U) {
      i154 = 255;
    }

    if (g_d_direction.North <= 7) {
      row_dir_node[(i154 + row_num_node_tmp) - 1] = (unsigned char)(1 <<
        g_d_direction.North);
    } else {
      row_dir_node[(i154 + row_num_node_tmp) - 1] = 0U;
    }

    /* 更新ノードを更新 */
    if ((unsigned int)i157 > 255U) {
      i157 = 255;
    }

    contor_renew_node_row[0] = (unsigned char)i157;
    contor_renew_node_row[1024] = maze_goal[0];

    /* 更新マス用インデックスを増加 */
    contor_renew_node_row_idx = 2U;
  }

  /* 東壁 */
  if (g_direction.East <= 7) {
    i159 = (unsigned char)(1 << g_direction.East);
  } else {
    i159 = 0;
  }

  if ((maze_wall[i155] & i159) == 0) {
    /* 歩数更新 */
    i154 = (int)(maze_goal[0] + 1U);
    i157 = i154;
    if ((unsigned int)i154 > 255U) {
      i157 = 255;
    }

    col_num_node[(maze_goal[1] + ((i157 - 1) << 5)) - 1] = 3U;

    /* 方向追加 */
    i157 = i154;
    if ((unsigned int)i154 > 255U) {
      i157 = 255;
    }

    if (g_d_direction.East <= 7) {
      col_dir_node[(maze_goal[1] + ((i157 - 1) << 5)) - 1] = (unsigned char)(1 <<
        g_d_direction.East);
    } else {
      col_dir_node[(maze_goal[1] + ((i157 - 1) << 5)) - 1] = 0U;
    }

    /* 更新ノードを更新 */
    contor_renew_node_col[0] = maze_goal[1];
    if ((unsigned int)i154 > 255U) {
      i154 = 255;
    }

    contor_renew_node_col[1024] = (unsigned char)i154;

    /* 更新マス用インデックスを増加 */
    contor_renew_node_col_idx = 2U;
  }

  /* 南壁 */
  if (g_direction.South <= 7) {
    i160 = (unsigned char)(1 << g_direction.South);
  } else {
    i160 = 0;
  }

  if ((maze_wall[i155] & i160) == 0) {
    /* 歩数更新 */
    row_num_node_tmp = (maze_goal[1] + 33 * (maze_goal[0] - 1)) - 1;
    row_num_node[row_num_node_tmp] = 3U;

    /* 方向追加 */
    if (g_d_direction.South <= 7) {
      i162 = (unsigned char)(1 << g_d_direction.South);
    } else {
      i162 = 0;
    }

    row_dir_node[row_num_node_tmp] = (unsigned char)
      (row_dir_node[row_num_node_tmp] | i162);

    /* 更新ノードを更新 */
    contor_renew_node_row[contor_renew_node_row_idx - 1] = maze_goal[1];
    contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[0];

    /* 更新マス用インデックスを増加 */
    contor_renew_node_row_idx++;
  }

  /* 西壁 */
  if (g_direction.West <= 7) {
    i161 = (unsigned char)(1 << g_direction.West);
  } else {
    i161 = 0;
  }

  if ((maze_wall[i155] & i161) == 0) {
    /* 歩数更新 */
    col_num_node[i155] = 3U;

    /* 方向追加 */
    if (g_d_direction.West <= 7) {
      i163 = (unsigned char)(1 << g_d_direction.West);
    } else {
      i163 = 0;
    }

    col_dir_node[i155] = (unsigned char)(col_dir_node[i155] | i163);

    /* 更新ノードを更新 */
    contor_renew_node_col[contor_renew_node_col_idx - 1] = maze_goal[1];
    contor_renew_node_col[contor_renew_node_col_idx + 1023] = maze_goal[0];

    /* 更新マス用インデックスを増加 */
    contor_renew_node_col_idx++;
  }

  /* ゴールセクションが確定していない場合 */
  row_num_node_tmp = max_length->contents;
  qY = row_num_node_tmp - 1U;
  if (qY > (unsigned int)row_num_node_tmp) {
    qY = 0U;
  }

  i = 0U;
  exitg1 = false;
  while ((!exitg1) && (i <= (unsigned short)qY)) {
    /* 更新確認用の歩数カウントは0~max_length */
    change_flag = 0U;

    /* map更新確認用フラグ */
    /* Row_Edgeの処理[33行,32列] */
    /* 検索した座標に対し、歩数mapを更新 */
    i154 = contor_renew_node_row_idx;
    for (row_num_node_tmp = 0; row_num_node_tmp <= i154 - 2; row_num_node_tmp++)
    {
      /* 北側 */
      /* 壁が無い & 探索済みであるとき */
      i155 = contor_renew_node_row[row_num_node_tmp + 1024] - 1;
      i157 = (contor_renew_node_row[row_num_node_tmp] + (i155 << 5)) - 1;
      if (g_direction.North <= 7) {
        i164 = (unsigned char)(1 << g_direction.North);
      } else {
        i164 = 0;
      }

      if (((maze_wall[i157] & i164) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i165 = (unsigned char)(1 << g_direction.North);
        } else {
          i165 = 0;
        }

        if (((maze_wall_search[i157] & i165) != 0) == search->contents.known) {
          /* かつ進行方向が北向きである時 */
          i155 *= 33;
          i157 = (contor_renew_node_row[row_num_node_tmp] + i155) - 1;
          if (g_d_direction.North <= 7) {
            i170 = (unsigned char)(1 << g_d_direction.North);
          } else {
            i170 = 0;
          }

          if ((row_dir_node[i157] & i170) != 0) {
            /* かつ北のノードが更新予定値よりも大きな値の場合 */
            i157 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
            if ((unsigned int)i157 > 255U) {
              i157 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i157 + i155) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i155 + 33 * (contor_renew_node_row[row_num_node_tmp
                + 1024] - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i155 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.North);
              } else {
                row_dir_node[(i155 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i155;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i155 + 33 *
                                (contor_renew_node_row[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                i157 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i186 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i186 = 0;
                }

                row_dir_node[(i155 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[(i157 +
                  33 * (contor_renew_node_row[row_num_node_tmp + 1024] - 1)) - 1]
                  | i186);
              }
            }

            /* かつ進行方向が北向きでないとき */
          } else {
            /* かつ北のノードの歩数MAP値が、更新予定値より大きい場合 */
            i158 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
            if ((unsigned int)i158 > 255U) {
              i158 = 255;
            }

            b_qY = row_num_node[i157] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i158 + i155) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i155 + 33 * (contor_renew_node_row[row_num_node_tmp
                + 1024] - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i155 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.North);
              } else {
                row_dir_node[(i155 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i155;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              i157 = i155;
              if ((unsigned int)i155 > 255U) {
                i157 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i157 + 33 *
                                (contor_renew_node_row[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i157 = i155;
                if ((unsigned int)i155 > 255U) {
                  i157 = 255;
                  i155 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i184 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i184 = 0;
                }

                row_dir_node[(i157 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[(i155 +
                  33 * (contor_renew_node_row[row_num_node_tmp + 1024] - 1)) - 1]
                  | i184);
              }
            }
          }
        }
      }

      /* 北東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.East <= 7) {
        i167 = (unsigned char)(1 << g_direction.East);
      } else {
        i167 = 0;
      }

      if (((maze_wall[(contor_renew_node_row[row_num_node_tmp] +
                       ((contor_renew_node_row[row_num_node_tmp + 1024] - 1) <<
                        5)) - 1] & i167) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i169 = (unsigned char)(1 << g_direction.East);
        } else {
          i169 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_row[row_num_node_tmp] +
                                ((contor_renew_node_row[row_num_node_tmp + 1024]
                 - 1) << 5)) - 1] & i169) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i173 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i173 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i173) != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i155;

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i155
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i194 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i194 = 0;
                }

                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_row[row_num_node_tmp] + ((i157 - 1) << 5))
                  - 1] | i194);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            i157 = (contor_renew_node_row[row_num_node_tmp] + 33 *
                    (contor_renew_node_row[row_num_node_tmp + 1024] - 1)) - 1;
            b_qY = row_num_node[i157] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i155;

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              i158 = i155;
              if ((unsigned int)i155 > 255U) {
                i158 = 255;
              }

              b_qY = row_num_node[i157] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i158
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i157 = i155;
                if ((unsigned int)i155 > 255U) {
                  i157 = 255;
                  i155 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i192 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i192 = 0;
                }

                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i157 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_row[row_num_node_tmp] + ((i155 - 1) << 5))
                  - 1] | i192);
              }
            }
          }
        }
      }

      /* 東側は柱 */
      /* 南東側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
      if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
        c_qY = 0U;
      }

      i155 = (contor_renew_node_row[row_num_node_tmp + 1024] - 1) << 5;
      if (g_direction.East <= 7) {
        i175 = (unsigned char)(1 << g_direction.East);
      } else {
        i175 = 0;
      }

      if (((maze_wall[((int)c_qY + i155) - 1] & i175) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
        if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
          c_qY = 0U;
        }

        if (g_direction.East <= 7) {
          i179 = (unsigned char)(1 << g_direction.East);
        } else {
          i179 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i155) - 1] & i179) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i182 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i182 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i182) != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i157 > 255U) {
              i157 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 10U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i157 - 1) << 5)) - 1] > (int)u9) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = (unsigned
                short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i157;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i157 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                i158 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i158 > 255U) {
                  i158 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i213 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i213 = 0;
                }

                col_dir_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i158 - 1) << 5)) - 1] | i213);
              }
            }

            /* かつ進行方向が南東向きでないとき */
          } else {
            /* かつ南東のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i157 > 255U) {
              i157 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 18U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i157 - 1) << 5)) - 1] > (int)u9) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = (unsigned
                short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i157;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i157 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                i157 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                i158 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i158 > 255U) {
                  i158 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i212 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i212 = 0;
                }

                col_dir_node[((int)c_qY + ((i157 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i158 - 1) << 5)) - 1] | i212);
              }
            }
          }
        }
      }

      /* 南側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
      if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
        c_qY = 0U;
      }

      if (g_direction.South <= 7) {
        i180 = (unsigned char)(1 << g_direction.South);
      } else {
        i180 = 0;
      }

      if (((maze_wall[((int)c_qY + i155) - 1] & i180) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
        if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i185 = (unsigned char)(1 << g_direction.South);
        } else {
          i185 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i155) - 1] & i185) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南向きである時 */
          if (g_d_direction.South <= 7) {
            i191 = (unsigned char)(1 << g_d_direction.South);
          } else {
            i191 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i191) != 0) {
            /* かつ南のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 6U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            if (row_num_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] > (int)u9) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[((int)c_qY + 33 *
                            (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
                - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (g_d_direction.South <= 7) {
                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.South);
              } else {
                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i157;

              /* かつ南のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[((int)c_qY + 33 *
                                (contor_renew_node_row[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South <= 7) {
                  i211 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i211 = 0;
                }

                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[((int)
                  b_qY + 33 * (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] | i211);
              }
            }

            /* かつ進行方向が南向きでないとき */
          } else {
            /* かつ南のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[((int)c_qY + 33 *
                            (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
                - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (g_d_direction.South <= 7) {
                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.South);
              } else {
                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i157;

              /* かつ南のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[((int)c_qY + 33 *
                                (contor_renew_node_row[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South <= 7) {
                  i210 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i210 = 0;
                }

                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[((int)
                  b_qY + 33 * (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] | i210);
              }
            }
          }
        }
      }

      /* 南西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
      if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
        c_qY = 0U;
      }

      if (g_direction.West <= 7) {
        i189 = (unsigned char)(1 << g_direction.West);
      } else {
        i189 = 0;
      }

      if (((maze_wall[((int)c_qY + i155) - 1] & i189) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
        if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i198 = (unsigned char)(1 << g_direction.West);
        } else {
          i198 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i155) - 1] & i198) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i204 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i204 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i204) != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[((int)c_qY + i155) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + i155) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i155) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i155) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + i155) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i219 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i219 = 0;
                }

                col_dir_node[((int)c_qY + i155) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i155) - 1] | i219);
              }
            }

            /* かつ進行方向が南西向きでないとき */
          } else {
            /* かつ南西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[((int)c_qY + i155) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + i155) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i155) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i155) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + i155) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i218 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i218 = 0;
                }

                col_dir_node[((int)c_qY + i155) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i155) - 1] | i218);
              }
            }
          }
        }
      }

      /* 北西側 */
      /* 壁が無い & 探索済みであるとき */
      i155 = (contor_renew_node_row[row_num_node_tmp] + i155) - 1;
      if (g_direction.West <= 7) {
        i200 = (unsigned char)(1 << g_direction.West);
      } else {
        i200 = 0;
      }

      if (((maze_wall[i155] & i200) != 0) == wall->contents.nowall) {
        if (g_direction.West <= 7) {
          i203 = (unsigned char)(1 << g_direction.West);
        } else {
          i203 = 0;
        }

        if (((maze_wall_search[i155] & i203) != 0) == search->contents.known) {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i207 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i207 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i207) != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[row_num_node_tmp] +
                              ((contor_renew_node_row[row_num_node_tmp + 1024] -
                                1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i155] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i155] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i155] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[row_num_node_tmp] +
                                ((contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i217 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i217 = 0;
                }

                col_dir_node[i155] = (unsigned char)(col_dir_node[i155] | i217);
              }
            }

            /* かつ進行方向が北西向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[i155] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i155] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i155] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i155] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[i155] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i216 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i216 = 0;
                }

                col_dir_node[i155] = (unsigned char)(col_dir_node[i155] | i216);
              }
            }
          }
        }
      }
    }

    /* Col_Edgeの処理[32行,33列] */
    /* 検索した座標に対し、歩数mapを更新 */
    i154 = contor_renew_node_col_idx;
    for (row_num_node_tmp = 0; row_num_node_tmp <= i154 - 2; row_num_node_tmp++)
    {
      /* 北側は壁 */
      /* 北東側 */
      /* 壁が無い & 探索済みであるとき */
      i155 = (contor_renew_node_col[row_num_node_tmp] +
              ((contor_renew_node_col[row_num_node_tmp + 1024] - 1) << 5)) - 1;
      if (g_direction.North <= 7) {
        i166 = (unsigned char)(1 << g_direction.North);
      } else {
        i166 = 0;
      }

      if (((maze_wall[i155] & i166) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i168 = (unsigned char)(1 << g_direction.North);
        } else {
          i168 = 0;
        }

        if (((maze_wall_search[i155] & i168) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i172 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i172 = 0;
          }

          if ((col_dir_node[i155] & i172) != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 10U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            if (row_num_node[(i155 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] > (int)u9) {
              /* 歩数MAP更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i155 + 33 * (contor_renew_node_col[row_num_node_tmp
                + 1024] - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i155 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.North_East);
              } else {
                row_dir_node[(i155 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i155;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i155 + 33 *
                                (contor_renew_node_col[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                i157 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i190 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i190 = 0;
                }

                row_dir_node[(i155 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[(i157 +
                  33 * (contor_renew_node_col[row_num_node_tmp + 1024] - 1)) - 1]
                  | i190);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            i157 = i155;
            if ((unsigned int)i155 > 255U) {
              i157 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 18U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            i158 = 33 * (contor_renew_node_col[row_num_node_tmp + 1024] - 1);
            if (row_num_node[(i157 + i158) - 1] > (int)u9) {
              /* 歩数MAP更新(重みづけあり) */
              i157 = i155;
              if ((unsigned int)i155 > 255U) {
                i157 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i157 + i158) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i157 = i155;
              if ((unsigned int)i155 > 255U) {
                i157 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i157 + i158) - 1] = (unsigned char)(1 <<
                  g_d_direction.North_East);
              } else {
                row_dir_node[(i157 + i158) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i155;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i157 = i155;
              if ((unsigned int)i155 > 255U) {
                i157 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i157 + i158) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i157 = i155;
                if ((unsigned int)i155 > 255U) {
                  i157 = 255;
                  i155 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i188 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i188 = 0;
                }

                row_dir_node[(i157 + i158) - 1] = (unsigned char)(row_dir_node
                  [(i155 + i158) - 1] | i188);
              }
            }
          }
        }
      }

      /* 東側 */
      /* 壁が無い & 探索済みであるとき */
      contor_renew_node_row_idx = contor_renew_node_col[row_num_node_tmp + 1024];
      i155 = (contor_renew_node_col[row_num_node_tmp] +
              ((contor_renew_node_row_idx - 1) << 5)) - 1;
      if (g_direction.East <= 7) {
        i171 = (unsigned char)(1 << g_direction.East);
      } else {
        i171 = 0;
      }

      if (((maze_wall[i155] & i171) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i174 = (unsigned char)(1 << g_direction.East);
        } else {
          i174 = 0;
        }

        if (((maze_wall_search[i155] & i174) != 0) == search->contents.known) {
          /* かつ進行方向が東向きである時 */
          if (g_d_direction.East <= 7) {
            i177 = (unsigned char)(1 << g_d_direction.East);
          } else {
            i177 = 0;
          }

          if ((col_dir_node[i155] & i177) != 0) {
            /* かつ東のノードが更新予定値よりも大きな値の場合 */
            i157 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i157 > 255U) {
              i157 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i157 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i157 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i157 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i157 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i157 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i157 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              i157 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i157;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i157 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i157
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i157 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                i158 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i158 > 255U) {
                  i158 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i202 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i202 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i157 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + ((i158 - 1) << 5))
                  - 1] | i202);
              }
            }

            /* かつ進行方向が東向きでないとき */
          } else {
            /* かつ東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i157 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
            i158 = i157;
            if ((unsigned int)i157 > 255U) {
              i158 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i158 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i158 = i157;
              if ((unsigned int)i157 > 255U) {
                i158 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i158 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i158 = i157;
              if ((unsigned int)i157 > 255U) {
                i158 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i158 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i158 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i157;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i158 = i157;
              if ((unsigned int)i157 > 255U) {
                i158 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i158
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i158 = i157;
                if ((unsigned int)i157 > 255U) {
                  i158 = 255;
                  i157 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i199 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i199 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i158 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + ((i157 - 1) << 5))
                  - 1] | i199);
              }
            }
          }
        }
      }

      /* 南東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.South <= 7) {
        i176 = (unsigned char)(1 << g_direction.South);
      } else {
        i176 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] +
                       ((contor_renew_node_col[row_num_node_tmp + 1024] - 1) <<
                        5)) - 1] & i176) != 0) == wall->contents.nowall) {
        if (g_direction.South <= 7) {
          i178 = (unsigned char)(1 << g_direction.South);
        } else {
          i178 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] +
                                ((contor_renew_node_col[row_num_node_tmp + 1024]
                 - 1) << 5)) - 1] & i178) != 0) == search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i181 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i181 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[row_num_node_tmp] +
                             ((contor_renew_node_col[row_num_node_tmp + 1024] -
                               1) << 5)) - 1] & i181) != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                            (contor_renew_node_col[row_num_node_tmp + 1024] - 1))
                - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.South_East <= 7) {
                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_East);
              } else {
                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row_idx;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i157;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                (contor_renew_node_col[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i196 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i196 = 0;
                }

                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + 33 *
                    (contor_renew_node_col[row_num_node_tmp + 1024] - 1)) - 1] |
                  i196);
              }
            }

            /* かつ進行方向が南東向きでないとき */
          } else {
            /* かつ南東のノードの歩数MAP値が、更新予定値より大きい場合 */
            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            i157 = (contor_renew_node_col[row_num_node_tmp] + 33 *
                    (contor_renew_node_col[row_num_node_tmp + 1024] - 1)) - 1;
            if (row_num_node[i157] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[i157] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.South_East <= 7) {
                row_dir_node[i157] = (unsigned char)(1 <<
                  g_d_direction.South_East);
              } else {
                row_dir_node[i157] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row_idx;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i157;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[i155] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[i157] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i197 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i197 = 0;
                }

                row_dir_node[i157] = (unsigned char)(row_dir_node[i157] | i197);
              }
            }
          }
        }
      }

      /* 南側は柱 */
      /* 南西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row_idx - 1U;
      if (c_qY > contor_renew_node_row_idx) {
        c_qY = 0U;
      }

      if (g_direction.South <= 7) {
        i183 = (unsigned char)(1 << g_direction.South);
      } else {
        i183 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] + (((int)c_qY - 1)
              << 5)) - 1] & i183) != 0) == wall->contents.nowall) {
        c_qY = contor_renew_node_row_idx - 1U;
        if (c_qY > contor_renew_node_row_idx) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i187 = (unsigned char)(1 << g_direction.South);
        } else {
          i187 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] + (((int)
                 c_qY - 1) << 5)) - 1] & i187) != 0) == search->contents.known)
        {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i195 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i195 = 0;
          }

          if ((col_dir_node[i155] & i195) != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 * ((int)
                c_qY - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i157;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                ((int)c_qY - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i215 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i215 = 0;
                }

                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                 ((int)b_qY - 1)) - 1] | i215);
              }
            }

            /* かつ進行方向が南西向きでないとき */
          } else {
            /* かつ南西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i155] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 * ((int)
                c_qY - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i157;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i155] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                ((int)c_qY - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i214 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i214 = 0;
                }

                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                 ((int)b_qY - 1)) - 1] | i214);
              }
            }
          }
        }
      }

      /* 西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row_idx - 1U;
      if (c_qY > contor_renew_node_row_idx) {
        c_qY = 0U;
      }

      if (g_direction.West <= 7) {
        i193 = (unsigned char)(1 << g_direction.West);
      } else {
        i193 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] + (((int)c_qY - 1)
              << 5)) - 1] & i193) != 0) == wall->contents.nowall) {
        c_qY = contor_renew_node_row_idx - 1U;
        if (c_qY > contor_renew_node_row_idx) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i201 = (unsigned char)(1 << g_direction.West);
        } else {
          i201 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] + (((int)
                 c_qY - 1) << 5)) - 1] & i201) != 0) == search->contents.known)
        {
          /* かつ進行方向が西向きである時 */
          if (g_d_direction.West <= 7) {
            i206 = (unsigned char)(1 << g_d_direction.West);
          } else {
            i206 = 0;
          }

          if ((col_dir_node[i155] & i206) != 0) {
            /* かつ西のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                   c_qY - 1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                c_qY - 1) << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.West <= 7) {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = (unsigned char)(1 <<
                  g_d_direction.West);
              } else {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                     c_qY - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.West <= 7) {
                  i221 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i221 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + (((int)b_qY - 1) <<
                  5)) - 1] | i221);
              }
            }

            /* かつ進行方向が西向きでないとき */
          } else {
            /* かつ西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                   c_qY - 1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                c_qY - 1) << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.West <= 7) {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = (unsigned char)(1 <<
                  g_d_direction.West);
              } else {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i157 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i157;

              /* かつ西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i155] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                     c_qY - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.West <= 7) {
                  i220 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i220 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + (((int)b_qY - 1) <<
                  5)) - 1] | i220);
              }
            }
          }
        }
      }

      /* 北西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row_idx - 1U;
      if (c_qY > contor_renew_node_row_idx) {
        c_qY = 0U;
      }

      if (g_direction.North <= 7) {
        i205 = (unsigned char)(1 << g_direction.North);
      } else {
        i205 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] + (((int)c_qY - 1)
              << 5)) - 1] & i205) != 0) == wall->contents.nowall) {
        c_qY = contor_renew_node_row_idx - 1U;
        if (c_qY > contor_renew_node_row_idx) {
          c_qY = 0U;
        }

        if (g_direction.North <= 7) {
          i208 = (unsigned char)(1 << g_direction.North);
        } else {
          i208 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] + (((int)
                 c_qY - 1) << 5)) - 1] & i208) != 0) == search->contents.known)
        {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i209 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i209 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[row_num_node_tmp] +
                             ((contor_renew_node_col[row_num_node_tmp + 1024] -
                               1) << 5)) - 1] & i209) != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i155 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i155;
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i155 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                i157 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i223 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i223 = 0;
                }

                row_dir_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i157 + 33 * ((int)b_qY - 1)) - 1] | i223);
              }
            }

            /* かつ進行方向が北西向きでないとき */
          } else {
            /* かつ北西のノードの歩数MAP値が、更新予定値より大きい場合 */
            i157 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            if ((unsigned int)i157 > 255U) {
              i157 = 255;
            }

            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i155] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i157 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i155;
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              i157 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i157 > 255U) {
                i157 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i155] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i157 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                i157 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i157 > 255U) {
                  i157 = 255;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i222 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i222 = 0;
                }

                row_dir_node[(i155 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i157 + 33 * ((int)b_qY - 1)) - 1] | i222);
              }
            }
          }
        }
      }
    }

    /* ゴール更新ノードの更新とインデックスのクリア */
    contor_renew_node_col_idx = contor_renew_node_col_idx_temp;
    contor_renew_node_col_idx_temp = 1U;
    for (i154 = 0; i154 < 2048; i154++) {
      contor_renew_node_col[i154] = contor_renew_node_col_temp[i154];
      contor_renew_node_col_temp[i154] = 0U;
      contor_renew_node_row[i154] = contor_renew_node_row_temp[i154];
      contor_renew_node_row_temp[i154] = 0U;
    }

    contor_renew_node_row_idx = contor_renew_node_row_idx_temp;
    contor_renew_node_row_idx_temp = 1U;

    /* 更新がなければ終了(スタート地点の歩数マップを更新) */
    if (change_flag == 0) {
      b_qY = row_num_node[1] + 3U;
      if (b_qY > 65535U) {
        b_qY = 65535U;
      }

      *start_num = (unsigned short)b_qY;
      exitg1 = true;
    } else {
      i++;
    }
  }
}

/*
 * 入力　現在位置x,y,現在方向,迷路行方向サイズ,迷路列方向サイズ,迷路壁情報,迷路壁の探索情報,ゴール座標
 * 出力  現在位置x,y,現在方向,壁情報,探索情報
 * Arguments    : const coder_internal_ref_6 *wall
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_5 *search
 *                const coder_internal_ref_2 *maze_goal
 *                const coder_internal_ref_1 *maze_wall
 *                const coder_internal_ref_4 *adachi_search_mode
 *                unsigned char *current_x
 *                unsigned char *current_y
 *                unsigned char *current_dir
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char b_maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 *                const unsigned char exploration_goal[2]
 *                unsigned char *start_flg
 *                unsigned char stop_flg
 *                unsigned char goal_after_flg
 *                unsigned char adachi_s_mode
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void b_search_adachi(const coder_internal_ref_6 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_5 *search, const coder_internal_ref_2
  *maze_goal, const coder_internal_ref_1 *maze_wall, const coder_internal_ref_4 *
  adachi_search_mode, unsigned char *current_x, unsigned char *current_y,
  unsigned char *current_dir, unsigned char maze_row_size, unsigned char
  maze_col_size, unsigned char b_maze_wall[1024], unsigned char
  maze_wall_search[1024], const unsigned char exploration_goal[2], unsigned char
  *start_flg, unsigned char stop_flg, unsigned char goal_after_flg, unsigned
  char adachi_s_mode, unsigned short contour_map[1024])
{
  unsigned char straight_count;
  unsigned char goal_flg;
  unsigned char contour_flg;
  int exitg1;
  unsigned char temp_maze_wall_tmp;
  unsigned char error_flg;
  int q0;
  unsigned int qY;

  /*     %% search_adachi 足立法での探索 */
  /* local変数宣言 */
  straight_count = 0U;
  goal_flg = 0U;

  /* ゴール判定フラグ */
  /* 壁情報更新確認用変数 */
  contour_flg = 0U;

  /*      search_start_x = current_x %探索開始時x */
  /*      search_start_y = current_y %探索開始時y */
  /* 初回のコンターマップ作製 */
  b_make_map_find(wall, exploration_goal, b_maze_wall, *current_x, *current_y,
                  contour_map);
  do {
    exitg1 = 0;

    /* 壁情報取得 */
    /* ゴール直後は壁情報を更新しない */
    if (goal_after_flg != 1) {
      /* 既知区間加速時でなければ、壁情報を取得しない */
      if (straight_count == 0) {
        temp_maze_wall_tmp = b_maze_wall[(*current_y + ((*current_x - 1) << 5))
          - 1];
        wall_set(wall, wall_flg, search, maze_goal, maze_row_size, maze_col_size,
                 *current_x, *current_y, *current_dir, b_maze_wall,
                 maze_wall_search);

        /* 壁情報が更新されれば、コンター更新のフラグを立てる。 */
        contour_flg = (unsigned char)(temp_maze_wall_tmp != b_maze_wall
          [(*current_y + ((*current_x - 1) << 5)) - 1]);
      }
    } else {
      /* ゴール直後のとき */
      goal_after_flg = 0U;

      /* ゴール直後フラグをクリア */
    }

    /*  等高線MAP生成 */
    /* 壁情報に変更があった場合のみ */
    if (contour_flg != 0) {
      b_make_map_find(wall, exploration_goal, b_maze_wall, *current_x,
                      *current_y, contour_map);
    }

    /* 現在位置がゴールか判定 */
    if ((*current_x == exploration_goal[0]) && (*current_y == exploration_goal[1]))
    {
      goal_flg = 1U;
    }

    /* 探索モードの場合、対象のマスがすべて探索済みのとき、ゴールフラグを立てる */
    if (adachi_s_mode == adachi_search_mode->contents.search) {
      goal_flg = 1U;

      /* ゴール座標が未探索であれば、フラグをおろし、ブレイク */
      if (maze_wall_search[(exploration_goal[1] + ((exploration_goal[0] - 1) <<
            5)) - 1] != 15) {
        goal_flg = 0U;
      }
    }

    /* ゴール時処理 */
    if (goal_flg == 1) {
      exitg1 = 1;
    } else {
      /*  進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      get_nextdir2(*current_x, *current_y, b_maze_wall, contour_map,
                   &temp_maze_wall_tmp, &error_flg);

      /* 進行方向が見つからなかったときのエラー処理 */
      if (error_flg == 1) {
        m_error_movement(1);
      }

      /*  現在方向と進行方向に応じた処理 */
      q0 = (int)(4U + temp_maze_wall_tmp);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      qY = (unsigned int)q0 - *current_dir;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      temp_maze_wall_tmp = b_rem((unsigned char)qY);
      if (l_direction.front == temp_maze_wall_tmp) {
        q0 = 0;
      } else if (l_direction.right == temp_maze_wall_tmp) {
        q0 = 1;
      } else if (l_direction.back == temp_maze_wall_tmp) {
        q0 = 2;
      } else if (l_direction.left == temp_maze_wall_tmp) {
        q0 = 3;
      } else {
        q0 = -1;
      }

      switch (q0) {
       case 0:
        /* 直進制御時、壁フラグを現在参照位置で更新 */
        if (straight_count > 0) {
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* 参照位置を次回の参照位置に変更 */
        move_step(current_x, current_y, *current_dir);

        /* 移動先が探索済みであるなら、ストレートカウンタをインクリメント */
        if (maze_wall_search[(*current_y + ((*current_x - 1) << 5)) - 1] == 15)
        {
          q0 = (int)(straight_count + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          straight_count = (unsigned char)q0;

          /* 探索済みでない場合 */
        } else {
          /* 直線カウンタがある場合、移動 */
          /* 現在方向も直進なので、直進カウンタ+1だけ移動する。 */
          if (straight_count > 0) {
            q0 = (int)(straight_count + 1U);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            m_move_front_long((unsigned char)q0, *start_flg, wall_flg->contents,
                              move_dir_property.straight);
            straight_count = 0U;

            /* スタート直後フラグをクリア */
            *start_flg = 0U;

            /* 壁フラグをクリア */
            wall_flg->contents = 0U;
          } else {
            /* 直進カウンタがない場合 */
            m_move_front(*start_flg, wall_flg->contents,
                         move_dir_property.straight);

            /* スタート直後フラグをクリア */
            *start_flg = 0U;

            /* 壁フラグをクリア */
            wall_flg->contents = 0U;
          }
        }
        break;

       case 1:
        /* 直進カウンタがある場合、移動 */
        if (straight_count > 0) {
          /* 前回位置での壁フラグを使用し、マウスを移動させる（C) */
          m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                            move_dir_property.straight);
          straight_count = 0U;

          /* スタート直後フラグをクリア */
          *start_flg = 0U;

          /* 壁フラグをクリア */
          wall_flg->contents = 0U;

          /* 現在参照位置で壁フラグを更新 */
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* マウスを移動させる（C) */
        m_move_right(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* 参照位置、方向を更新 */
        turn_clk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 2:
        /* disp("back") */
        /* 直進カウンタがある場合、移動 */
        if (straight_count > 0) {
          /* 前回位置での壁フラグを使用し、マウスを移動させる（C) */
          m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                            move_dir_property.straight);
          straight_count = 0U;

          /* スタート直後フラグをクリア */
          *start_flg = 0U;

          /* 壁フラグをクリア */
          wall_flg->contents = 0U;

          /* 現在参照位置で壁フラグを更新 */
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* マウスを移動させる（C） */
        m_move_back(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* 参照位置、方向を更新 */
        turn_180deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 3:
        /* 直進カウンタがある場合、移動 */
        if (straight_count > 0) {
          /* 前回位置での壁フラグを使用し、マウスを移動させる（C) */
          m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                            move_dir_property.straight);
          straight_count = 0U;

          /* スタート直後フラグをクリア */
          *start_flg = 0U;

          /* 壁フラグをクリア */
          wall_flg->contents = 0U;

          /* 現在参照位置で壁フラグを更新 */
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* マウスを移動させる（C） */
        m_move_left(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* 参照位置、方向を更新 */
        turn_conclk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;
      }

      /* for code generation */
    }
  } while (exitg1 == 0);

  /* 直進カウンタ分移動 */
  if (straight_count > 0) {
    m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                      move_dir_property.straight);

    /* スタート直後フラグをクリア */
    *start_flg = 0U;

    /* 壁フラグをクリア */
    wall_flg->contents = 0U;
  }

  if (stop_flg == 1) {
    /* ゴール時停止フラグが立っているとき */
    /* 停止動作を実施 */
    m_goal_movement(*start_flg, wall_flg->contents, move_dir_property.straight);
  }

  /* ゴール時停止フラグが立っていなければ、動作させたまま終了 */
}

/*
 * スタートノード,方向を設定
 * Arguments    : const unsigned char maze_goal[18]
 *                unsigned char goal_size
 *                const unsigned short row_num_node[1056]
 *                const unsigned short col_num_node[1056]
 *                unsigned char goal_node[2]
 *                unsigned char *goal_matrix_dir
 *                unsigned char *goal_dir
 * Return Type  : void
 */
static void decide_goal_node_dir(const unsigned char maze_goal[18], unsigned
  char goal_size, const unsigned short row_num_node[1056], const unsigned short
  col_num_node[1056], unsigned char goal_node[2], unsigned char *goal_matrix_dir,
  unsigned char *goal_dir)
{
  unsigned char next_matrix_dir;
  unsigned char next_node_idx_0;
  unsigned char next_node_idx_1;
  unsigned short map_min;
  int i150;
  int exitg1;
  unsigned char goal_flag;
  int i;
  bool guard1 = false;
  int i151;
  bool guard2 = false;
  bool guard3 = false;
  bool guard4 = false;
  unsigned int qY;
  unsigned short u8;

  /*     %% decide_goal_node_dir ゴールノード、および方向の確定 */
  /*  斜めのコンターマップから、ゴールとなるノードと、ゴール時の進入角度を確定する。 */
  *goal_matrix_dir = matrix_dir.Row;

  /* スタートノードは行方向 */
  /* スタートマスの北のノード */
  next_matrix_dir = matrix_dir.Row;

  /* 進行方向先のノード方向（仮） */
  goal_node[0] = 2U;
  next_node_idx_0 = 2U;
  goal_node[1] = 1U;
  next_node_idx_1 = 1U;

  /* 進行方向先のノード座標 */
  *goal_dir = g_d_direction.North;

  /* 進行方向（最初の仮の値は北向き） */
  map_min = MAX_uint16_T;
  i150 = goal_size;
  do {
    exitg1 = 0;
    goal_flag = 0U;
    for (i = 0; i < i150; i++) {
      /* 現在のノードが行方向の時 */
      if (*goal_matrix_dir == matrix_dir.Row) {
        /*  %x */
        if (maze_goal[i] == goal_node[1]) {
          guard1 = false;
          if (maze_goal[i + 9] == goal_node[0]) {
            guard1 = true;
          } else {
            i151 = (int)(maze_goal[i + 9] + 1U);
            if ((unsigned int)i151 > 255U) {
              i151 = 255;
            }

            if (i151 == goal_node[0]) {
              guard1 = true;
            }
          }

          if (guard1) {
            /* y座標の一致 */
            /* 進行方向先に壁があれば(マップ値が65535であれば)、進行方向を変更する（斜め侵入時） */
            /* 北東 */
            guard2 = false;
            guard3 = false;
            guard4 = false;
            if (*goal_dir == g_d_direction.North_East) {
              i151 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              if (col_num_node[(goal_node[0] + ((i151 - 1) << 5)) - 1] == 65535)
              {
                *goal_dir = g_d_direction.North;

                /* 南東 */
              } else {
                guard4 = true;
              }
            } else {
              guard4 = true;
            }

            if (guard4) {
              if (*goal_dir == g_d_direction.South_East) {
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                i151 = (int)(goal_node[1] + 1U);
                if ((unsigned int)i151 > 255U) {
                  i151 = 255;
                }

                if (col_num_node[((int)qY + ((i151 - 1) << 5)) - 1] == 65535) {
                  *goal_dir = g_d_direction.South;

                  /* 南西 */
                } else {
                  guard3 = true;
                }
              } else {
                guard3 = true;
              }
            }

            if (guard3) {
              if (*goal_dir == g_d_direction.South_West) {
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                if (col_num_node[((int)qY + ((goal_node[1] - 1) << 5)) - 1] ==
                    65535) {
                  *goal_dir = g_d_direction.South;

                  /* 北西 */
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2 && ((*goal_dir == g_d_direction.North_West) &&
                           (col_num_node[(goal_node[0] + ((goal_node[1] - 1) <<
                    5)) - 1] == 65535))) {
              *goal_dir = g_d_direction.North;
            }

            /* フラグ */
            goal_flag = 1U;
          }
        }

        /* 現在のノードが列方向の時 */
      } else {
        /*  %x */
        guard1 = false;
        if (maze_goal[i] == goal_node[1]) {
          guard1 = true;
        } else {
          i151 = (int)(maze_goal[i] + 1U);
          if ((unsigned int)i151 > 255U) {
            i151 = 255;
          }

          if (i151 == goal_node[1]) {
            guard1 = true;
          }
        }

        if (guard1 && (maze_goal[i + 9] == goal_node[0])) {
          /* y座標の一致 */
          /* 進行方向先に壁があれば(マップ値が65535であれば)、進行方向を変更する（斜め侵入時） */
          /* 北東 */
          guard2 = false;
          guard3 = false;
          if (*goal_dir == g_d_direction.North_East) {
            i151 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i151 > 255U) {
              i151 = 255;
            }

            if (row_num_node[(i151 + 33 * (goal_node[1] - 1)) - 1] == 65535) {
              *goal_dir = g_d_direction.East;

              /* 南東 */
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }

          if (guard3) {
            if ((*goal_dir == g_d_direction.South_East) && (row_num_node
                 [(goal_node[0] + 33 * (goal_node[1] - 1)) - 1] == 65535)) {
              *goal_dir = g_d_direction.East;

              /* 南西 */
            } else if (*goal_dir == g_d_direction.South_West) {
              qY = goal_node[1] - 1U;
              if (qY > goal_node[1]) {
                qY = 0U;
              }

              if (row_num_node[(goal_node[0] + 33 * ((int)qY - 1)) - 1] == 65535)
              {
                *goal_dir = g_d_direction.West;

                /* 北西 */
              } else {
                guard2 = true;
              }
            } else {
              guard2 = true;
            }
          }

          if (guard2 && (*goal_dir == g_d_direction.North_West)) {
            i151 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i151 > 255U) {
              i151 = 255;
            }

            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            if (row_num_node[(i151 + 33 * ((int)qY - 1)) - 1] == 65535) {
              *goal_dir = g_d_direction.West;
            }
          }

          /* フラグ */
          goal_flag = 1U;
        }
      }
    }

    /*              disp("current_node") */
    /*              disp(current_node) */
    /*              disp("current_matrix_dir") */
    /*              disp(current_matrix_dir) */
    /*              disp("current_dir_dgnd") */
    /*              disp(current_dir_dgnd) */
    /* ゴールノード判定の時 */
    /* 現在のノードをゴールノードとし、 */
    /* 現在の方向をゴール時の方向とする。 */
    if (goal_flag == 1) {
      exitg1 = 1;
    } else {
      /* 現在のノードの方向から優先的に進行方向を確定 */
      for (i = 0; i < 8; i++) {
        i151 = *goal_dir + i;
        if (i151 > 255) {
          i151 = 255;
        }

        goal_flag = (unsigned char)(i151 % 8);

        /* 現在のノードが行方向の時 */
        if (*goal_matrix_dir == matrix_dir.Row) {
          if (goal_flag == g_d_direction.North) {
            i151 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i151 > 255U) {
              i151 = 255;
            }

            if (row_num_node[(i151 + 33 * (goal_node[1] - 1)) - 1] < map_min) {
              /* 最小値を更新 */
              i151 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              map_min = row_num_node[(i151 + 33 * (goal_node[1] - 1)) - 1];

              /* 現在ノードの進行方向を北向きに */
              *goal_dir = g_d_direction.North;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Row;
              i151 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              next_node_idx_0 = (unsigned char)i151;
              next_node_idx_1 = goal_node[1];
            }
          } else if (goal_flag == g_d_direction.North_East) {
            i151 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i151 > 255U) {
              i151 = 255;
            }

            if (col_num_node[(goal_node[0] + ((i151 - 1) << 5)) - 1] < map_min)
            {
              /* 最小値を更新 */
              i151 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              map_min = col_num_node[(goal_node[0] + ((i151 - 1) << 5)) - 1];

              /* 現在ノードの進行方向を北向きに */
              *goal_dir = g_d_direction.North_East;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Col;
              next_node_idx_0 = goal_node[0];
              i151 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              next_node_idx_1 = (unsigned char)i151;
            }
          } else if (goal_flag != g_d_direction.East) {
            if (goal_flag == g_d_direction.South_East) {
              qY = goal_node[0] - 1U;
              if (qY > goal_node[0]) {
                qY = 0U;
              }

              i151 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              if (col_num_node[((int)qY + ((i151 - 1) << 5)) - 1] < map_min) {
                /* 最小値を更新 */
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                i151 = (int)(goal_node[1] + 1U);
                if ((unsigned int)i151 > 255U) {
                  i151 = 255;
                }

                map_min = col_num_node[((int)qY + ((i151 - 1) << 5)) - 1];

                /* 現在ノードの進行方向を北向きに */
                *goal_dir = g_d_direction.South_East;

                /* 進行方向先の座標、行列の方向を更新 */
                next_matrix_dir = matrix_dir.Col;
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                next_node_idx_0 = (unsigned char)qY;
                i151 = (int)(goal_node[1] + 1U);
                if ((unsigned int)i151 > 255U) {
                  i151 = 255;
                }

                next_node_idx_1 = (unsigned char)i151;
              }
            } else if (goal_flag == g_d_direction.South) {
              qY = goal_node[0] - 1U;
              if (qY > goal_node[0]) {
                qY = 0U;
              }

              if (row_num_node[((int)qY + 33 * (goal_node[1] - 1)) - 1] <
                  map_min) {
                /* 最小値を更新 */
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                map_min = row_num_node[((int)qY + 33 * (goal_node[1] - 1)) - 1];

                /* 現在ノードの進行方向を北向きに */
                *goal_dir = g_d_direction.South;

                /* 進行方向先の座標、行列の方向を更新 */
                next_matrix_dir = matrix_dir.Row;
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                next_node_idx_0 = (unsigned char)qY;
                next_node_idx_1 = goal_node[1];
              }
            } else if (goal_flag == g_d_direction.South_West) {
              qY = goal_node[0] - 1U;
              if (qY > goal_node[0]) {
                qY = 0U;
              }

              if (col_num_node[((int)qY + ((goal_node[1] - 1) << 5)) - 1] <
                  map_min) {
                /* 最小値を更新 */
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                map_min = col_num_node[((int)qY + ((goal_node[1] - 1) << 5)) - 1];

                /* 現在ノードの進行方向を北向きに */
                *goal_dir = g_d_direction.South_West;

                /* 進行方向先の座標、行列の方向を更新 */
                next_matrix_dir = matrix_dir.Col;
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                next_node_idx_0 = (unsigned char)qY;
                next_node_idx_1 = goal_node[1];
              }
            } else if ((goal_flag != g_d_direction.West) && (goal_flag ==
                        g_d_direction.North_West) && (col_num_node[(goal_node[0]
              + ((goal_node[1] - 1) << 5)) - 1] < map_min)) {
              /* 最小値を更新 */
              map_min = col_num_node[(goal_node[0] + ((goal_node[1] - 1) << 5))
                - 1];

              /* 現在ノードの進行方向を北向きに */
              *goal_dir = g_d_direction.North_West;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Col;
              next_node_idx_0 = goal_node[0];
              next_node_idx_1 = goal_node[1];
            } else {
              /* 柱 */
            }
          } else {
            /* 柱 */
          }

          /* 現在のノードが列方向の時 */
        } else if (goal_flag != g_d_direction.North) {
          if (goal_flag == g_d_direction.North_East) {
            i151 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i151 > 255U) {
              i151 = 255;
            }

            if (row_num_node[(i151 + 33 * (goal_node[1] - 1)) - 1] < map_min) {
              /* 最小値を更新 */
              i151 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              map_min = row_num_node[(i151 + 33 * (goal_node[1] - 1)) - 1];

              /* 現在ノードの進行方向を北東向きに */
              *goal_dir = g_d_direction.North_East;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Row;
              i151 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              next_node_idx_0 = (unsigned char)i151;
              next_node_idx_1 = goal_node[1];
            }
          } else if (goal_flag == g_d_direction.East) {
            i151 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i151 > 255U) {
              i151 = 255;
            }

            if (col_num_node[(goal_node[0] + ((i151 - 1) << 5)) - 1] < map_min)
            {
              /* 最小値を更新 */
              i151 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              map_min = col_num_node[(goal_node[0] + ((i151 - 1) << 5)) - 1];

              /* 現在ノードの進行方向を東向きに */
              *goal_dir = g_d_direction.East;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Col;
              next_node_idx_0 = goal_node[0];
              i151 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i151 > 255U) {
                i151 = 255;
              }

              next_node_idx_1 = (unsigned char)i151;
            }
          } else if (goal_flag == g_d_direction.South_East) {
            u8 = row_num_node[(goal_node[0] + 33 * (goal_node[1] - 1)) - 1];
            if (u8 < map_min) {
              /* 最小値を更新 */
              map_min = u8;

              /* 現在ノードの進行方向を南東向きに */
              *goal_dir = g_d_direction.South_East;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Row;
              next_node_idx_0 = goal_node[0];
              next_node_idx_1 = goal_node[1];
            }
          } else if (goal_flag != g_d_direction.South) {
            if (goal_flag == g_d_direction.South_West) {
              qY = goal_node[1] - 1U;
              if (qY > goal_node[1]) {
                qY = 0U;
              }

              if (row_num_node[(goal_node[0] + 33 * ((int)qY - 1)) - 1] <
                  map_min) {
                /* 最小値を更新 */
                qY = goal_node[1] - 1U;
                if (qY > goal_node[1]) {
                  qY = 0U;
                }

                map_min = row_num_node[(goal_node[0] + 33 * ((int)qY - 1)) - 1];

                /* 現在ノードの進行方向を南西向きに */
                *goal_dir = g_d_direction.South_West;

                /* 進行方向先の座標、行列の方向を更新 */
                next_matrix_dir = matrix_dir.Row;
                next_node_idx_0 = goal_node[0];
                qY = goal_node[1] - 1U;
                if (qY > goal_node[1]) {
                  qY = 0U;
                }

                next_node_idx_1 = (unsigned char)qY;
              }
            } else if (goal_flag == g_d_direction.West) {
              qY = goal_node[1] - 1U;
              if (qY > goal_node[1]) {
                qY = 0U;
              }

              if (col_num_node[(goal_node[0] + (((int)qY - 1) << 5)) - 1] <
                  map_min) {
                /* 最小値を更新 */
                qY = goal_node[1] - 1U;
                if (qY > goal_node[1]) {
                  qY = 0U;
                }

                map_min = col_num_node[(goal_node[0] + (((int)qY - 1) << 5)) - 1];

                /* 現在ノードの進行方向を西向きに */
                *goal_dir = g_d_direction.West;

                /* 進行方向先の座標、行列の方向を更新 */
                next_matrix_dir = matrix_dir.Col;
                next_node_idx_0 = goal_node[0];
                qY = goal_node[1] - 1U;
                if (qY > goal_node[1]) {
                  qY = 0U;
                }

                next_node_idx_1 = (unsigned char)qY;
              }
            } else {
              if (goal_flag == g_d_direction.North_West) {
                i151 = (int)(goal_node[0] + 1U);
                if ((unsigned int)i151 > 255U) {
                  i151 = 255;
                }

                qY = goal_node[1] - 1U;
                if (qY > goal_node[1]) {
                  qY = 0U;
                }

                if (row_num_node[(i151 + 33 * ((int)qY - 1)) - 1] < map_min) {
                  /* 最小値を更新 */
                  i151 = (int)(goal_node[0] + 1U);
                  if ((unsigned int)i151 > 255U) {
                    i151 = 255;
                  }

                  qY = goal_node[1] - 1U;
                  if (qY > goal_node[1]) {
                    qY = 0U;
                  }

                  map_min = row_num_node[(i151 + 33 * ((int)qY - 1)) - 1];

                  /* 現在ノードの進行方向を北西向きに */
                  *goal_dir = g_d_direction.North_West;

                  /* 進行方向先の座標、行列の方向を更新 */
                  next_matrix_dir = matrix_dir.Row;
                  i151 = (int)(goal_node[0] + 1U);
                  if ((unsigned int)i151 > 255U) {
                    i151 = 255;
                  }

                  next_node_idx_0 = (unsigned char)i151;
                  qY = goal_node[1] - 1U;
                  if (qY > goal_node[1]) {
                    qY = 0U;
                  }

                  next_node_idx_1 = (unsigned char)qY;
                }
              }
            }
          } else {
            /* 柱 */
          }
        } else {
          /* 柱 */
        }
      }

      goal_node[0] = next_node_idx_0;
      goal_node[1] = next_node_idx_1;
      *goal_matrix_dir = next_matrix_dir;
    }
  } while (exitg1 == 0);

  /*  */
}

/*
 * Arguments    : const unsigned char maze_goal[18]
 *                const unsigned char goal_node[2]
 *                unsigned char goal_matrix_dir
 *                unsigned char goal_dir
 *                unsigned char goal_section[2]
 *                unsigned char goal_node2[2]
 *                unsigned char *goal_matrix_dir2
 * Return Type  : void
 */
static void decide_goal_section(const unsigned char maze_goal[18], const
  unsigned char goal_node[2], unsigned char goal_matrix_dir, unsigned char
  goal_dir, unsigned char goal_section[2], unsigned char goal_node2[2], unsigned
  char *goal_matrix_dir2)
{
  int i152;
  int ex;
  unsigned int qY;
  int k;
  unsigned int b_qY;
  unsigned char uv0[18];
  bool temp1[18];
  signed char varargin_1[9];

  /*     %% decide_goal_section ゴールマスの確定 */
  /*  確定されたゴールのノードと、ゴール時の進入角度から、 */
  /*  ゴールマスを確定する。 */
  /* (y,x) */
  goal_section[0] = 1U;
  goal_node2[0] = 1U;
  goal_section[1] = 1U;
  goal_node2[1] = 1U;
  *goal_matrix_dir2 = matrix_dir.Row;
  if (goal_dir == g_d_direction.North) {
    if (goal_matrix_dir == matrix_dir.Row) {
      i152 = (int)(goal_node[0] + 1U);
      ex = i152;
      if ((unsigned int)i152 > 255U) {
        ex = 255;
      }

      if (goal_judge(maze_goal, goal_node[1], (unsigned char)ex) != 0.0) {
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_section[0] = (unsigned char)i152;
        goal_section[1] = goal_node[1];

        /* (y,x) */
        goal_node2[0] = (unsigned char)i152;
        goal_node2[1] = goal_node[1];
      } else {
        goal_section[0] = goal_node[0];
        goal_section[1] = goal_node[1];

        /* (y,x) */
        goal_node2[0] = goal_node[0];
        goal_node2[1] = goal_node[1];
      }
    }

    /* 列方向は柱なのでなし */
  } else if (goal_dir == g_d_direction.North_East) {
    if (goal_matrix_dir == matrix_dir.Row) {
      i152 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i152 > 255U) {
        i152 = 255;
      }

      ex = (int)(goal_node[0] + 1U);
      if ((unsigned int)ex > 255U) {
        ex = 255;
      }

      if (goal_judge(maze_goal, (unsigned char)i152, (unsigned char)ex) != 0.0)
      {
        i152 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_section[0] = (unsigned char)i152;
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_section[1] = (unsigned char)i152;

        /* (y,x) */
        i152 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_node2[0] = (unsigned char)i152;
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_node2[1] = (unsigned char)i152;
      } else {
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        if (goal_judge(maze_goal, (unsigned char)i152, goal_node[0]) != 0.0) {
          goal_section[0] = goal_node[0];
          i152 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_section[1] = (unsigned char)i152;

          /* (y,x) */
          goal_node2[0] = goal_node[0];
          i152 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_node2[1] = (unsigned char)i152;
          *goal_matrix_dir2 = matrix_dir.Col;
        } else {
          i152 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          if (goal_judge(maze_goal, goal_node[1], (unsigned char)i152) != 0.0) {
            i152 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            goal_section[0] = (unsigned char)i152;
            goal_section[1] = goal_node[1];

            /* (y,x) */
            i152 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            goal_node2[0] = (unsigned char)i152;
            goal_node2[1] = goal_node[1];
          } else {
            goal_section[0] = goal_node[0];
            goal_section[1] = goal_node[1];

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            goal_node2[1] = goal_node[1];
          }
        }
      }
    } else {
      i152 = (int)(goal_node[1] + 1U);
      ex = i152;
      if ((unsigned int)i152 > 255U) {
        ex = 255;
      }

      k = (int)(goal_node[0] + 1U);
      if ((unsigned int)k > 255U) {
        k = 255;
      }

      if (goal_judge(maze_goal, (unsigned char)ex, (unsigned char)k) != 0.0) {
        ex = (int)(goal_node[0] + 1U);
        if ((unsigned int)ex > 255U) {
          ex = 255;
        }

        goal_section[0] = (unsigned char)ex;
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_section[1] = (unsigned char)i152;

        /* (y,x) */
        goal_node2[0] = (unsigned char)ex;
        goal_node2[1] = (unsigned char)i152;
        *goal_matrix_dir2 = matrix_dir.Col;
      } else {
        ex = (int)(goal_node[0] + 1U);
        if ((unsigned int)ex > 255U) {
          ex = 255;
        }

        if (goal_judge(maze_goal, goal_node[1], (unsigned char)ex) != 0.0) {
          i152 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_section[0] = (unsigned char)i152;
          goal_section[1] = goal_node[1];

          /* (y,x) */
          i152 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_node2[0] = (unsigned char)i152;
          goal_node2[1] = goal_node[1];
        } else {
          ex = i152;
          if ((unsigned int)i152 > 255U) {
            ex = 255;
          }

          if (goal_judge(maze_goal, (unsigned char)ex, goal_node[0]) != 0.0) {
            goal_section[0] = goal_node[0];
            ex = i152;
            if ((unsigned int)i152 > 255U) {
              ex = 255;
            }

            goal_section[1] = (unsigned char)ex;

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            goal_node2[1] = (unsigned char)i152;
            *goal_matrix_dir2 = matrix_dir.Col;
          } else {
            goal_section[0] = goal_node[0];
            goal_section[1] = goal_node[1];

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            goal_node2[1] = goal_node[1];
            *goal_matrix_dir2 = matrix_dir.Col;
          }
        }
      }
    }
  } else if (goal_dir == g_d_direction.East) {
    /* 行方向は柱なのでなし */
    if (goal_matrix_dir == matrix_dir.Col) {
      i152 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i152 > 255U) {
        i152 = 255;
      }

      if (goal_judge(maze_goal, (unsigned char)i152, goal_node[0]) != 0.0) {
        goal_section[0] = goal_node[0];
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_section[1] = (unsigned char)i152;

        /* (y,x) */
        goal_node2[0] = goal_node[0];
        goal_node2[1] = (unsigned char)i152;
        *goal_matrix_dir2 = matrix_dir.Col;
      } else {
        goal_section[0] = goal_node[0];
        goal_section[1] = goal_node[1];

        /* (y,x) */
        goal_node2[0] = goal_node[0];
        goal_node2[1] = goal_node[1];
        *goal_matrix_dir2 = matrix_dir.Col;
      }
    }
  } else if (goal_dir == g_d_direction.South_East) {
    if (goal_matrix_dir == matrix_dir.Row) {
      i152 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i152 > 255U) {
        i152 = 255;
      }

      qY = goal_node[0] - 2U;
      if (qY > goal_node[0]) {
        qY = 0U;
      }

      if (goal_judge(maze_goal, (unsigned char)i152, (unsigned char)qY) != 0.0)
      {
        qY = goal_node[0] - 2U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_section[1] = (unsigned char)i152;

        /* (y,x) */
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_node2[0] = (unsigned char)qY;
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_node2[1] = (unsigned char)i152;
      } else {
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        if (goal_judge(maze_goal, (unsigned char)i152, (unsigned char)qY) != 0.0)
        {
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_section[0] = (unsigned char)qY;
          i152 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_section[1] = (unsigned char)i152;

          /* (y,x) */
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_node2[0] = (unsigned char)qY;
          i152 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_node2[1] = (unsigned char)i152;
          *goal_matrix_dir2 = matrix_dir.Col;
        } else {
          qY = goal_node[0] - 2U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          if (goal_judge(maze_goal, goal_node[1], (unsigned char)qY) != 0.0) {
            qY = goal_node[0] - 1U;
            if (qY > goal_node[0]) {
              qY = 0U;
            }

            goal_section[0] = (unsigned char)qY;
            goal_section[1] = goal_node[1];

            /* (y,x) */
            qY = goal_node[0] - 1U;
            if (qY > goal_node[0]) {
              qY = 0U;
            }

            goal_node2[0] = (unsigned char)qY;
            goal_node2[1] = goal_node[1];
          } else {
            qY = goal_node[0] - 1U;
            if (qY > goal_node[0]) {
              qY = 0U;
            }

            goal_section[0] = (unsigned char)qY;
            goal_section[1] = goal_node[1];

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            goal_node2[1] = goal_node[1];
          }
        }
      }
    } else {
      i152 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i152 > 255U) {
        i152 = 255;
      }

      qY = goal_node[0] - 1U;
      if (qY > goal_node[0]) {
        qY = 0U;
      }

      if (goal_judge(maze_goal, (unsigned char)i152, (unsigned char)qY) != 0.0)
      {
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        i152 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        goal_section[1] = (unsigned char)i152;

        /* (y,x) */
        goal_node2[0] = (unsigned char)qY;
        goal_node2[1] = (unsigned char)i152;
        *goal_matrix_dir2 = matrix_dir.Col;
      } else {
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        if (goal_judge(maze_goal, goal_node[1], (unsigned char)qY) != 0.0) {
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_section[0] = (unsigned char)qY;
          goal_section[1] = goal_node[1];

          /* (y,x) */
          goal_node2[0] = goal_node[0];
          goal_node2[1] = goal_node[1];
        } else {
          i152 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          if (goal_judge(maze_goal, (unsigned char)i152, goal_node[0]) != 0.0) {
            goal_section[0] = goal_node[0];
            i152 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            goal_section[1] = (unsigned char)i152;

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            i152 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            goal_node2[1] = (unsigned char)i152;
            *goal_matrix_dir2 = matrix_dir.Col;
          } else {
            goal_section[0] = goal_node[0];
            goal_section[1] = goal_node[1];

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            goal_node2[1] = goal_node[1];
            *goal_matrix_dir2 = matrix_dir.Col;
          }
        }
      }
    }
  } else if (goal_dir == g_d_direction.South) {
    if (goal_matrix_dir == matrix_dir.Row) {
      qY = goal_node[0] - 2U;
      if (qY > goal_node[0]) {
        qY = 0U;
      }

      if (goal_judge(maze_goal, goal_node[1], (unsigned char)qY) != 0.0) {
        qY = goal_node[0] - 2U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        goal_section[1] = goal_node[1];

        /* (y,x) */
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_node2[0] = (unsigned char)qY;
        goal_node2[1] = goal_node[1];
      } else {
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        goal_section[1] = goal_node[1];

        /* (y,x) */
        goal_node2[0] = goal_node[0];
        goal_node2[1] = goal_node[1];
      }
    }
  } else if (goal_dir == g_d_direction.South_West) {
    if (goal_matrix_dir == matrix_dir.Row) {
      qY = goal_node[1] - 1U;
      if (qY > goal_node[1]) {
        qY = 0U;
      }

      b_qY = goal_node[0] - 2U;
      if (b_qY > goal_node[0]) {
        b_qY = 0U;
      }

      if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)b_qY) != 0.0)
      {
        qY = goal_node[0] - 2U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        goal_section[1] = (unsigned char)qY;

        /* (y,x) */
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_node2[0] = (unsigned char)qY;
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        goal_node2[1] = (unsigned char)qY;
      } else {
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        b_qY = goal_node[0] - 1U;
        if (b_qY > goal_node[0]) {
          b_qY = 0U;
        }

        if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)b_qY) != 0.0)
        {
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_section[0] = (unsigned char)qY;
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_section[1] = (unsigned char)qY;

          /* (y,x) */
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_node2[0] = (unsigned char)qY;
          goal_node2[1] = goal_node[1];
          *goal_matrix_dir2 = matrix_dir.Col;
        } else {
          qY = goal_node[0] - 2U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          if (goal_judge(maze_goal, goal_node[1], (unsigned char)qY) != 0.0) {
            qY = goal_node[0] - 2U;
            if (qY > goal_node[0]) {
              qY = 0U;
            }

            goal_section[0] = (unsigned char)qY;
            goal_section[1] = goal_node[1];

            /* (y,x) */
            qY = goal_node[0] - 1U;
            if (qY > goal_node[0]) {
              qY = 0U;
            }

            goal_node2[0] = (unsigned char)qY;
            goal_node2[1] = goal_node[1];
          } else {
            qY = goal_node[0] - 1U;
            if (qY > goal_node[0]) {
              qY = 0U;
            }

            goal_section[0] = (unsigned char)qY;
            goal_section[1] = goal_node[1];

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            goal_node2[1] = goal_node[1];
          }
        }
      }
    } else {
      qY = goal_node[1] - 1U;
      if (qY > goal_node[1]) {
        qY = 0U;
      }

      b_qY = goal_node[0] - 2U;
      if (b_qY > goal_node[0]) {
        b_qY = 0U;
      }

      if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)b_qY) != 0.0)
      {
        qY = goal_node[0] - 2U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        goal_section[1] = (unsigned char)qY;

        /* (y,x) */
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_node2[0] = (unsigned char)qY;
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        goal_node2[1] = (unsigned char)qY;
        *goal_matrix_dir2 = matrix_dir.Col;
      } else {
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        b_qY = goal_node[0] - 1U;
        if (b_qY > goal_node[0]) {
          b_qY = 0U;
        }

        if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)b_qY) != 0.0)
        {
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_section[0] = (unsigned char)qY;
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_section[1] = (unsigned char)qY;

          /* (y,x) */
          goal_node2[0] = goal_node[0];
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_node2[1] = (unsigned char)qY;
        } else {
          qY = goal_node[1] - 2U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          if (goal_judge(maze_goal, (unsigned char)qY, goal_node[0]) != 0.0) {
            goal_section[0] = goal_node[0];
            qY = goal_node[1] - 2U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            goal_section[1] = (unsigned char)qY;

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            goal_node2[1] = (unsigned char)qY;
            *goal_matrix_dir2 = matrix_dir.Col;
          } else {
            goal_section[0] = goal_node[0];
            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            goal_section[1] = (unsigned char)qY;

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            goal_node2[1] = goal_node[1];
            *goal_matrix_dir2 = matrix_dir.Col;
          }
        }
      }
    }
  } else if (goal_dir == g_d_direction.West) {
    /* 行方向は柱なのでなし */
    if (goal_matrix_dir == matrix_dir.Col) {
      qY = goal_node[1] - 2U;
      if (qY > goal_node[1]) {
        qY = 0U;
      }

      if (goal_judge(maze_goal, (unsigned char)qY, goal_node[0]) != 0.0) {
        goal_section[0] = goal_node[0];
        qY = goal_node[1] - 2U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        goal_section[1] = (unsigned char)qY;

        /* (y,x) */
        goal_node2[0] = goal_node[0];
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        goal_node2[1] = (unsigned char)qY;
        *goal_matrix_dir2 = matrix_dir.Col;
      } else {
        goal_section[0] = goal_node[0];
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        goal_section[1] = (unsigned char)qY;

        /* (y,x) */
        goal_node2[0] = goal_node[0];
        goal_node2[1] = goal_node[1];
        *goal_matrix_dir2 = matrix_dir.Col;
      }
    }
  } else {
    if (goal_dir == g_d_direction.North_West) {
      if (goal_matrix_dir == matrix_dir.Row) {
        qY = goal_node[1] - 1U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        i152 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)i152) != 0.0)
        {
          i152 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_section[0] = (unsigned char)i152;
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_section[1] = (unsigned char)qY;

          /* (y,x) */
          goal_node2[0] = (unsigned char)i152;
          goal_node2[1] = (unsigned char)qY;
        } else {
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          if (goal_judge(maze_goal, (unsigned char)qY, goal_node[0]) != 0.0) {
            goal_section[0] = goal_node[0];
            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            goal_section[1] = (unsigned char)qY;

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            goal_node2[1] = goal_node[1];
            *goal_matrix_dir2 = matrix_dir.Col;
          } else {
            i152 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            if (goal_judge(maze_goal, goal_node[1], (unsigned char)i152) != 0.0)
            {
              i152 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i152 > 255U) {
                i152 = 255;
              }

              goal_section[0] = (unsigned char)i152;
              goal_section[1] = goal_node[1];

              /* (y,x) */
              i152 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i152 > 255U) {
                i152 = 255;
              }

              goal_node2[0] = (unsigned char)i152;
              goal_node2[1] = goal_node[1];
            } else {
              goal_section[0] = goal_node[0];
              goal_section[1] = goal_node[1];

              /* (y,x) */
              goal_node2[0] = goal_node[0];
              goal_node2[1] = goal_node[1];
            }
          }
        }
      } else {
        qY = goal_node[1] - 2U;
        if (qY > goal_node[1]) {
          qY = 0U;
        }

        i152 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i152 > 255U) {
          i152 = 255;
        }

        if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)i152) != 0.0)
        {
          i152 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_section[0] = (unsigned char)i152;
          qY = goal_node[1] - 2U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_section[1] = (unsigned char)qY;

          /* (y,x) */
          i152 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          goal_node2[0] = (unsigned char)i152;
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_node2[1] = (unsigned char)qY;
          *goal_matrix_dir2 = matrix_dir.Col;
        } else {
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          i152 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i152 > 255U) {
            i152 = 255;
          }

          if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)i152) !=
              0.0) {
            i152 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            goal_section[0] = (unsigned char)i152;
            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            goal_section[1] = (unsigned char)qY;

            /* (y,x) */
            i152 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i152 > 255U) {
              i152 = 255;
            }

            goal_node2[0] = (unsigned char)i152;
            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            goal_node2[1] = (unsigned char)qY;
          } else {
            qY = goal_node[1] - 2U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            /* 入力座標の配列を作成 */
            /* ゴール判定関数 */
            /* ゴール座標と比較 */
            for (i152 = 0; i152 < 9; i152++) {
              uv0[i152] = (unsigned char)qY;
              uv0[9 + i152] = goal_node[0];
            }

            for (i152 = 0; i152 < 18; i152++) {
              temp1[i152] = (maze_goal[i152] == uv0[i152]);
            }

            /* x,yともに一致するか確認、一致なら1を返す */
            for (i152 = 0; i152 < 9; i152++) {
              varargin_1[i152] = (signed char)(temp1[i152] * temp1[9 + i152]);
            }

            ex = varargin_1[0];
            for (k = 0; k < 8; k++) {
              i152 = varargin_1[k + 1];
              if (ex < i152) {
                ex = i152;
              }
            }

            if (ex != 0) {
              goal_section[0] = goal_node[0];
              qY = goal_node[1] - 2U;
              if (qY > goal_node[1]) {
                qY = 0U;
              }

              goal_section[1] = (unsigned char)qY;

              /* (y,x) */
              goal_node2[0] = goal_node[0];
              qY = goal_node[1] - 1U;
              if (qY > goal_node[1]) {
                qY = 0U;
              }

              goal_node2[1] = (unsigned char)qY;
              *goal_matrix_dir2 = matrix_dir.Col;
            } else {
              goal_section[0] = goal_node[0];
              qY = goal_node[1] - 1U;
              if (qY > goal_node[1]) {
                qY = 0U;
              }

              goal_section[1] = (unsigned char)qY;

              /* (y,x) */
              goal_node2[0] = goal_node[0];
              goal_node2[1] = goal_node[1];
              *goal_matrix_dir2 = matrix_dir.Col;
            }
          }
        }
      }
    }
  }
}

/*
 * 入力　壁情報,壁探索情報,等高線MAP,ゴール座標,最大経路長
 * 出力   最短経路上の未探索マスの座標、未探索マスの数
 * Arguments    : const coder_internal_ref_1 *maze_wall
 *                const coder_internal_ref *goal_size
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_6 *wall
 *                const unsigned char b_maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                const unsigned short contour_map[1024]
 *                const unsigned char maze_goal[18]
 *                unsigned short max_length
 *                unsigned char unexp_square[1024]
 *                unsigned char *unexp_square_idx
 * Return Type  : void
 */
static void fust_run(const coder_internal_ref_1 *maze_wall, const
                     coder_internal_ref *goal_size, coder_internal_ref *wall_flg,
                     const coder_internal_ref_6 *wall, const unsigned char
                     b_maze_wall[1024], const unsigned char maze_wall_search
                     [1024], const unsigned short contour_map[1024], const
                     unsigned char maze_goal[18], unsigned short max_length,
                     unsigned char unexp_square[1024], unsigned char
                     *unexp_square_idx)
{
  unsigned char goal_flag;
  unsigned short little;
  unsigned char temp_x;
  unsigned char temp_y;
  unsigned char temp_dir;
  unsigned char next_dir;
  int tempk;
  bool exitg1;
  int i57;
  int i58;
  int i59;
  int b_unexp_square_idx;
  int i60;
  int i61;
  unsigned short u3;
  int i62;
  int i63;
  unsigned int qY;
  unsigned char switch_expression;

  /*     %% fust_run 最短経路走行 */
  /* 最短経路表示用ax */
  /*          global sh_route_ax */
  /* local変数宣言 */
  goal_flag = 0U;

  /* ゴール判定フラグ */
  little = max_length;

  /* 進行方向選定用閾値 */
  memset(&unexp_square[0], 0, sizeof(unsigned char) << 10);
  *unexp_square_idx = 0U;

  /*          %マウス位置表示用オブジェクト */
  /*          if coder.target('MATLAB') */
  /*              ax = gca; */
  /*              h = hgtransform('Parent',ax); */
  /*          end */
  /* マウスの初期位置設定 */
  temp_x = 1U;
  temp_y = 1U;

  /* マウスの初期方向定義 */
  temp_dir = g_direction.North;
  next_dir = g_direction.North;

  /* 探索開始時x */
  /* 探索開始時y */
  /* 実行時、既存の最短ルート表示を削除する(MATLABのみ) */
  tempk = 0;
  exitg1 = false;
  while ((!exitg1) && (tempk <= max_length - 1)) {
    /* 非走行モードのとき、非探索マスを記録する */
    /* 現在位置が未探索マスか判定 */
    i57 = temp_y + ((temp_x - 1) << 5);
    i58 = i57 - 1;
    if (maze_wall_search[i58] != 15) {
      /* 未探索マスであれば、記録する。インデックスを増加させる。 */
      i59 = (int)(*unexp_square_idx + 1U);
      b_unexp_square_idx = i59;
      if ((unsigned int)i59 > 255U) {
        b_unexp_square_idx = 255;
      }

      b_unexp_square_idx--;
      unexp_square[b_unexp_square_idx] = temp_y;
      unexp_square[512 + b_unexp_square_idx] = temp_x;
      if ((unsigned int)i59 > 255U) {
        i59 = 255;
      }

      *unexp_square_idx = (unsigned char)i59;
    }

    /* 現在位置がゴールか判定 */
    i59 = goal_size->contents;
    for (b_unexp_square_idx = 0; b_unexp_square_idx < i59; b_unexp_square_idx++)
    {
      if ((temp_x == maze_goal[b_unexp_square_idx]) && (temp_y ==
           maze_goal[b_unexp_square_idx + 9])) {
        goal_flag = 1U;
      }
    }

    if (goal_flag == 1) {
      /* 参照マス、参照方向を更新 */
      /* ゴールかつ走行時、停止処理を実施 */
      exitg1 = true;
    } else {
      /*         %%進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      /* 北側の壁のありなし */
      if (g_direction.North <= 7) {
        i60 = (unsigned char)(1 << g_direction.North);
      } else {
        i60 = 0;
      }

      if (((b_maze_wall[i58] & i60) == wall->contents.nowall) &&
          (contour_map[i57] < little)) {
        /* 北側の等高線mapが閾値より低ければ、 */
        /* 閾値を北側の等高map値に変更 */
        little = contour_map[temp_y + ((temp_x - 1) << 5)];

        /* 北側を進行方向に変更y */
        next_dir = g_direction.North;
      }

      /* 東側 */
      if (g_direction.East <= 7) {
        i61 = (unsigned char)(1 << g_direction.East);
      } else {
        i61 = 0;
      }

      if ((b_maze_wall[(temp_y + ((temp_x - 1) << 5)) - 1] & i61) ==
          wall->contents.nowall) {
        u3 = contour_map[(temp_y + (temp_x << 5)) - 1];
        if (u3 < little) {
          little = u3;
          next_dir = g_direction.East;
        }
      }

      /* 南側 */
      if (g_direction.South <= 7) {
        i62 = (unsigned char)(1 << g_direction.South);
      } else {
        i62 = 0;
      }

      if ((b_maze_wall[(temp_y + ((temp_x - 1) << 5)) - 1] & i62) ==
          wall->contents.nowall) {
        u3 = contour_map[i57 - 2];
        if (u3 < little) {
          little = u3;
          next_dir = g_direction.South;
        }
      }

      /* 西側 */
      if (g_direction.West <= 7) {
        i63 = (unsigned char)(1 << g_direction.West);
      } else {
        i63 = 0;
      }

      if ((b_maze_wall[(temp_y + ((temp_x - 1) << 5)) - 1] & i63) ==
          wall->contents.nowall) {
        u3 = contour_map[(temp_y + ((temp_x - 2) << 5)) - 1];
        if (u3 < little) {
          little = u3;
          next_dir = g_direction.West;
        }
      }

      /*          %走行時、探索壁情報に応じて、壁フラグをセット */
      /*          if (Sh_r_mode) */
      /*              %前 */
      /*              if bitand(maze_wall(temp_y,temp_x),rem(bitshift(uint8(1),temp_dir),15)) */
      /*                  wall_flg = bitor(wall_flg,1,'uint8'); */
      /*              end */
      /*              %右 */
      /*              if bitand(maze_wall(temp_y,temp_x),rem(bitshift(uint8(1),temp_dir+1),15)) */
      /*                  wall_flg = bitor(wall_flg,2,'uint8'); */
      /*              end */
      /*              %左 */
      /*              if bitand(maze_wall(temp_y,temp_x),rem(bitshift(uint8(1),temp_dir+3),15)) */
      /*                  wall_flg = bitor(wall_flg,8,'uint8'); */
      /*              end */
      /*          end */
      /*         %%現在方向と進行方向に応じた処理 */
      b_unexp_square_idx = (int)(4U + next_dir);
      if ((unsigned int)b_unexp_square_idx > 255U) {
        b_unexp_square_idx = 255;
      }

      qY = (unsigned int)b_unexp_square_idx - temp_dir;
      if (qY > (unsigned int)b_unexp_square_idx) {
        qY = 0U;
      }

      switch_expression = (unsigned char)((int)qY % 4);
      if (l_direction.front == switch_expression) {
        b_unexp_square_idx = 0;
      } else if (l_direction.right == switch_expression) {
        b_unexp_square_idx = 1;
      } else if (l_direction.back == switch_expression) {
        b_unexp_square_idx = 2;
      } else if (l_direction.left == switch_expression) {
        b_unexp_square_idx = 3;
      } else {
        b_unexp_square_idx = -1;
      }

      switch (b_unexp_square_idx) {
       case 0:
        /* 直進の場合、直進カウンタをインクリメント */
        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }

        /* 参照マスを移動 */
        /*                  %disp("front") */
        /*                  if (Sh_r_mode) %走行モード時、Cの動作関数を呼び出し */
        /*                      if ~coder.target('MATLAB') */
        /*                          coder.ceval('m_move_front',start_flg,wall_flg,uint8(move_dir_property.straight)); */
        /*                      end */
        /*                      %スタート直後フラグと壁フラグをクリア */
        /*                      start_flg = uint8(0); */
        /*                      wall_flg = uint8(0); */
        /*                  end */
        break;

       case 1:
        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 参照マス、参照方向を更新 */
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_clk_90deg 時計周りに90度ターンする関数 */
        i57 = (int)(4U + temp_dir);
        if ((unsigned int)i57 > 255U) {
          i57 = 255;
        }

        i57++;
        if ((unsigned int)i57 > 255U) {
          i57 = 255;
        }

        temp_dir = (unsigned char)(i57 % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }
        break;

       case 2:
        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 参照マス、参照方向を更新 */
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_180deg 180度ターンする関数 */
        i57 = (int)(4U + temp_dir);
        if ((unsigned int)i57 > 255U) {
          i57 = 255;
        }

        temp_dir = (unsigned char)((i57 - 2) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }
        break;

       case 3:
        wall_flg->contents = fust_run_wallset(maze_wall, temp_y, temp_x,
          temp_dir);

        /* 現在参照マスの壁情報を記憶 */
        /* 参照マス、参照方向を更新 */
        /* 入力　現在方向 */
        /* 出力　現在方向 */
        /*     %% turn_conclk_90deg 反時計周りに90度回る関数 */
        i57 = (int)(4U + temp_dir);
        if ((unsigned int)i57 > 255U) {
          i57 = 255;
        }

        temp_dir = (unsigned char)((i57 - 1) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step １マスだけ前進する関数 */
        /* 北に一マス */
        if (temp_dir == g_direction.North) {
          temp_y++;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (temp_dir == g_direction.East) {
          temp_x++;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (temp_dir == g_direction.South) {
          temp_y--;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (temp_dir == g_direction.West) {
          temp_x--;

          /* disp("west_step") */
        }

        /* disp("left") */
        break;
      }

      /* for code generation */
      tempk++;
    }
  }
}

/*
 * Arguments    : const coder_internal_ref_1 *maze_wall
 *                unsigned char temp_y
 *                unsigned char temp_x
 *                unsigned char temp_dir
 * Return Type  : unsigned char
 */
static unsigned char fust_run_wallset(const coder_internal_ref_1 *maze_wall,
  unsigned char temp_y, unsigned char temp_x, unsigned char temp_dir)
{
  unsigned char wall_flg;
  int i17;
  int b_temp_dir;
  int i18;
  int i19;
  int i20;

  /*     %% 既知壁の壁情報取得関数 */
  wall_flg = 0U;

  /* 前 */
  i17 = maze_wall->contents[(temp_y + ((temp_x - 1) << 5)) - 1];
  if (temp_dir <= 7) {
    b_temp_dir = (unsigned char)(1 << temp_dir);
  } else {
    b_temp_dir = 0;
  }

  if ((i17 & (b_temp_dir % 15)) != 0) {
    wall_flg = 1U;
  }

  /* 右 */
  i18 = (int)(temp_dir + 1U);
  if ((unsigned int)i18 > 255U) {
    i18 = 255;
  }

  if ((unsigned char)i18 <= 7) {
    i19 = (unsigned char)(1 << (unsigned char)i18);
  } else {
    i19 = 0;
  }

  if ((i17 & (i19 % 15)) != 0) {
    wall_flg = (unsigned char)(wall_flg | 2);
  }

  /* 左 */
  i18 = (int)(temp_dir + 3U);
  if ((unsigned int)i18 > 255U) {
    i18 = 255;
  }

  if ((unsigned char)i18 <= 7) {
    i20 = (unsigned char)(1 << (unsigned char)i18);
  } else {
    i20 = 0;
  }

  if ((i17 & (i20 % 15)) != 0) {
    wall_flg = (unsigned char)(wall_flg | 8);
  }

  return wall_flg;
}

/*
 * 出力変数定義
 * Arguments    : const unsigned short row_num_node[1056]
 *                const unsigned short col_num_node[1056]
 *                unsigned char current_move_dir
 *                const unsigned char current_node[2]
 *                unsigned char current_matrix_dir
 *                const unsigned char goal_node2[2]
 *                unsigned char goal_matrix_dir2
 *                const unsigned char goal_section[2]
 *                unsigned char *next_dir
 *                unsigned char next_node[2]
 *                unsigned char *next_node_property
 * Return Type  : void
 */
static void get_next_dir_diagonal(const unsigned short row_num_node[1056], const
  unsigned short col_num_node[1056], unsigned char current_move_dir, const
  unsigned char current_node[2], unsigned char current_matrix_dir, const
  unsigned char goal_node2[2], unsigned char goal_matrix_dir2, const unsigned
  char goal_section[2], unsigned char *next_dir, unsigned char next_node[2],
  unsigned char *next_node_property)
{
  bool p;
  bool b_p;
  int k;
  bool exitg1;
  unsigned short map_min;
  unsigned int qY;
  int i224;
  unsigned char temp;
  int i225;
  unsigned short u10;

  /*     %% get_next_dir_diagonal 斜め有での進行方向,行先取得 */
  *next_dir = g_d_direction.North;
  next_node[0] = 1U;
  next_node[1] = 1U;
  *next_node_property = matrix_dir.Row;

  /* 現在のエッジはゴールノードか */
  p = false;
  b_p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (current_node[k] != goal_node2[k]) {
      b_p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = true;
  }

  if (p && (current_matrix_dir == goal_matrix_dir2)) {
    /* ゴールノードの場合 */
    /* ゴールノードが行方向の場合 */
    if (goal_matrix_dir2 == matrix_dir.Row) {
      if ((goal_section[1] == goal_node2[0]) && (goal_section[0] == goal_node2[1]))
      {
        next_node[0] = goal_node2[0];
        next_node[1] = goal_node2[1];
        *next_node_property = matrix_dir.section;
      } else {
        *next_dir = g_d_direction.South;
        qY = goal_node2[0] - 1U;
        if (qY > goal_node2[0]) {
          qY = 0U;
        }

        next_node[0] = (unsigned char)qY;
        next_node[1] = goal_node2[1];
        *next_node_property = matrix_dir.section;
      }
    } else {
      /* ゴールノードが列方向の場合 */
      if ((goal_section[1] == goal_node2[0]) && (goal_section[0] == goal_node2[1]))
      {
        *next_dir = g_d_direction.East;
        next_node[0] = goal_node2[0];
        next_node[1] = goal_node2[1];
        *next_node_property = matrix_dir.section;
      } else {
        *next_dir = g_d_direction.West;
        next_node[0] = goal_node2[0];
        qY = goal_node2[1] - 1U;
        if (qY > goal_node2[1]) {
          qY = 0U;
        }

        next_node[1] = (unsigned char)qY;
        *next_node_property = matrix_dir.section;
      }
    }
  } else {
    /* ゴールノードでない場合 */
    /* 閾値を定義 */
    map_min = MAX_uint16_T;

    /* 現在のノードの方向から優先的に進行方向を確定 */
    for (k = 0; k < 8; k++) {
      i224 = current_move_dir + k;
      if (i224 > 255) {
        i224 = 255;
      }

      temp = (unsigned char)(i224 % 8);

      /* 現在のノードが行方向の時 */
      if (current_matrix_dir == matrix_dir.Row) {
        if (temp == g_d_direction.North) {
          i224 = (int)(current_node[0] + 1U);
          i225 = i224;
          if ((unsigned int)i224 > 255U) {
            i225 = 255;
          }

          if (row_num_node[(i225 + 33 * (current_node[1] - 1)) - 1] < map_min) {
            /* 最小値を更新 */
            i225 = i224;
            if ((unsigned int)i224 > 255U) {
              i225 = 255;
            }

            map_min = row_num_node[(i225 + 33 * (current_node[1] - 1)) - 1];

            /* 現在ノードの進行方向を北向きに */
            *next_dir = g_d_direction.North;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Row;
            if ((unsigned int)i224 > 255U) {
              i224 = 255;
            }

            next_node[0] = (unsigned char)i224;
            next_node[1] = current_node[1];
          }
        } else if (temp == g_d_direction.North_East) {
          i224 = (int)(current_node[1] + 1U);
          i225 = i224;
          if ((unsigned int)i224 > 255U) {
            i225 = 255;
          }

          if (col_num_node[(current_node[0] + ((i225 - 1) << 5)) - 1] < map_min)
          {
            /* 最小値を更新 */
            i225 = i224;
            if ((unsigned int)i224 > 255U) {
              i225 = 255;
            }

            map_min = col_num_node[(current_node[0] + ((i225 - 1) << 5)) - 1];

            /* 現在ノードの進行方向を東北向きに */
            *next_dir = g_d_direction.North_East;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Col;
            next_node[0] = current_node[0];
            if ((unsigned int)i224 > 255U) {
              i224 = 255;
            }

            next_node[1] = (unsigned char)i224;
          }
        } else if (temp != g_d_direction.East) {
          if (temp == g_d_direction.South_East) {
            qY = current_node[0] - 1U;
            if (qY > current_node[0]) {
              qY = 0U;
            }

            i224 = (int)(current_node[1] + 1U);
            if ((unsigned int)i224 > 255U) {
              i224 = 255;
            }

            if (col_num_node[((int)qY + ((i224 - 1) << 5)) - 1] < map_min) {
              /* 最小値を更新 */
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              i224 = (int)(current_node[1] + 1U);
              if ((unsigned int)i224 > 255U) {
                i224 = 255;
              }

              map_min = col_num_node[((int)qY + ((i224 - 1) << 5)) - 1];

              /* 現在ノードの進行方向を南東向きに */
              *next_dir = g_d_direction.South_East;

              /* 進行方向先の座標、行列の方向を更新 */
              *next_node_property = matrix_dir.Col;
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              next_node[0] = (unsigned char)qY;
              i224 = (int)(current_node[1] + 1U);
              if ((unsigned int)i224 > 255U) {
                i224 = 255;
              }

              next_node[1] = (unsigned char)i224;
            }
          } else if (temp == g_d_direction.South) {
            qY = current_node[0] - 1U;
            if (qY > current_node[0]) {
              qY = 0U;
            }

            if (row_num_node[((int)qY + 33 * (current_node[1] - 1)) - 1] <
                map_min) {
              /* 最小値を更新 */
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              map_min = row_num_node[((int)qY + 33 * (current_node[1] - 1)) - 1];

              /* 現在ノードの進行方向を南向きに */
              *next_dir = g_d_direction.South;

              /* 進行方向先の座標、行列の方向を更新 */
              *next_node_property = matrix_dir.Row;
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              next_node[0] = (unsigned char)qY;
              next_node[1] = current_node[1];
            }
          } else if (temp == g_d_direction.South_West) {
            qY = current_node[0] - 1U;
            if (qY > current_node[0]) {
              qY = 0U;
            }

            if (col_num_node[((int)qY + ((current_node[1] - 1) << 5)) - 1] <
                map_min) {
              /* 最小値を更新 */
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              map_min = col_num_node[((int)qY + ((current_node[1] - 1) << 5)) -
                1];

              /* 現在ノードの進行方向を南西向きに */
              *next_dir = g_d_direction.South_West;

              /* 進行方向先の座標、行列の方向を更新 */
              *next_node_property = matrix_dir.Col;
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              next_node[0] = (unsigned char)qY;
              next_node[1] = current_node[1];
            }
          } else if ((temp != g_d_direction.West) && (temp ==
                      g_d_direction.North_West)) {
            u10 = col_num_node[(current_node[0] + ((current_node[1] - 1) << 5))
              - 1];
            if (u10 < map_min) {
              /* 最小値を更新 */
              map_min = u10;

              /* 現在ノードの進行方向を北西向きに */
              *next_dir = g_d_direction.North_West;

              /* 進行方向先の座標、行列の方向を更新 */
              *next_node_property = matrix_dir.Col;
              next_node[0] = current_node[0];
              next_node[1] = current_node[1];
            }
          } else {
            /* 柱 */
          }
        } else {
          /* 柱 */
        }

        /* 現在のノードが行方向の時 */
      } else if (temp != g_d_direction.North) {
        if (temp == g_d_direction.North_East) {
          i224 = (int)(current_node[0] + 1U);
          if ((unsigned int)i224 > 255U) {
            i224 = 255;
          }

          if (row_num_node[(i224 + 33 * (current_node[1] - 1)) - 1] < map_min) {
            /* 最小値を更新 */
            i224 = (int)(current_node[0] + 1U);
            if ((unsigned int)i224 > 255U) {
              i224 = 255;
            }

            map_min = row_num_node[(i224 + 33 * (current_node[1] - 1)) - 1];

            /* 現在ノードの進行方向を北東向きに */
            *next_dir = g_d_direction.North_East;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Row;
            i224 = (int)(current_node[0] + 1U);
            if ((unsigned int)i224 > 255U) {
              i224 = 255;
            }

            next_node[0] = (unsigned char)i224;
            next_node[1] = current_node[1];
          }
        } else if (temp == g_d_direction.East) {
          i224 = (int)(current_node[1] + 1U);
          if ((unsigned int)i224 > 255U) {
            i224 = 255;
          }

          if (col_num_node[(current_node[0] + ((i224 - 1) << 5)) - 1] < map_min)
          {
            /* 最小値を更新 */
            i224 = (int)(current_node[1] + 1U);
            if ((unsigned int)i224 > 255U) {
              i224 = 255;
            }

            map_min = col_num_node[(current_node[0] + ((i224 - 1) << 5)) - 1];

            /* 現在ノードの進行方向を東向きに */
            *next_dir = g_d_direction.East;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Col;
            next_node[0] = current_node[0];
            i224 = (int)(current_node[1] + 1U);
            if ((unsigned int)i224 > 255U) {
              i224 = 255;
            }

            next_node[1] = (unsigned char)i224;
          }
        } else if (temp == g_d_direction.South_East) {
          u10 = row_num_node[(current_node[0] + 33 * (current_node[1] - 1)) - 1];
          if (u10 < map_min) {
            /* 最小値を更新 */
            map_min = u10;

            /* 現在ノードの進行方向を南東向きに */
            *next_dir = g_d_direction.South_East;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Row;
            next_node[0] = current_node[0];
            next_node[1] = current_node[1];
          }
        } else if (temp != g_d_direction.South) {
          if (temp == g_d_direction.South_West) {
            qY = current_node[1] - 1U;
            if (qY > current_node[1]) {
              qY = 0U;
            }

            if ((int)qY > 0) {
              qY = current_node[1] - 1U;
              if (qY > current_node[1]) {
                qY = 0U;
              }

              if (row_num_node[(current_node[0] + 33 * ((int)qY - 1)) - 1] <
                  map_min) {
                /* 最小値を更新 */
                qY = current_node[1] - 1U;
                if (qY > current_node[1]) {
                  qY = 0U;
                }

                map_min = row_num_node[(current_node[0] + 33 * ((int)qY - 1)) -
                  1];

                /* 現在ノードの進行方向を南西向きに */
                *next_dir = g_d_direction.South_West;

                /* 進行方向先の座標、行列の方向を更新 */
                *next_node_property = matrix_dir.Row;
                next_node[0] = current_node[0];
                qY = current_node[1] - 1U;
                if (qY > current_node[1]) {
                  qY = 0U;
                }

                next_node[1] = (unsigned char)qY;
              }
            }
          } else if (temp == g_d_direction.West) {
            qY = current_node[1] - 1U;
            if (qY > current_node[1]) {
              qY = 0U;
            }

            if ((int)qY > 0) {
              qY = current_node[1] - 1U;
              if (qY > current_node[1]) {
                qY = 0U;
              }

              if (col_num_node[(current_node[0] + (((int)qY - 1) << 5)) - 1] <
                  map_min) {
                /* 最小値を更新 */
                qY = current_node[1] - 1U;
                if (qY > current_node[1]) {
                  qY = 0U;
                }

                map_min = col_num_node[(current_node[0] + (((int)qY - 1) << 5))
                  - 1];

                /* 現在ノードの進行方向を西向きに */
                *next_dir = g_d_direction.West;

                /* 進行方向先の座標、行列の方向を更新 */
                *next_node_property = matrix_dir.Col;
                next_node[0] = current_node[0];
                qY = current_node[1] - 1U;
                if (qY > current_node[1]) {
                  qY = 0U;
                }

                next_node[1] = (unsigned char)qY;
              }
            }
          } else {
            if (temp == g_d_direction.North_West) {
              qY = current_node[1] - 1U;
              if (qY > current_node[1]) {
                qY = 0U;
              }

              if ((int)qY > 0) {
                i224 = (int)(current_node[0] + 1U);
                if ((unsigned int)i224 > 255U) {
                  i224 = 255;
                }

                qY = current_node[1] - 1U;
                if (qY > current_node[1]) {
                  qY = 0U;
                }

                if (row_num_node[(i224 + 33 * ((int)qY - 1)) - 1] < map_min) {
                  /* 最小値を更新 */
                  i224 = (int)(current_node[0] + 1U);
                  if ((unsigned int)i224 > 255U) {
                    i224 = 255;
                  }

                  qY = current_node[1] - 1U;
                  if (qY > current_node[1]) {
                    qY = 0U;
                  }

                  map_min = row_num_node[(i224 + 33 * ((int)qY - 1)) - 1];

                  /* 現在ノードの進行方向を北西向きに */
                  *next_dir = g_d_direction.North_West;

                  /* 進行方向先の座標、行列の方向を更新 */
                  *next_node_property = matrix_dir.Row;
                  i224 = (int)(current_node[0] + 1U);
                  if ((unsigned int)i224 > 255U) {
                    i224 = 255;
                  }

                  next_node[0] = (unsigned char)i224;
                  qY = current_node[1] - 1U;
                  if (qY > current_node[1]) {
                    qY = 0U;
                  }

                  next_node[1] = (unsigned char)qY;
                }
              }
            }
          }
        } else {
          /* 柱 */
        }
      } else {
        /* 柱 */
      }
    }
  }
}

/*
 * 入力 現在地x,y,壁情報,等高線map,最大経路長
 *  出力 次の進行方角,エラーフラグ
 * Arguments    : unsigned char current_x
 *                unsigned char current_y
 *                const unsigned char maze_wall[1024]
 *                const unsigned short contour_map[1024]
 *                unsigned char *next_dir
 *                unsigned char *error_flg
 * Return Type  : void
 */
static void get_nextdir2(unsigned char current_x, unsigned char current_y, const
  unsigned char maze_wall[1024], const unsigned short contour_map[1024],
  unsigned char *next_dir, unsigned char *error_flg)
{
  unsigned short little;
  int i11;
  int i12;
  int i13;
  int i14;
  unsigned short u0;
  int i15;
  int i16;

  /*     %% get_nextdir2 等高線mapから次に向かう方向を選択 */
  /* エラーフラグセット(方向が定まる場合リセット) */
  *error_flg = 1U;

  /* 出力の初期化 */
  *next_dir = 0U;
  little = MAX_uint16_T;

  /*     %%進行方向選定 */
  /* 優先順位　北⇒東⇒南⇒西 */
  /* 北側の壁のありなし判定 */
  i11 = current_y + ((current_x - 1) << 5);
  i12 = maze_wall[i11 - 1];
  if (g_direction.North <= 7) {
    i13 = (unsigned char)(1 << g_direction.North);
  } else {
    i13 = 0;
  }

  if (((i12 & i13) == 0) && (contour_map[i11] < 65535)) {
    /* 北側の等高線mapが閾値より低ければ、 */
    /* 閾値を北側の等高map値に変更 */
    little = contour_map[current_y + ((current_x - 1) << 5)];

    /* 北側を進行方向に変更y */
    *next_dir = g_direction.North;
    *error_flg = 0U;
  }

  /* 東側 */
  if (g_direction.East <= 7) {
    i14 = (unsigned char)(1 << g_direction.East);
  } else {
    i14 = 0;
  }

  if ((i12 & i14) == 0) {
    u0 = contour_map[(current_y + (current_x << 5)) - 1];
    if (u0 < little) {
      little = u0;
      *next_dir = g_direction.East;
      *error_flg = 0U;
    }
  }

  /* 南側 */
  if (g_direction.South <= 7) {
    i15 = (unsigned char)(1 << g_direction.South);
  } else {
    i15 = 0;
  }

  if ((i12 & i15) == 0) {
    u0 = contour_map[i11 - 2];
    if (u0 < little) {
      little = u0;
      *next_dir = g_direction.South;
      *error_flg = 0U;
    }
  }

  /* 西側 */
  if (g_direction.West <= 7) {
    i16 = (unsigned char)(1 << g_direction.West);
  } else {
    i16 = 0;
  }

  if (((i12 & i16) == 0) && (contour_map[(current_y + ((current_x - 2) << 5)) -
       1] < little)) {
    /*  little = contour_map(current_y,current_x-1); */
    *next_dir = g_direction.West;
    *error_flg = 0U;
  }
}

/*
 * 旋回パターン番号の初期化
 * Arguments    : const double move_dir_buffer[3]
 *                unsigned char ref_move_mode
 * Return Type  : unsigned char
 */
static unsigned char get_turn_pattern_num(const double move_dir_buffer[3],
  unsigned char ref_move_mode)
{
  unsigned char turn_pattern_num;

  /* パターンを判定する関数 */
  turn_pattern_num = turn_pattern.b_default;

  /* 直進時 */
  if (ref_move_mode == move_dir_property.straight) {
    /* 次が45度まがるとき(右折パターン) */
    if (move_dir_buffer[0] == 1.0) {
      if (move_dir_buffer[1] == 1.0) {
        turn_pattern_num = turn_pattern.r_45;
      } else if (move_dir_buffer[1] == 2.0) {
        turn_pattern_num = turn_pattern.r_90;
      } else {
        if (move_dir_buffer[1] == 3.0) {
          if (move_dir_buffer[2] == 3.0) {
            turn_pattern_num = turn_pattern.r_135;
          } else {
            if (move_dir_buffer[2] == 4.0) {
              turn_pattern_num = turn_pattern.r_180;
            }
          }
        }
      }

      /* 次が-45度まがるとき(左折パターン) */
    } else {
      if (move_dir_buffer[0] == 7.0) {
        if (move_dir_buffer[1] == 7.0) {
          turn_pattern_num = turn_pattern.l_45;
        } else if (move_dir_buffer[1] == 6.0) {
          turn_pattern_num = turn_pattern.l_90;
        } else {
          if (move_dir_buffer[1] == 5.0) {
            if (move_dir_buffer[2] == 5.0) {
              turn_pattern_num = turn_pattern.l_135;
            } else {
              if (move_dir_buffer[2] == 4.0) {
                turn_pattern_num = turn_pattern.l_180;
              }
            }
          }
        }
      }
    }

    /* 斜めの時 */
  } else {
    if (ref_move_mode == move_dir_property.diagonal) {
      /* 右折パターン */
      if (move_dir_buffer[0] == 1.0) {
        turn_pattern_num = turn_pattern.r_45;
      } else if (move_dir_buffer[0] == 2.0) {
        if (move_dir_buffer[1] == 2.0) {
          turn_pattern_num = turn_pattern.r_90;
        } else {
          if (move_dir_buffer[1] == 3.0) {
            turn_pattern_num = turn_pattern.r_135;
          }
        }

        /* 左折パターン */
      } else if (move_dir_buffer[0] == 7.0) {
        turn_pattern_num = turn_pattern.l_45;
      } else {
        if (move_dir_buffer[0] == 6.0) {
          if (move_dir_buffer[1] == 6.0) {
            turn_pattern_num = turn_pattern.l_90;
          } else {
            if (move_dir_buffer[1] == 5.0) {
              turn_pattern_num = turn_pattern.l_135;
            }
          }
        }
      }
    }
  }

  return turn_pattern_num;
}

/*
 * 入力座標の配列を作成
 * Arguments    : const unsigned char maze_goal[18]
 *                unsigned char x
 *                unsigned char y
 * Return Type  : double
 */
static double goal_judge(const unsigned char maze_goal[18], unsigned char x,
  unsigned char y)
{
  int i153;
  unsigned char uv1[18];
  bool temp1[18];
  int ex;
  signed char varargin_1[9];
  int k;

  /* ゴール判定関数 */
  /* ゴール座標と比較 */
  for (i153 = 0; i153 < 9; i153++) {
    uv1[i153] = x;
    uv1[9 + i153] = y;
  }

  for (i153 = 0; i153 < 18; i153++) {
    temp1[i153] = (maze_goal[i153] == uv1[i153]);
  }

  /* x,yともに一致するか確認、一致なら1を返す */
  for (i153 = 0; i153 < 9; i153++) {
    varargin_1[i153] = (signed char)(temp1[i153] * temp1[9 + i153]);
  }

  ex = varargin_1[0];
  for (k = 0; k < 8; k++) {
    i153 = varargin_1[k + 1];
    if (ex < i153) {
      ex = i153;
    }
  }

  return ex;
}

/*
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : const coder_internal_ref_6 *wall
 *                const unsigned char maze_goal[18]
 *                unsigned char l_goal_size
 *                const unsigned char maze_wall[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void make_map_find(const coder_internal_ref_6 *wall, const unsigned char
  maze_goal[18], unsigned char l_goal_size, const unsigned char maze_wall[1024],
  unsigned char current_x, unsigned char current_y, unsigned short contour_map
  [1024])
{
  unsigned char contor_renew_square[2048];
  unsigned char contor_renew_square_temp[2048];
  unsigned char contor_renew_square_idx;
  unsigned char contor_renew_square_idx_temp;
  int i2;
  int temp;
  unsigned short tempi;
  bool exitg1;
  unsigned char change_flag;
  int i3;
  int i4;
  int i5;
  int i6;
  int i7;
  int i8;
  int i9;
  unsigned int qY;
  int i10;

  /*     %%  make_map_find 壁情報から等高線MAPを生成 */
  /*  迷路パラメータ設定 */
  /* コンター更新マス保管用 */
  /* 更新座標 */
  memset(&contor_renew_square[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_square_temp[0], 0, sizeof(unsigned char) << 11);

  /* 更新座標更新用 */
  contor_renew_square_idx = 1U;

  /* 更新座標 */
  contor_renew_square_idx_temp = 1U;

  /* 更新座標更新用 */
  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* 32マス分mapを保持 */
  /* 16bitにすべき */
  for (i2 = 0; i2 < 1024; i2++) {
    contour_map[i2] = MAX_uint16_T;
  }

  /* ゴール座標に0を入力 */
  i2 = l_goal_size;
  for (temp = 0; temp < i2; temp++) {
    contor_renew_square_idx = maze_goal[temp + 9];
    contour_map[(contor_renew_square_idx + ((maze_goal[temp] - 1) << 5)) - 1] =
      0U;

    /* 初回の更新座標 = ゴール座標　を入力 */
    contor_renew_square[temp] = contor_renew_square_idx;
    contor_renew_square[1024 + temp] = maze_goal[temp];
    contor_renew_square_idx = (unsigned char)(1 + temp);
  }

  tempi = 0U;
  exitg1 = false;
  while ((!exitg1) && (tempi < 65535)) {
    /* 歩数カウントは0~max_length */
    /* map更新確認用フラグ */
    change_flag = 0U;

    /* 更新された座標に対し、歩数mapを更新 */
    i2 = contor_renew_square_idx;
    for (temp = 0; temp < i2; temp++) {
      /* 北側 */
      /* if (bitand(maze_wall(row(tempn),col(tempn)),bitshift(uint8(1),g_direction.North)) == wall.nowall) */
      if (g_direction.North <= 7) {
        i3 = (unsigned char)(1 << g_direction.North);
      } else {
        i3 = 0;
      }

      if ((maze_wall[(contor_renew_square[temp] + ((contor_renew_square[temp +
              1024] - 1) << 5)) - 1] & i3) == wall->contents.nowall) {
        /* 北側のMAPが更新されているか判断、されていなければ書き込み */
        i4 = (int)(contor_renew_square[temp] + 1U);
        i5 = i4;
        if ((unsigned int)i4 > 255U) {
          i5 = 255;
        }

        if (contour_map[(i5 + ((contor_renew_square[temp + 1024] - 1) << 5)) - 1]
            == 65535) {
          i5 = i4;
          if ((unsigned int)i4 > 255U) {
            i5 = 255;
          }

          contour_map[(i5 + ((contor_renew_square[temp + 1024] - 1) << 5)) - 1] =
            (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          if ((unsigned int)i4 > 255U) {
            i4 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)i4;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square[temp + 1024];

          /* 更新マス用インデックスを増加 */
          i4 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i4 > 255U) {
            i4 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i4;
        }
      }

      /* 東側 */
      contor_renew_square_idx = contor_renew_square[temp + 1024];
      i4 = (contor_renew_square_idx - 1) << 5;
      i5 = maze_wall[(contor_renew_square[temp] + i4) - 1];
      if (g_direction.East <= 7) {
        i6 = (unsigned char)(1 << g_direction.East);
      } else {
        i6 = 0;
      }

      if ((i5 & i6) == wall->contents.nowall) {
        /* 東側のMAPが更新されているか判断、されていなければ書き込み */
        i7 = (int)(contor_renew_square[temp + 1024] + 1U);
        i8 = i7;
        if ((unsigned int)i7 > 255U) {
          i8 = 255;
        }

        if (contour_map[(contor_renew_square[temp] + ((i8 - 1) << 5)) - 1] ==
            65535) {
          i8 = i7;
          if ((unsigned int)i7 > 255U) {
            i8 = 255;
          }

          contour_map[(contor_renew_square[temp] + ((i8 - 1) << 5)) - 1] =
            (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
            contor_renew_square[temp];
          if ((unsigned int)i7 > 255U) {
            i7 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            (unsigned char)i7;

          /* 更新マス用インデックスを増加 */
          i7 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i7 > 255U) {
            i7 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i7;
        }
      }

      /* 南側 */
      if (g_direction.South <= 7) {
        i9 = (unsigned char)(1 << g_direction.South);
      } else {
        i9 = 0;
      }

      if ((i5 & i9) == wall->contents.nowall) {
        /* 南側のMAPが更新されているか判断、されていなければ書き込み */
        qY = contor_renew_square[temp] - 1U;
        if (qY > contor_renew_square[temp]) {
          qY = 0U;
        }

        if (contour_map[((int)qY + i4) - 1] == 65535) {
          qY = contor_renew_square[temp] - 1U;
          if (qY > contor_renew_square[temp]) {
            qY = 0U;
          }

          contour_map[((int)qY + i4) - 1] = (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          qY = contor_renew_square[temp] - 1U;
          if (qY > contor_renew_square[temp]) {
            qY = 0U;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)qY;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square_idx;

          /* 更新マス用インデックスを増加 */
          i4 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i4 > 255U) {
            i4 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i4;
        }
      }

      /* 西側 */
      if (g_direction.West <= 7) {
        i10 = (unsigned char)(1 << g_direction.West);
      } else {
        i10 = 0;
      }

      if ((i5 & i10) == wall->contents.nowall) {
        /* 西側のMAPが更新されているか判断、されていなければ書き込み */
        qY = contor_renew_square_idx - 1U;
        if (qY > contor_renew_square_idx) {
          qY = 0U;
        }

        if (contour_map[(contor_renew_square[temp] + (((int)qY - 1) << 5)) - 1] ==
            65535) {
          qY = contor_renew_square_idx - 1U;
          if (qY > contor_renew_square_idx) {
            qY = 0U;
          }

          contour_map[(contor_renew_square[temp] + (((int)qY - 1) << 5)) - 1] =
            (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
            contor_renew_square[temp];
          qY = contor_renew_square_idx - 1U;
          if (qY > contor_renew_square_idx) {
            qY = 0U;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            (unsigned char)qY;

          /* 更新マス用インデックスを増加 */
          i4 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i4 > 255U) {
            i4 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i4;
        }
      }
    }

    /* ゴール更新マスの更新とインデックスのクリア */
    for (i2 = 0; i2 < 2048; i2++) {
      contor_renew_square[i2] = contor_renew_square_temp[i2];
      contor_renew_square_temp[i2] = 0U;
    }

    contor_renew_square_idx = (unsigned char)(contor_renew_square_idx_temp - 1);
    contor_renew_square_idx_temp = 1U;

    /* 更新がない、もしくは現在位置が更新されていれば終了 */
    if ((change_flag == 0) || (contour_map[(current_y + ((current_x - 1) << 5))
         - 1] != 65535)) {
      exitg1 = true;
    } else {
      tempi++;
    }
  }
}

/*
 * 未知壁の領域は仮想壁をおいて侵入しない。
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数),迷路探索情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : const coder_internal_ref *goal_size
 *                const coder_internal_ref_6 *wall
 *                const coder_internal_ref_5 *search
 *                const unsigned char maze_goal[18]
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned char unknown_wall_flg
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void make_map_fustrun(const coder_internal_ref *goal_size, const
  coder_internal_ref_6 *wall, const coder_internal_ref_5 *search, const unsigned
  char maze_goal[18], const unsigned char maze_wall[1024], const unsigned char
  maze_wall_search[1024], unsigned char unknown_wall_flg, unsigned short
  contour_map[1024])
{
  unsigned char contor_renew_square[2048];
  unsigned char contor_renew_square_temp[2048];
  unsigned char contor_renew_square_idx;
  unsigned char contor_renew_square_idx_temp;
  int i37;
  unsigned char move_dir_map[1024];
  int tempn;
  unsigned short tempi;
  bool exitg1;
  int contour_map_tmp;
  int i38;
  unsigned char change_flag;
  int i39;
  int i40;
  int i41;
  int i42;
  int i43;
  int i44;
  int i45;
  int i46;
  int i47;
  int i48;
  int i49;
  int i50;
  int i51;
  int i52;
  unsigned int u2;
  unsigned int qY;
  int i53;
  int i54;
  int i55;
  int i56;

  /*     %% make_map_fustrun 最短走行用等高線MAPを生成 */
  /* コンター更新マス保管用 */
  /* 更新座標 */
  memset(&contor_renew_square[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_square_temp[0], 0, sizeof(unsigned char) << 11);

  /* 更新座標更新用 */
  contor_renew_square_idx = 1U;

  /* 更新座標 */
  contor_renew_square_idx_temp = 1U;

  /* 更新座標更新用 */
  /* ローカル変数設定 */
  /* パラメータ設定 */
  /*  迷路パラメータ設定 */
  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* 32マス分mapを保持 */
  /* 進行方向補完用変数定義 */
  for (i37 = 0; i37 < 1024; i37++) {
    contour_map[i37] = MAX_uint16_T;
    move_dir_map[i37] = 0U;
  }

  /* ゴール座標に */
  /*  歩数マップ：0を入力 */
  /*  進行方向 : 1+2+4+8(東西南北すべて)=15 */
  /*  を入力 */
  i37 = goal_size->contents;
  for (tempn = 0; tempn < i37; tempn++) {
    contor_renew_square_idx = maze_goal[tempn + 9];
    contour_map_tmp = (contor_renew_square_idx + ((maze_goal[tempn] - 1) << 5))
      - 1;
    contour_map[contour_map_tmp] = 0U;
    if (g_direction.North <= 7) {
      i38 = (unsigned char)(1 << g_direction.North);
    } else {
      i38 = 0;
    }

    if (g_direction.East <= 7) {
      i39 = (unsigned char)(1 << g_direction.East);
    } else {
      i39 = 0;
    }

    i40 = (int)((unsigned int)i38 + i39);
    if ((unsigned int)i40 > 255U) {
      i40 = 255;
    }

    if (g_direction.South <= 7) {
      i41 = (unsigned char)(1 << g_direction.South);
    } else {
      i41 = 0;
    }

    i40 = (int)((unsigned int)i40 + i41);
    if ((unsigned int)i40 > 255U) {
      i40 = 255;
    }

    if (g_direction.West <= 7) {
      i44 = (unsigned char)(1 << g_direction.West);
    } else {
      i44 = 0;
    }

    i40 = (int)((unsigned int)i40 + i44);
    if ((unsigned int)i40 > 255U) {
      i40 = 255;
    }

    move_dir_map[contour_map_tmp] = (unsigned char)i40;

    /* 初回の更新座標 = ゴール座標　を入力 */
    contor_renew_square[tempn] = contor_renew_square_idx;
    contor_renew_square[1024 + tempn] = maze_goal[tempn];
    contor_renew_square_idx = (unsigned char)(1 + tempn);
  }

  tempi = 0U;
  exitg1 = false;
  while ((!exitg1) && (tempi < 65535)) {
    /* 更新確認用の歩数カウントは0~max_length */
    change_flag = 0U;

    /* 更新フラグのクリア */
    /* 検索した座標に対し、歩数mapを更新 */
    i37 = contor_renew_square_idx;
    for (tempn = 0; tempn < i37; tempn++) {
      /* 北側 */
      /* 壁が無い & (探索済み || ~未知壁フラグ)であるとき */
      if (g_direction.North <= 7) {
        i42 = (unsigned char)(1 << g_direction.North);
      } else {
        i42 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i42) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i43 = (unsigned char)(1 << g_direction.North);
        } else {
          i43 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i43) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が北向きである時 */
          contor_renew_square_idx = contor_renew_square[tempn + 1024];
          i40 = (contor_renew_square_idx - 1) << 5;
          contour_map_tmp = (contor_renew_square[tempn] + i40) - 1;
          if (g_direction.North <= 7) {
            i47 = (unsigned char)(1 << g_direction.North);
          } else {
            i47 = 0;
          }

          if ((move_dir_map[contour_map_tmp] & i47) != 0) {
            /* かつ北のマスが更新予定値よりも大きな値の場合 */
            contour_map_tmp = (int)(contor_renew_square[tempn] + 1U);
            if ((unsigned int)contour_map_tmp > 255U) {
              contour_map_tmp = 255;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 1U;
            qY = u2;
            if (u2 > 65535U) {
              qY = 65535U;
            }

            if (contour_map[(contour_map_tmp + i40) - 1] > (int)qY) {
              /* 歩数MAP更新 */
              contour_map_tmp = (int)(contor_renew_square[tempn] + 1U);
              if ((unsigned int)contour_map_tmp > 255U) {
                contour_map_tmp = 255;
              }

              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contour_map_tmp + i40) - 1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              contour_map_tmp = (int)(contor_renew_square[tempn] + 1U);
              if ((unsigned int)contour_map_tmp > 255U) {
                contour_map_tmp = 255;
              }

              if (g_direction.North <= 7) {
                move_dir_map[(contour_map_tmp + i40) - 1] = (unsigned char)(1 <<
                  g_direction.North);
              } else {
                move_dir_map[(contour_map_tmp + i40) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              i40 = (int)(contor_renew_square[tempn] + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                (unsigned char)i40;
              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                contor_renew_square_idx;

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }

            /* かつ進行方向が北向きでないとき */
          } else {
            /* かつ北のマスの歩数MAP値が、更新予定値より大きい場合 */
            i50 = (int)(contor_renew_square[tempn] + 1U);
            i51 = i50;
            if ((unsigned int)i50 > 255U) {
              i51 = 255;
            }

            u2 = contour_map[contour_map_tmp] + 5U;
            qY = u2;
            if (u2 > 65535U) {
              qY = 65535U;
            }

            if (contour_map[(i51 + i40) - 1] > (int)qY) {
              /* 歩数MAP更新(重みづけあり) */
              contour_map_tmp = i50;
              if ((unsigned int)i50 > 255U) {
                contour_map_tmp = 255;
              }

              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contour_map_tmp + i40) - 1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              contour_map_tmp = i50;
              if ((unsigned int)i50 > 255U) {
                contour_map_tmp = 255;
              }

              if (g_direction.North <= 7) {
                move_dir_map[(contour_map_tmp + i40) - 1] = (unsigned char)(1 <<
                  g_direction.North);
              } else {
                move_dir_map[(contour_map_tmp + i40) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              if ((unsigned int)i50 > 255U) {
                i50 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                (unsigned char)i50;
              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                contor_renew_square_idx;

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }
          }
        }
      }

      /* 東側 */
      /* 壁が無い & (探索済み|| ~未知壁フラグ)であるとき */
      if (g_direction.East <= 7) {
        i45 = (unsigned char)(1 << g_direction.East);
      } else {
        i45 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i45) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i46 = (unsigned char)(1 << g_direction.East);
        } else {
          i46 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i46) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が東向きである時 */
          if (g_direction.East <= 7) {
            i49 = (unsigned char)(1 << g_direction.East);
          } else {
            i49 = 0;
          }

          if ((move_dir_map[(contor_renew_square[tempn] +
                             ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
               & i49) != 0) {
            /* かつ東のマスが更新予定値よりも大きな値の場合 */
            i40 = (int)(contor_renew_square[tempn + 1024] + 1U);
            if ((unsigned int)i40 > 255U) {
              i40 = 255;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 1U;
            if (u2 > 65535U) {
              u2 = 65535U;
            }

            if (contour_map[(contor_renew_square[tempn] + ((i40 - 1) << 5)) - 1]
                > (int)u2) {
              /*                                  %更新確認用のMAP更新 */
              /*                                  contour_refine_map(row(tempn),col(tempn)+1) = contour_refine_map(row(tempn),col(tempn))+uint16(1); */
              /* 歩数MAP更新 */
              i40 = (int)(contor_renew_square[tempn + 1024] + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              u2 = contour_map[(contor_renew_square[tempn] +
                                ((contor_renew_square[tempn + 1024] - 1) << 5))
                - 1] + 1U;
              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contor_renew_square[tempn] + ((i40 - 1) << 5)) - 1] =
                (unsigned short)u2;

              /* 移動方向MAP更新 */
              i40 = (int)(contor_renew_square[tempn + 1024] + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              if (g_direction.East <= 7) {
                move_dir_map[(contor_renew_square[tempn] + ((i40 - 1) << 5)) - 1]
                  = (unsigned char)(1 << g_direction.East);
              } else {
                move_dir_map[(contor_renew_square[tempn] + ((i40 - 1) << 5)) - 1]
                  = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                contor_renew_square[tempn];
              i40 = (int)(contor_renew_square[tempn + 1024] + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                (unsigned char)i40;

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }

            /* かつ進行方向が東向きでないとき */
          } else {
            /* かつ東のマスの歩数MAP値が、更新予定値より大きい場合 */
            i40 = (int)(contor_renew_square[tempn + 1024] + 1U);
            contour_map_tmp = i40;
            if ((unsigned int)i40 > 255U) {
              contour_map_tmp = 255;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 5U;
            if (u2 > 65535U) {
              u2 = 65535U;
            }

            if (contour_map[(contor_renew_square[tempn] + ((contour_map_tmp - 1)
                  << 5)) - 1] > (int)u2) {
              /*                                  %更新確認用のMAP更新 */
              /*                                  contour_refine_map(contor_renew_square(tempn,1),contor_renew_square(tempn,2)+1) = contour_refine_map(contor_renew_square(tempn,1),contor_renew_square(tempn,2))+uint16(1); */
              /* 歩数MAP更新(重みづけあり) */
              contour_map_tmp = i40;
              if ((unsigned int)i40 > 255U) {
                contour_map_tmp = 255;
              }

              u2 = contour_map[(contor_renew_square[tempn] +
                                ((contor_renew_square[tempn + 1024] - 1) << 5))
                - 1] + 5U;
              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contor_renew_square[tempn] + ((contour_map_tmp - 1) <<
                5)) - 1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              contour_map_tmp = i40;
              if ((unsigned int)i40 > 255U) {
                contour_map_tmp = 255;
              }

              if (g_direction.East <= 7) {
                move_dir_map[(contor_renew_square[tempn] + ((contour_map_tmp - 1)
                  << 5)) - 1] = (unsigned char)(1 << g_direction.East);
              } else {
                move_dir_map[(contor_renew_square[tempn] + ((contour_map_tmp - 1)
                  << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                contor_renew_square[tempn];
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                (unsigned char)i40;

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }
          }
        }
      }

      /* 南側 */
      /* 壁が無い &  (探索済み|| ~未知壁フラグ)であるとき */
      if (g_direction.South <= 7) {
        i48 = (unsigned char)(1 << g_direction.South);
      } else {
        i48 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i48) != 0) == wall->contents.nowall) {
        if (g_direction.South <= 7) {
          i52 = (unsigned char)(1 << g_direction.South);
        } else {
          i52 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i52) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が南向きである時 */
          if (g_direction.South <= 7) {
            i54 = (unsigned char)(1 << g_direction.South);
          } else {
            i54 = 0;
          }

          if ((move_dir_map[(contor_renew_square[tempn] +
                             ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
               & i54) != 0) {
            /* かつ南のマスが更新予定値よりも大きな値の場合 */
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 1U;
            if (u2 > 65535U) {
              u2 = 65535U;
            }

            if (contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) <<
                  5)) - 1] > (int)u2) {
              /*                                  %更新確認用のMAP更新 */
              /*                                  contour_refine_map(row(tempn)-1,col(tempn)) = contour_refine_map(row(tempn),col(tempn))+uint16(1); */
              /* 歩数MAP更新 */
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              u2 = contour_map[(contor_renew_square[tempn] +
                                ((contor_renew_square[tempn + 1024] - 1) << 5))
                - 1] + 1U;
              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) <<
                5)) - 1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              if (g_direction.South <= 7) {
                move_dir_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1)
                  << 5)) - 1] = (unsigned char)(1 << g_direction.South);
              } else {
                move_dir_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1)
                  << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                (unsigned char)qY;
              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                contor_renew_square[tempn + 1024];

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }

            /* かつ進行方向が南向きでないとき */
          } else {
            /* かつ南のマスの歩数MAP値が、更新予定値より大きい場合 */
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 5U;
            if (u2 > 65535U) {
              u2 = 65535U;
            }

            if (contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) <<
                  5)) - 1] > (int)u2) {
              /* 歩数MAP更新(重みづけあり) */
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              u2 = contour_map[(contor_renew_square[tempn] +
                                ((contor_renew_square[tempn + 1024] - 1) << 5))
                - 1] + 5U;
              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) <<
                5)) - 1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              if (g_direction.South <= 7) {
                move_dir_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1)
                  << 5)) - 1] = (unsigned char)(1 << g_direction.South);
              } else {
                move_dir_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1)
                  << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                (unsigned char)qY;
              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                contor_renew_square[tempn + 1024];

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }
          }
        }
      }

      /* 西側 */
      /* 壁が無い &  (探索済み|| ~未知壁フラグ)であるとき */
      if (g_direction.West <= 7) {
        i53 = (unsigned char)(1 << g_direction.West);
      } else {
        i53 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i53) != 0) == wall->contents.nowall) {
        if (g_direction.West <= 7) {
          i55 = (unsigned char)(1 << g_direction.West);
        } else {
          i55 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i55) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が西向きである時 */
          if (g_direction.West <= 7) {
            i56 = (unsigned char)(1 << g_direction.West);
          } else {
            i56 = 0;
          }

          if ((move_dir_map[(contor_renew_square[tempn] +
                             ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
               & i56) != 0) {
            /* かつ北のマスが更新予定値よりも大きな値の場合 */
            contour_map_tmp = contor_renew_square[tempn + 1024];
            qY = contour_map_tmp - 1U;
            if (qY > (unsigned int)contour_map_tmp) {
              qY = 0U;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 1U;
            if (u2 > 65535U) {
              u2 = 65535U;
            }

            if (contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5))
                - 1] > (int)u2) {
              /* 歩数MAP更新 */
              contour_map_tmp = contor_renew_square[tempn + 1024];
              qY = contour_map_tmp - 1U;
              if (qY > (unsigned int)contour_map_tmp) {
                qY = 0U;
              }

              u2 = contour_map[(contor_renew_square[tempn] +
                                ((contor_renew_square[tempn + 1024] - 1) << 5))
                - 1] + 1U;
              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) -
                1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              contour_map_tmp = contor_renew_square[tempn + 1024];
              qY = contour_map_tmp - 1U;
              if (qY > (unsigned int)contour_map_tmp) {
                qY = 0U;
              }

              if (g_direction.West <= 7) {
                move_dir_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5))
                  - 1] = (unsigned char)(1 << g_direction.West);
              } else {
                move_dir_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                contor_renew_square[tempn];
              contour_map_tmp = contor_renew_square[tempn + 1024];
              qY = contour_map_tmp - 1U;
              if (qY > (unsigned int)contour_map_tmp) {
                qY = 0U;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                (unsigned char)qY;

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }

            /* かつ進行方向が西向きでないとき */
          } else {
            /* かつ北のマスの歩数MAP値が、更新予定値より大きい場合 */
            contour_map_tmp = contor_renew_square[tempn + 1024];
            qY = contour_map_tmp - 1U;
            if (qY > (unsigned int)contour_map_tmp) {
              qY = 0U;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 5U;
            if (u2 > 65535U) {
              u2 = 65535U;
            }

            if (contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5))
                - 1] > (int)u2) {
              /* 歩数MAP更新(重みづけあり) */
              contour_map_tmp = contor_renew_square[tempn + 1024];
              qY = contour_map_tmp - 1U;
              if (qY > (unsigned int)contour_map_tmp) {
                qY = 0U;
              }

              u2 = contour_map[(contor_renew_square[tempn] +
                                ((contor_renew_square[tempn + 1024] - 1) << 5))
                - 1] + 5U;
              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) -
                1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              contour_map_tmp = contor_renew_square[tempn + 1024];
              qY = contour_map_tmp - 1U;
              if (qY > (unsigned int)contour_map_tmp) {
                qY = 0U;
              }

              if (g_direction.West <= 7) {
                move_dir_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5))
                  - 1] = (unsigned char)(1 << g_direction.West);
              } else {
                move_dir_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                contor_renew_square[tempn];
              contour_map_tmp = contor_renew_square[tempn + 1024];
              qY = contour_map_tmp - 1U;
              if (qY > (unsigned int)contour_map_tmp) {
                qY = 0U;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                (unsigned char)qY;

              /* 更新マス用インデックスを増加 */
              i40 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i40 > 255U) {
                i40 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i40;
            }
          }
        }
      }
    }

    /* ゴール更新マスの更新とインデックスのクリア */
    for (i37 = 0; i37 < 2048; i37++) {
      contor_renew_square[i37] = contor_renew_square_temp[i37];
      contor_renew_square_temp[i37] = 0U;
    }

    contor_renew_square_idx = (unsigned char)(contor_renew_square_idx_temp - 1);
    contor_renew_square_idx_temp = 1U;

    /* 更新がなければ || スタート地点が更新されていれば終了 */
    if ((change_flag == 0) || (contour_map[0] != 65535)) {
      exitg1 = true;
    } else {
      tempi++;
    }
  }
}

/*
 * 未知壁の領域は仮想壁をおいて侵入しない。
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数),迷路探索情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : coder_internal_ref_3 *max_length
 *                const coder_internal_ref_6 *wall
 *                const coder_internal_ref_5 *search
 *                const unsigned char maze_goal[18]
 *                unsigned char goal_size
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned short row_num_node[1056]
 *                unsigned short col_num_node[1056]
 *                unsigned short *start_num
 * Return Type  : void
 */
static void make_map_fustrun_diagonal(coder_internal_ref_3 *max_length, const
  coder_internal_ref_6 *wall, const coder_internal_ref_5 *search, const unsigned
  char maze_goal[18], unsigned char goal_size, const unsigned char maze_wall
  [1024], const unsigned char maze_wall_search[1024], unsigned short
  row_num_node[1056], unsigned short col_num_node[1056], unsigned short
  *start_num)
{
  unsigned char contor_renew_node_row_idx;
  unsigned char contor_renew_node_row_idx_temp;
  unsigned char contor_renew_node_row[2048];
  unsigned char contor_renew_node_row_temp[2048];
  unsigned char contor_renew_node_col[2048];
  unsigned char contor_renew_node_col_temp[2048];
  unsigned char contor_renew_node_col_idx;
  unsigned char contor_renew_node_col_idx_temp;
  int q0;
  unsigned char row_dir_node[1056];
  unsigned char col_dir_node[1056];
  int i76;
  int n;
  int i77;
  unsigned char u6;
  int i78;
  int i79;
  int i80;
  unsigned int qY;
  int i81;
  int row_num_node_tmp;
  unsigned short i;
  bool exitg1;
  int i82;
  int i83;
  int i84;
  unsigned char change_flag;
  int i85;
  int i86;
  int i87;
  int i88;
  int i89;
  int i90;
  int i91;
  int i92;
  int i93;
  unsigned int b_qY;
  int i94;
  int i95;
  int i96;
  int i97;
  int i98;
  unsigned int c_qY;
  int i99;
  int i100;
  int i101;
  int i102;
  unsigned int u7;
  int i103;
  int i104;
  int i105;
  int i106;
  int i107;
  int i108;
  int i109;
  int i110;
  int i111;
  int i112;
  int i113;
  int i114;
  int i115;
  int i116;
  int i117;
  int i118;
  int i119;
  int i120;
  int i121;
  int i122;
  int i123;
  int i124;
  int i125;
  int i126;
  int i127;
  int i128;
  int i129;
  int i130;
  int i131;
  int i132;
  int i133;
  int i134;
  int i135;
  int i136;
  int i137;
  int i138;
  int i139;
  int i140;
  int i141;
  int i142;
  int i143;
  int i144;
  int i145;
  int i146;
  int i147;
  int i148;
  int i149;

  /*     %% make_map_fustrun_diagonal 最短走行用等高線MAPを生成 */
  /* ローカル変数設定 */
  /* コンター更新ノード(行)保管用 */
  /* 更新座標 */
  /* 更新座標更新用 */
  contor_renew_node_row_idx = 1U;

  /* 更新座標 */
  contor_renew_node_row_idx_temp = 1U;

  /* 更新座標更新用 */
  /* コンター更新ノード（列）保管用 */
  /* 更新座標 */
  memset(&contor_renew_node_row[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_node_row_temp[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_node_col[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_node_col_temp[0], 0, sizeof(unsigned char) << 11);

  /* 更新座標更新用 */
  contor_renew_node_col_idx = 1U;

  /* 更新座標 */
  contor_renew_node_col_idx_temp = 1U;

  /* 更新座標更新用 */
  /* パラメータ設定 */
  /*  迷路パラメータ設定 */
  max_length->contents = 1024U;

  /*  ルートの重み設定 */
  /* MAPの初期化(すべてのノードにmax_lengthを入力) */
  /* 歩数MAP */
  /*  %更新用MAP */
  /*  row_num_node_temp = ones(33,32,'uint16')*uint16(65535); */
  /*  col_num_node_temp = ones(32,33,'uint16')*uint16(65535); */
  /* 進行方向保持用ノード作成 */
  for (q0 = 0; q0 < 1056; q0++) {
    row_num_node[q0] = MAX_uint16_T;
    col_num_node[q0] = MAX_uint16_T;
    row_dir_node[q0] = 0U;
    col_dir_node[q0] = 0U;
  }

  /* ゴールセクションが確定している場合 */
  *start_num = MAX_uint16_T;
  if (goal_size == 1) {
    /* ゴールマスから、東西南北にマップを展開 */
    /* 北壁 */
    q0 = maze_goal[0] - 1;
    i76 = (maze_goal[9] + (q0 << 5)) - 1;
    if (g_direction.North <= 7) {
      i77 = (unsigned char)(1 << g_direction.North);
    } else {
      i77 = 0;
    }

    if ((maze_wall[i76] & i77) == 0) {
      /* 歩数更新 */
      i78 = (int)(maze_goal[9] + 1U);
      i79 = i78;
      if ((unsigned int)i78 > 255U) {
        i79 = 255;
      }

      row_num_node_tmp = 33 * q0;
      row_num_node[(i79 + row_num_node_tmp) - 1] = 3U;

      /* 方向追加 */
      q0 = i78;
      if ((unsigned int)i78 > 255U) {
        q0 = 255;
      }

      if (g_d_direction.North <= 7) {
        row_dir_node[(q0 + row_num_node_tmp) - 1] = (unsigned char)(1 <<
          g_d_direction.North);
      } else {
        row_dir_node[(q0 + row_num_node_tmp) - 1] = 0U;
      }

      /* 更新ノードを更新 */
      if ((unsigned int)i78 > 255U) {
        i78 = 255;
      }

      contor_renew_node_row[0] = (unsigned char)i78;
      contor_renew_node_row[1024] = maze_goal[0];

      /* 更新マス用インデックスを増加 */
      contor_renew_node_row_idx = 2U;
    }

    /* 東壁 */
    if (g_direction.East <= 7) {
      i80 = (unsigned char)(1 << g_direction.East);
    } else {
      i80 = 0;
    }

    if ((maze_wall[i76] & i80) == 0) {
      /* 歩数更新 */
      q0 = (int)(maze_goal[0] + 1U);
      i78 = q0;
      if ((unsigned int)q0 > 255U) {
        i78 = 255;
      }

      col_num_node[(maze_goal[9] + ((i78 - 1) << 5)) - 1] = 3U;

      /* 方向追加 */
      i78 = q0;
      if ((unsigned int)q0 > 255U) {
        i78 = 255;
      }

      if (g_d_direction.East <= 7) {
        col_dir_node[(maze_goal[9] + ((i78 - 1) << 5)) - 1] = (unsigned char)(1 <<
          g_d_direction.East);
      } else {
        col_dir_node[(maze_goal[9] + ((i78 - 1) << 5)) - 1] = 0U;
      }

      /* 更新ノードを更新 */
      contor_renew_node_col[0] = maze_goal[9];
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      contor_renew_node_col[1024] = (unsigned char)q0;

      /* 更新マス用インデックスを増加 */
      contor_renew_node_col_idx = 2U;
    }

    /* 南壁 */
    if (g_direction.South <= 7) {
      i83 = (unsigned char)(1 << g_direction.South);
    } else {
      i83 = 0;
    }

    if ((maze_wall[i76] & i83) == 0) {
      /* 歩数更新 */
      row_num_node_tmp = (maze_goal[9] + 33 * (maze_goal[0] - 1)) - 1;
      row_num_node[row_num_node_tmp] = 3U;

      /* 方向追加 */
      if (g_d_direction.South <= 7) {
        i87 = (unsigned char)(1 << g_d_direction.South);
      } else {
        i87 = 0;
      }

      row_dir_node[row_num_node_tmp] = (unsigned char)
        (row_dir_node[row_num_node_tmp] | i87);

      /* 更新ノードを更新 */
      contor_renew_node_row[contor_renew_node_row_idx - 1] = maze_goal[9];
      contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[0];

      /* 更新マス用インデックスを増加 */
      contor_renew_node_row_idx++;
    }

    /* 西壁 */
    if (g_direction.West <= 7) {
      i85 = (unsigned char)(1 << g_direction.West);
    } else {
      i85 = 0;
    }

    if ((maze_wall[i76] & i85) == 0) {
      /* 歩数更新 */
      col_num_node[i76] = 3U;

      /* 方向追加 */
      if (g_d_direction.West <= 7) {
        i89 = (unsigned char)(1 << g_d_direction.West);
      } else {
        i89 = 0;
      }

      col_dir_node[i76] = (unsigned char)(col_dir_node[i76] | i89);

      /* 更新ノードを更新 */
      contor_renew_node_col[contor_renew_node_col_idx - 1] = maze_goal[9];
      contor_renew_node_col[contor_renew_node_col_idx + 1023] = maze_goal[0];

      /* 更新マス用インデックスを増加 */
      contor_renew_node_col_idx++;
    }

    /* ゴールセクションが確定していない場合 */
  } else {
    /* ゴールノードに */
    /*  歩数：0を入力 */
    /*  進行方向 : 壁がなければ全方向=0b11111111=255 */
    /*  を入力 */
    q0 = goal_size;
    for (n = 0; n < q0; n++) {
      /* 北壁 */
      i76 = maze_goal[n] - 1;
      u6 = maze_goal[n + 9];
      i78 = (u6 + (i76 << 5)) - 1;
      if (g_direction.North <= 7) {
        i81 = (unsigned char)(1 << g_direction.North);
      } else {
        i81 = 0;
      }

      if ((maze_wall[i78] & i81) == 0) {
        /* 歩数更新 */
        i79 = (int)(maze_goal[n + 9] + 1U);
        i82 = i79;
        if ((unsigned int)i79 > 255U) {
          i82 = 255;
        }

        row_num_node_tmp = 33 * i76;
        row_num_node[(i82 + row_num_node_tmp) - 1] = 0U;

        /* 方向更新 */
        i76 = i79;
        if ((unsigned int)i79 > 255U) {
          i76 = 255;
        }

        row_dir_node[(i76 + row_num_node_tmp) - 1] = MAX_uint8_T;

        /* 更新ノードを更新 */
        if ((unsigned int)i79 > 255U) {
          i79 = 255;
        }

        contor_renew_node_row[contor_renew_node_row_idx - 1] = (unsigned char)
          i79;
        contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[n];

        /* 更新マス用インデックスを増加 */
        i76 = (int)(contor_renew_node_row_idx + 1U);
        if ((unsigned int)i76 > 255U) {
          i76 = 255;
        }

        contor_renew_node_row_idx = (unsigned char)i76;
      }

      /* 東壁 */
      if (g_direction.East <= 7) {
        i84 = (unsigned char)(1 << g_direction.East);
      } else {
        i84 = 0;
      }

      if ((maze_wall[i78] & i84) == 0) {
        /* 歩数更新 */
        i76 = (int)(maze_goal[n] + 1U);
        i79 = i76;
        if ((unsigned int)i76 > 255U) {
          i79 = 255;
        }

        col_num_node[(u6 + ((i79 - 1) << 5)) - 1] = 0U;

        /* 方向更新 */
        i79 = i76;
        if ((unsigned int)i76 > 255U) {
          i79 = 255;
        }

        col_dir_node[(u6 + ((i79 - 1) << 5)) - 1] = MAX_uint8_T;

        /* 更新ノードを更新 */
        contor_renew_node_col[contor_renew_node_col_idx - 1] = u6;
        if ((unsigned int)i76 > 255U) {
          i76 = 255;
        }

        contor_renew_node_col[contor_renew_node_col_idx + 1023] = (unsigned char)
          i76;

        /* 更新マス用インデックスを増加 */
        i76 = (int)(contor_renew_node_col_idx + 1U);
        if ((unsigned int)i76 > 255U) {
          i76 = 255;
        }

        contor_renew_node_col_idx = (unsigned char)i76;
      }

      /* 南壁 */
      if (g_direction.South <= 7) {
        i86 = (unsigned char)(1 << g_direction.South);
      } else {
        i86 = 0;
      }

      if ((maze_wall[i78] & i86) == 0) {
        /* 歩数更新 */
        row_num_node_tmp = (u6 + 33 * (maze_goal[n] - 1)) - 1;
        row_num_node[row_num_node_tmp] = 0U;

        /* 方向更新 */
        row_dir_node[row_num_node_tmp] = MAX_uint8_T;

        /* 更新ノードを更新 */
        contor_renew_node_row[contor_renew_node_row_idx - 1] = u6;
        contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[n];

        /* 更新マス用インデックスを増加 */
        i76 = (int)(contor_renew_node_row_idx + 1U);
        if ((unsigned int)i76 > 255U) {
          i76 = 255;
        }

        contor_renew_node_row_idx = (unsigned char)i76;
      }

      /* 西壁 */
      if (g_direction.West <= 7) {
        i88 = (unsigned char)(1 << g_direction.West);
      } else {
        i88 = 0;
      }

      if ((maze_wall[i78] & i88) == 0) {
        /* 歩数更新 */
        col_num_node[i78] = 0U;

        /* 方向更新 */
        col_dir_node[i78] = MAX_uint8_T;

        /* 更新ノードを更新 */
        contor_renew_node_col[contor_renew_node_col_idx - 1] = u6;
        contor_renew_node_col[contor_renew_node_col_idx + 1023] = maze_goal[n];

        /* 更新マス用インデックスを増加 */
        i76 = (int)(contor_renew_node_col_idx + 1U);
        if ((unsigned int)i76 > 255U) {
          i76 = 255;
        }

        contor_renew_node_col_idx = (unsigned char)i76;
      }
    }

    /*      %更新判定用変数(重みづけなしの歩数マップ) */
    /*      row_num_node_temp = row_num_node; */
    /*      col_num_node_temp = col_num_node; */
  }

  q0 = max_length->contents;
  qY = q0 - 1U;
  if (qY > (unsigned int)q0) {
    qY = 0U;
  }

  i = 0U;
  exitg1 = false;
  while ((!exitg1) && (i <= (unsigned short)qY)) {
    /* 更新確認用の歩数カウントは0~max_length */
    change_flag = 0U;

    /* map更新確認用フラグ */
    /* Row_Edgeの処理[33行,32列] */
    /* 検索した座標に対し、歩数mapを更新 */
    q0 = contor_renew_node_row_idx;
    for (n = 0; n <= q0 - 2; n++) {
      /* 北側 */
      /* 壁が無い & 探索済みであるとき */
      i76 = contor_renew_node_row[n + 1024] - 1;
      i78 = (contor_renew_node_row[n] + (i76 << 5)) - 1;
      if (g_direction.North <= 7) {
        i90 = (unsigned char)(1 << g_direction.North);
      } else {
        i90 = 0;
      }

      if (((maze_wall[i78] & i90) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i91 = (unsigned char)(1 << g_direction.North);
        } else {
          i91 = 0;
        }

        if (((maze_wall_search[i78] & i91) != 0) == search->contents.known) {
          /* かつ進行方向が北向きである時 */
          i76 *= 33;
          i78 = (contor_renew_node_row[n] + i76) - 1;
          if (g_d_direction.North <= 7) {
            i96 = (unsigned char)(1 << g_d_direction.North);
          } else {
            i96 = 0;
          }

          if ((row_dir_node[i78] & i96) != 0) {
            /* かつ北のノードが更新予定値よりも大きな値の場合 */
            i78 = (int)(contor_renew_node_row[n] + 1U);
            if ((unsigned int)i78 > 255U) {
              i78 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i78 + i76) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i76 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(1 << g_d_direction.North);
              } else {
                row_dir_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i76 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i76;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i76 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = (int)(contor_renew_node_row[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                i78 = (int)(contor_renew_node_row[n] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i112 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i112 = 0;
                }

                row_dir_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(row_dir_node[(i78 + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i112);
              }
            }

            /* かつ進行方向が北向きでないとき */
          } else {
            /* かつ北のノードの歩数MAP値が、更新予定値より大きい場合 */
            i79 = (int)(contor_renew_node_row[n] + 1U);
            if ((unsigned int)i79 > 255U) {
              i79 = 255;
            }

            b_qY = row_num_node[i78] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i79 + i76) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i76 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(1 << g_d_direction.North);
              } else {
                row_dir_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i76 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i76;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i76 = (int)(contor_renew_node_row[n] + 1U);
              i78 = i76;
              if ((unsigned int)i76 > 255U) {
                i78 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i78 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i78 = i76;
                if ((unsigned int)i76 > 255U) {
                  i78 = 255;
                  i76 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i110 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i110 = 0;
                }

                row_dir_node[(i78 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(row_dir_node[(i76 + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i110);
              }
            }
          }
        }
      }

      /* 北東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.East <= 7) {
        i93 = (unsigned char)(1 << g_direction.East);
      } else {
        i93 = 0;
      }

      if (((maze_wall[(contor_renew_node_row[n] + ((contor_renew_node_row[n +
               1024] - 1) << 5)) - 1] & i93) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i95 = (unsigned char)(1 << g_direction.East);
        } else {
          i95 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_row[n] +
                                ((contor_renew_node_row[n + 1024] - 1) << 5)) -
              1] & i95) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i99 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i99 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i99)
              != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新 */
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i76;

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i120 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i120 = 0;
                }

                col_dir_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_row[n] + ((i78
                  - 1) << 5)) - 1] | i120);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            i78 = (contor_renew_node_row[n] + 33 * (contor_renew_node_row[n +
                    1024] - 1)) - 1;
            b_qY = row_num_node[i78] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i76;

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              i79 = i76;
              if ((unsigned int)i76 > 255U) {
                i79 = 255;
              }

              b_qY = row_num_node[i78] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[n] + ((i79 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i78 = i76;
                if ((unsigned int)i76 > 255U) {
                  i78 = 255;
                  i76 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i118 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i118 = 0;
                }

                col_dir_node[(contor_renew_node_row[n] + ((i78 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_row[n] + ((i76
                  - 1) << 5)) - 1] | i118);
              }
            }
          }
        }
      }

      /* 東側は柱 */
      /* 南東側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row[n] - 1U;
      if (c_qY > contor_renew_node_row[n]) {
        c_qY = 0U;
      }

      i76 = (contor_renew_node_row[n + 1024] - 1) << 5;
      if (g_direction.East <= 7) {
        i101 = (unsigned char)(1 << g_direction.East);
      } else {
        i101 = 0;
      }

      if (((maze_wall[((int)c_qY + i76) - 1] & i101) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[n] - 1U;
        if (c_qY > contor_renew_node_row[n]) {
          c_qY = 0U;
        }

        if (g_direction.East <= 7) {
          i105 = (unsigned char)(1 << g_direction.East);
        } else {
          i105 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i76) - 1] & i105) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i108 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i108 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i108)
              != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i78 > 255U) {
              i78 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              10U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i78 - 1) << 5)) - 1] > (int)u7) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i78;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i78 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                i79 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i79 > 255U) {
                  i79 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i139 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i139 = 0;
                }

                col_dir_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i79 - 1) << 5)) - 1] | i139);
              }
            }

            /* かつ進行方向が南東向きでないとき */
          } else {
            /* かつ南東のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i78 > 255U) {
              i78 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              18U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i78 - 1) << 5)) - 1] > (int)u7) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i78;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i78 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                i78 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                i79 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i79 > 255U) {
                  i79 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i138 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i138 = 0;
                }

                col_dir_node[((int)c_qY + ((i78 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i79 - 1) << 5)) - 1] | i138);
              }
            }
          }
        }
      }

      /* 南側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row[n] - 1U;
      if (c_qY > contor_renew_node_row[n]) {
        c_qY = 0U;
      }

      if (g_direction.South <= 7) {
        i106 = (unsigned char)(1 << g_direction.South);
      } else {
        i106 = 0;
      }

      if (((maze_wall[((int)c_qY + i76) - 1] & i106) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[n] - 1U;
        if (c_qY > contor_renew_node_row[n]) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i111 = (unsigned char)(1 << g_direction.South);
        } else {
          i111 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i76) - 1] & i111) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南向きである時 */
          if (g_d_direction.South <= 7) {
            i117 = (unsigned char)(1 << g_d_direction.South);
          } else {
            i117 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i117)
              != 0) {
            /* かつ南のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              6U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            if (row_num_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] > (int)u7) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024] -
                1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (g_d_direction.South <= 7) {
                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = (unsigned char)(1 << g_d_direction.South);
              } else {
                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i78;

              /* かつ南のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                    - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South <= 7) {
                  i137 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i137 = 0;
                }

                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = (unsigned char)(row_dir_node[((int)b_qY + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i137);
              }
            }

            /* かつ進行方向が南向きでないとき */
          } else {
            /* かつ南のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024] -
                1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (g_d_direction.South <= 7) {
                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = (unsigned char)(1 << g_d_direction.South);
              } else {
                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i78;

              /* かつ南のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                    - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South <= 7) {
                  i136 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i136 = 0;
                }

                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = (unsigned char)(row_dir_node[((int)b_qY + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i136);
              }
            }
          }
        }
      }

      /* 南西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = contor_renew_node_row[n] - 1U;
      if (c_qY > contor_renew_node_row[n]) {
        c_qY = 0U;
      }

      if (g_direction.West <= 7) {
        i115 = (unsigned char)(1 << g_direction.West);
      } else {
        i115 = 0;
      }

      if (((maze_wall[((int)c_qY + i76) - 1] & i115) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[n] - 1U;
        if (c_qY > contor_renew_node_row[n]) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i124 = (unsigned char)(1 << g_direction.West);
        } else {
          i124 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i76) - 1] & i124) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i130 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i130 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i130)
              != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[((int)c_qY + i76) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + i76) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i76) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i76) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + i76) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i145 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i145 = 0;
                }

                col_dir_node[((int)c_qY + i76) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i76) - 1] | i145);
              }
            }

            /* かつ進行方向が南西向きでないとき */
          } else {
            /* かつ南西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[((int)c_qY + i76) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + i76) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i76) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i76) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                (unsigned char)c_qY;
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + i76) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i144 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i144 = 0;
                }

                col_dir_node[((int)c_qY + i76) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i76) - 1] | i144);
              }
            }
          }
        }
      }

      /* 北西側 */
      /* 壁が無い & 探索済みであるとき */
      i76 = (contor_renew_node_row[n] + i76) - 1;
      if (g_direction.West <= 7) {
        i126 = (unsigned char)(1 << g_direction.West);
      } else {
        i126 = 0;
      }

      if (((maze_wall[i76] & i126) != 0) == wall->contents.nowall) {
        if (g_direction.West <= 7) {
          i129 = (unsigned char)(1 << g_direction.West);
        } else {
          i129 = 0;
        }

        if (((maze_wall_search[i76] & i129) != 0) == search->contents.known) {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i133 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i133 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i133)
              != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[n] +
                              ((contor_renew_node_row[n + 1024] - 1) << 5)) - 1]
                > (int)b_qY) {
              /* 歩数MAP更新 */
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i76] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i76] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i76] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[n] +
                                ((contor_renew_node_row[n + 1024] - 1) << 5)) -
                  1] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i143 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i143 = 0;
                }

                col_dir_node[i76] = (unsigned char)(col_dir_node[i76] | i143);
              }
            }

            /* かつ進行方向が北西向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[i76] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i76] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i76] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i76] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[i76] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i142 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i142 = 0;
                }

                col_dir_node[i76] = (unsigned char)(col_dir_node[i76] | i142);
              }
            }
          }
        }
      }
    }

    /* Col_Edgeの処理[32行,33列] */
    /* 検索した座標に対し、歩数mapを更新 */
    q0 = contor_renew_node_col_idx;
    for (n = 0; n <= q0 - 2; n++) {
      /* 北側は壁 */
      /* 北東側 */
      /* 壁が無い & 探索済みであるとき */
      i76 = (contor_renew_node_col[n] + ((contor_renew_node_col[n + 1024] - 1) <<
              5)) - 1;
      if (g_direction.North <= 7) {
        i92 = (unsigned char)(1 << g_direction.North);
      } else {
        i92 = 0;
      }

      if (((maze_wall[i76] & i92) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i94 = (unsigned char)(1 << g_direction.North);
        } else {
          i94 = 0;
        }

        if (((maze_wall_search[i76] & i94) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i98 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i98 = 0;
          }

          if ((col_dir_node[i76] & i98) != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i76 = (int)(contor_renew_node_col[n] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 10U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            if (row_num_node[(i76 + 33 * (contor_renew_node_col[n + 1024] - 1))
                - 1] > (int)u7) {
              /* 歩数MAP更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i76 + 33 * (contor_renew_node_col[n + 1024] - 1)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i76 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] = (unsigned char)(1 << g_d_direction.North_East);
              } else {
                row_dir_node[(i76 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i76;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[n + 1024];

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i76 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                i78 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i116 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i116 = 0;
                }

                row_dir_node[(i76 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] = (unsigned char)(row_dir_node[(i78 + 33 *
                  (contor_renew_node_col[n + 1024] - 1)) - 1] | i116);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i76 = (int)(contor_renew_node_col[n] + 1U);
            i78 = i76;
            if ((unsigned int)i76 > 255U) {
              i78 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 18U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            i79 = 33 * (contor_renew_node_col[n + 1024] - 1);
            if (row_num_node[(i78 + i79) - 1] > (int)u7) {
              /* 歩数MAP更新(重みづけあり) */
              i78 = i76;
              if ((unsigned int)i76 > 255U) {
                i78 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i78 + i79) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i78 = i76;
              if ((unsigned int)i76 > 255U) {
                i78 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i78 + i79) - 1] = (unsigned char)(1 <<
                  g_d_direction.North_East);
              } else {
                row_dir_node[(i78 + i79) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i76;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[n + 1024];

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i78 = i76;
              if ((unsigned int)i76 > 255U) {
                i78 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i78 + i79) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i78 = i76;
                if ((unsigned int)i76 > 255U) {
                  i78 = 255;
                  i76 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i114 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i114 = 0;
                }

                row_dir_node[(i78 + i79) - 1] = (unsigned char)(row_dir_node
                  [(i76 + i79) - 1] | i114);
              }
            }
          }
        }
      }

      /* 東側 */
      /* 壁が無い & 探索済みであるとき */
      u6 = contor_renew_node_col[n + 1024];
      i76 = (contor_renew_node_col[n] + ((u6 - 1) << 5)) - 1;
      if (g_direction.East <= 7) {
        i97 = (unsigned char)(1 << g_direction.East);
      } else {
        i97 = 0;
      }

      if (((maze_wall[i76] & i97) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i100 = (unsigned char)(1 << g_direction.East);
        } else {
          i100 = 0;
        }

        if (((maze_wall_search[i76] & i100) != 0) == search->contents.known) {
          /* かつ進行方向が東向きである時 */
          if (g_d_direction.East <= 7) {
            i103 = (unsigned char)(1 << g_d_direction.East);
          } else {
            i103 = 0;
          }

          if ((col_dir_node[i76] & i103) != 0) {
            /* かつ東のノードが更新予定値よりも大きな値の場合 */
            i78 = (int)(contor_renew_node_col[n + 1024] + 1U);
            if ((unsigned int)i78 > 255U) {
              i78 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[n] + ((i78 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新 */
              i78 = (int)(contor_renew_node_col[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[n] + ((i78 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i78 = (int)(contor_renew_node_col[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[n] + ((i78 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[n] + ((i78 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[n];
              i78 = (int)(contor_renew_node_col[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i78;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i78 = (int)(contor_renew_node_col[n + 1024] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[n] + ((i78 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i78 = (int)(contor_renew_node_col[n + 1024] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                i79 = (int)(contor_renew_node_col[n + 1024] + 1U);
                if ((unsigned int)i79 > 255U) {
                  i79 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i128 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i128 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + ((i78 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_col[n] + ((i79
                  - 1) << 5)) - 1] | i128);
              }
            }

            /* かつ進行方向が東向きでないとき */
          } else {
            /* かつ東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i78 = (int)(contor_renew_node_col[n + 1024] + 1U);
            i79 = i78;
            if ((unsigned int)i78 > 255U) {
              i79 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[n] + ((i79 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i79 = i78;
              if ((unsigned int)i78 > 255U) {
                i79 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[n] + ((i79 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i79 = i78;
              if ((unsigned int)i78 > 255U) {
                i79 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[n] + ((i79 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[n] + ((i79 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[n];
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i78;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i79 = i78;
              if ((unsigned int)i78 > 255U) {
                i79 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[n] + ((i79 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i79 = i78;
                if ((unsigned int)i78 > 255U) {
                  i79 = 255;
                  i78 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i125 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i125 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + ((i79 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_col[n] + ((i78
                  - 1) << 5)) - 1] | i125);
              }
            }
          }
        }
      }

      /* 南東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.South <= 7) {
        i102 = (unsigned char)(1 << g_direction.South);
      } else {
        i102 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + ((contor_renew_node_col[n +
               1024] - 1) << 5)) - 1] & i102) != 0) == wall->contents.nowall) {
        if (g_direction.South <= 7) {
          i104 = (unsigned char)(1 << g_direction.South);
        } else {
          i104 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] +
                                ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] & i104) != 0) == search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i107 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i107 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[n] +
                             ((contor_renew_node_col[n + 1024] - 1) << 5)) - 1]
               & i107) != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[n] + 33 *
                              (contor_renew_node_col[n + 1024] - 1)) - 1] > (int)
                b_qY) {
              /* 歩数MAP更新 */
              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(contor_renew_node_col[n] + 33 *
                            (contor_renew_node_col[n + 1024] - 1)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.South_East <= 7) {
                row_dir_node[(contor_renew_node_col[n] + 33 *
                              (contor_renew_node_col[n + 1024] - 1)) - 1] =
                  (unsigned char)(1 << g_d_direction.South_East);
              } else {
                row_dir_node[(contor_renew_node_col[n] + 33 *
                              (contor_renew_node_col[n + 1024] - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[n];
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                u6;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i78;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[n] + 33 *
                                (contor_renew_node_col[n + 1024] - 1)) - 1] ==
                  (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i122 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i122 = 0;
                }

                row_dir_node[(contor_renew_node_col[n] + 33 *
                              (contor_renew_node_col[n + 1024] - 1)) - 1] =
                  (unsigned char)(row_dir_node[(contor_renew_node_col[n] + 33 *
                  (contor_renew_node_col[n + 1024] - 1)) - 1] | i122);
              }
            }

            /* かつ進行方向が南東向きでないとき */
          } else {
            /* かつ南東のノードの歩数MAP値が、更新予定値より大きい場合 */
            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            i78 = (contor_renew_node_col[n] + 33 * (contor_renew_node_col[n +
                    1024] - 1)) - 1;
            if (row_num_node[i78] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[i78] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.South_East <= 7) {
                row_dir_node[i78] = (unsigned char)(1 <<
                  g_d_direction.South_East);
              } else {
                row_dir_node[i78] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[n];
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                u6;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i78;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[i76] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[i78] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i123 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i123 = 0;
                }

                row_dir_node[i78] = (unsigned char)(row_dir_node[i78] | i123);
              }
            }
          }
        }
      }

      /* 南側は柱 */
      /* 南西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = u6 - 1U;
      if (c_qY > u6) {
        c_qY = 0U;
      }

      if (g_direction.South <= 7) {
        i109 = (unsigned char)(1 << g_direction.South);
      } else {
        i109 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) - 1] &
            i109) != 0) == wall->contents.nowall) {
        c_qY = u6 - 1U;
        if (c_qY > u6) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i113 = (unsigned char)(1 << g_direction.South);
        } else {
          i113 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
              - 1] & i113) != 0) == search->contents.known) {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i121 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i121 = 0;
          }

          if ((col_dir_node[i76] & i121) != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) - 1]
                = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = (unsigned char)(1 << g_d_direction.South_West);
              } else {
                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[n];
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i78;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i141 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i141 = 0;
                }

                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = (unsigned char)(row_dir_node[(contor_renew_node_col[n] +
                  33 * ((int)b_qY - 1)) - 1] | i141);
              }
            }

            /* かつ進行方向が南西向きでないとき */
          } else {
            /* かつ南西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i76] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) - 1]
                = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = (unsigned char)(1 << g_d_direction.South_West);
              } else {
                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[n];
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i78;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i76] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.South_West <= 7) {
                  i140 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i140 = 0;
                }

                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = (unsigned char)(row_dir_node[(contor_renew_node_col[n] +
                  33 * ((int)b_qY - 1)) - 1] | i140);
              }
            }
          }
        }
      }

      /* 西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = u6 - 1U;
      if (c_qY > u6) {
        c_qY = 0U;
      }

      if (g_direction.West <= 7) {
        i119 = (unsigned char)(1 << g_direction.West);
      } else {
        i119 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) - 1] &
            i119) != 0) == wall->contents.nowall) {
        c_qY = u6 - 1U;
        if (c_qY > u6) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i127 = (unsigned char)(1 << g_direction.West);
        } else {
          i127 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
              - 1] & i127) != 0) == search->contents.known) {
          /* かつ進行方向が西向きである時 */
          if (g_d_direction.West <= 7) {
            i132 = (unsigned char)(1 << g_d_direction.West);
          } else {
            i132 = 0;
          }

          if ((col_dir_node[i76] & i132) != 0) {
            /* かつ西のノードが更新予定値よりも大きな値の場合 */
            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.West <= 7) {
                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = (unsigned char)(1 << g_d_direction.West);
              } else {
                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[n];
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.West <= 7) {
                  i147 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i147 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = (unsigned char)(col_dir_node[(contor_renew_node_col[n]
                  + (((int)b_qY - 1) << 5)) - 1] | i147);
              }
            }

            /* かつ進行方向が西向きでないとき */
          } else {
            /* かつ西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.West <= 7) {
                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = (unsigned char)(1 << g_d_direction.West);
              } else {
                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[n];
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i78 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i78;

              /* かつ西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i76] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.West <= 7) {
                  i146 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i146 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = (unsigned char)(col_dir_node[(contor_renew_node_col[n]
                  + (((int)b_qY - 1) << 5)) - 1] | i146);
              }
            }
          }
        }
      }

      /* 北西側 */
      /* 壁が無い & 探索済みであるとき */
      c_qY = u6 - 1U;
      if (c_qY > u6) {
        c_qY = 0U;
      }

      if (g_direction.North <= 7) {
        i131 = (unsigned char)(1 << g_direction.North);
      } else {
        i131 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) - 1] &
            i131) != 0) == wall->contents.nowall) {
        c_qY = u6 - 1U;
        if (c_qY > u6) {
          c_qY = 0U;
        }

        if (g_direction.North <= 7) {
          i134 = (unsigned char)(1 << g_direction.North);
        } else {
          i134 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
              - 1] & i134) != 0) == search->contents.known) {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i135 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i135 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[n] +
                             ((contor_renew_node_col[n + 1024] - 1) << 5)) - 1]
               & i135) != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            i76 = (int)(contor_renew_node_col[n] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 10U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i76 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i76;
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 10U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i76 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                i78 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i149 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i149 = 0;
                }

                row_dir_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i78 + 33 * ((int)b_qY - 1)) - 1] | i149);
              }
            }

            /* かつ進行方向が北西向きでないとき */
          } else {
            /* かつ北西のノードの歩数MAP値が、更新予定値より大きい場合 */
            i78 = (int)(contor_renew_node_col[n] + 1U);
            if ((unsigned int)i78 > 255U) {
              i78 = 255;
            }

            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i76] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i78 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i76 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i76;
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              i78 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i78 > 255U) {
                i78 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i76] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i78 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                i78 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i78 > 255U) {
                  i78 = 255;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i148 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i148 = 0;
                }

                row_dir_node[(i76 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i78 + 33 * ((int)b_qY - 1)) - 1] | i148);
              }
            }
          }
        }
      }
    }

    /* ゴール更新ノードの更新とインデックスのクリア */
    contor_renew_node_col_idx = contor_renew_node_col_idx_temp;
    contor_renew_node_col_idx_temp = 1U;
    for (q0 = 0; q0 < 2048; q0++) {
      contor_renew_node_col[q0] = contor_renew_node_col_temp[q0];
      contor_renew_node_col_temp[q0] = 0U;
      contor_renew_node_row[q0] = contor_renew_node_row_temp[q0];
      contor_renew_node_row_temp[q0] = 0U;
    }

    contor_renew_node_row_idx = contor_renew_node_row_idx_temp;
    contor_renew_node_row_idx_temp = 1U;

    /* 更新がなければ終了(スタート地点の歩数マップを更新) */
    if (change_flag == 0) {
      b_qY = row_num_node[1] + 3U;
      if (b_qY > 65535U) {
        b_qY = 65535U;
      }

      *start_num = (unsigned short)b_qY;
      exitg1 = true;
    } else {
      i++;
    }
  }
}

/*
 * 迷路パラメータ設定
 * Arguments    : const coder_internal_ref_6 *wall
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned short contour_map[1024]
 *                unsigned char new_goal[2]
 * Return Type  : void
 */
static void make_new_goal_all(const coder_internal_ref_6 *wall, const unsigned
  char maze_wall[1024], const unsigned char maze_wall_search[1024], unsigned
  char current_x, unsigned char current_y, unsigned short contour_map[1024],
  unsigned char new_goal[2])
{
  unsigned char contor_renew_square[2048];
  unsigned char contor_renew_square_temp[2048];
  unsigned char contor_renew_square_idx;
  unsigned char contor_renew_square_idx_temp;
  int q0;
  unsigned short tempi;
  bool exitg1;
  unsigned char change_flag;
  int tempn;
  bool exitg2;
  int i30;
  bool guard1 = false;
  bool guard2 = false;
  bool guard3 = false;
  int i31;
  int i32;
  int i33;
  int i34;
  int i35;
  int i36;
  unsigned int u1;
  unsigned int qY;

  /*     %% make_new_goal_all 現在位置から新規のゴールを作る。(全面探索用) */
  /* 新規ゴール座標格納用変数 */
  new_goal[0] = 0U;
  new_goal[1] = 0U;

  /* コンター更新マス保管用 */
  /* 更新座標 */
  memset(&contor_renew_square[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_square_temp[0], 0, sizeof(unsigned char) << 11);

  /* 更新座標更新用 */
  contor_renew_square_idx = 1U;

  /* 更新座標 */
  contor_renew_square_idx_temp = 1U;

  /* 更新座標更新用 */
  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* 32マス分mapを保持 */
  /* 16bitにすべき */
  for (q0 = 0; q0 < 1024; q0++) {
    contour_map[q0] = MAX_uint16_T;
  }

  /* スタート地点の歩数を1だけ減らし、判別可能な状態にする。 */
  contour_map[(current_y + ((current_x - 1) << 5)) - 1] = 65534U;

  /* 初回の更新座標 = 現在位置　を入力 */
  contor_renew_square[0] = current_y;
  contor_renew_square[1024] = current_x;

  /* 現在の位置からコンターを展開。 */
  /* 未探索位置にコンターが展開されればそこを新規ゴールとし、終了する。 */
  tempi = 0U;
  exitg1 = false;
  while ((!exitg1) && (tempi < 65535)) {
    /* 歩数カウントは0~max_length */
    /* map更新確認用フラグ */
    change_flag = 0U;

    /* 更新された座標に対し、コンターマップを展開 */
    tempn = 0;
    exitg2 = false;
    while ((!exitg2) && (tempn <= contor_renew_square_idx - 1)) {
      /* 北側 */
      q0 = maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn +
        1024] - 1) << 5)) - 1];
      if (g_direction.North <= 7) {
        i30 = (unsigned char)(1 << g_direction.North);
      } else {
        i30 = 0;
      }

      guard1 = false;
      guard2 = false;
      guard3 = false;
      if ((q0 & i30) == wall->contents.nowall) {
        /* 北側のMAPが更新されているか判断、されていなければ書き込み */
        i31 = (int)(contor_renew_square[tempn] + 1U);
        if ((unsigned int)i31 > 255U) {
          i31 = 255;
        }

        i32 = (contor_renew_square[tempn + 1024] - 1) << 5;
        if (contour_map[(i31 + i32) - 1] == 65535) {
          i31 = (int)(contor_renew_square[tempn] + 1U);
          i34 = i31;
          if ((unsigned int)i31 > 255U) {
            i34 = 255;
          }

          u1 = tempi + 2U;
          if (u1 > 65535U) {
            u1 = 65535U;
          }

          contour_map[(i34 + i32) - 1] = (unsigned short)(65535 - (int)u1);
          change_flag = 1U;

          /* 更新マスを更新 */
          i34 = i31;
          if ((unsigned int)i31 > 255U) {
            i34 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)i34;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square[tempn + 1024];

          /* 更新マス用インデックスを増加 */
          i34 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i34 > 255U) {
            i34 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i34;

          /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
          i34 = i31;
          if ((unsigned int)i31 > 255U) {
            i34 = 255;
          }

          if (maze_wall_search[(i34 + i32) - 1] != 15) {
            new_goal[0] = contor_renew_square[tempn + 1024];
            if ((unsigned int)i31 > 255U) {
              i31 = 255;
            }

            new_goal[1] = (unsigned char)i31;
            exitg2 = true;
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }

      if (guard3) {
        /* 東側 */
        if (g_direction.East <= 7) {
          i33 = (unsigned char)(1 << g_direction.East);
        } else {
          i33 = 0;
        }

        if ((q0 & i33) == wall->contents.nowall) {
          /* 東側のMAPが更新されているか判断、されていなければ書き込み */
          q0 = (int)(contor_renew_square[tempn + 1024] + 1U);
          i31 = q0;
          if ((unsigned int)q0 > 255U) {
            i31 = 255;
          }

          if (contour_map[(contor_renew_square[tempn] + ((i31 - 1) << 5)) - 1] ==
              65535) {
            i31 = q0;
            if ((unsigned int)q0 > 255U) {
              i31 = 255;
            }

            u1 = tempi + 2U;
            if (u1 > 65535U) {
              u1 = 65535U;
            }

            contour_map[(contor_renew_square[tempn] + ((i31 - 1) << 5)) - 1] =
              (unsigned short)(65535 - (int)u1);
            change_flag = 1U;

            /* 更新マスを更新 */
            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              contor_renew_square[tempn];
            i31 = q0;
            if ((unsigned int)q0 > 255U) {
              i31 = 255;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              (unsigned char)i31;

            /* 更新マス用インデックスを増加 */
            i31 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i31 > 255U) {
              i31 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i31;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            i31 = q0;
            if ((unsigned int)q0 > 255U) {
              i31 = 255;
            }

            if (maze_wall_search[(contor_renew_square[tempn] + ((i31 - 1) << 5))
                - 1] != 15) {
              if ((unsigned int)q0 > 255U) {
                q0 = 255;
              }

              new_goal[0] = (unsigned char)q0;
              new_goal[1] = contor_renew_square[tempn];
              exitg2 = true;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
      }

      if (guard2) {
        /* 南側 */
        q0 = maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
          + 1024] - 1) << 5)) - 1];
        if (g_direction.South <= 7) {
          i35 = (unsigned char)(1 << g_direction.South);
        } else {
          i35 = 0;
        }

        if ((q0 & i35) == wall->contents.nowall) {
          /* 南側のMAPが更新されているか判断、されていなければ書き込み */
          qY = contor_renew_square[tempn] - 1U;
          if (qY > contor_renew_square[tempn]) {
            qY = 0U;
          }

          if (contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) <<
                5)) - 1] == 65535) {
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            u1 = tempi + 2U;
            if (u1 > 65535U) {
              u1 = 65535U;
            }

            contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) << 5))
              - 1] = (unsigned short)(65535 - (int)u1);
            change_flag = 1U;

            /* 更新マスを更新 */
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              (unsigned char)qY;
            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              contor_renew_square[tempn + 1024];

            /* 更新マス用インデックスを増加 */
            i31 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i31 > 255U) {
              i31 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i31;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            if (maze_wall_search[((int)qY + ((contor_renew_square[tempn + 1024]
                   - 1) << 5)) - 1] != 15) {
              new_goal[0] = contor_renew_square[tempn + 1024];
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              new_goal[1] = (unsigned char)qY;
              exitg2 = true;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      }

      if (guard1) {
        /* 西側 */
        if (g_direction.West <= 7) {
          i36 = (unsigned char)(1 << g_direction.West);
        } else {
          i36 = 0;
        }

        if ((q0 & i36) == wall->contents.nowall) {
          /* 西側のMAPが更新されているか判断、されていなければ書き込み */
          q0 = contor_renew_square[tempn + 1024];
          qY = q0 - 1U;
          if (qY > (unsigned int)q0) {
            qY = 0U;
          }

          if (contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) -
              1] == 65535) {
            q0 = contor_renew_square[tempn + 1024];
            qY = q0 - 1U;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            u1 = tempi + 2U;
            if (u1 > 65535U) {
              u1 = 65535U;
            }

            contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) - 1]
              = (unsigned short)(65535 - (int)u1);
            change_flag = 1U;

            /* 更新マスを更新 */
            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              contor_renew_square[tempn];
            q0 = contor_renew_square[tempn + 1024];
            qY = q0 - 1U;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              (unsigned char)qY;

            /* 更新マス用インデックスを増加 */
            q0 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)q0;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            q0 = contor_renew_square[tempn + 1024];
            qY = q0 - 1U;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            if (maze_wall_search[(contor_renew_square[tempn] + (((int)qY - 1) <<
                  5)) - 1] != 15) {
              q0 = contor_renew_square[tempn + 1024];
              qY = q0 - 1U;
              if (qY > (unsigned int)q0) {
                qY = 0U;
              }

              new_goal[0] = (unsigned char)qY;
              new_goal[1] = contor_renew_square[tempn];
              exitg2 = true;
            } else {
              tempn++;
            }
          } else {
            tempn++;
          }
        } else {
          tempn++;
        }
      }
    }

    /* ゴール更新マスの更新とインデックスのクリア */
    for (q0 = 0; q0 < 2048; q0++) {
      contor_renew_square[q0] = contor_renew_square_temp[q0];
      contor_renew_square_temp[q0] = 0U;
    }

    contor_renew_square_idx = (unsigned char)(contor_renew_square_idx_temp - 1);
    contor_renew_square_idx_temp = 1U;

    /* 更新がない、もしくはゴールが設定されていれば終了 */
    if ((change_flag == 0) || (new_goal[0] != 0)) {
      /* disp(tempi) */
      exitg1 = true;
    } else {
      tempi++;
    }
  }
}

/*
 * 迷路パラメータ設定
 * Arguments    : const coder_internal_ref_6 *wall
 *                const unsigned char maze_wall[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                const unsigned char unexp_square[1024]
 *                unsigned char unexp_square_idx
 *                unsigned short contour_map[1024]
 *                unsigned char new_goal[2]
 * Return Type  : void
 */
static void make_new_goal_sh(const coder_internal_ref_6 *wall, const unsigned
  char maze_wall[1024], unsigned char current_x, unsigned char current_y, const
  unsigned char unexp_square[1024], unsigned char unexp_square_idx, unsigned
  short contour_map[1024], unsigned char new_goal[2])
{
  unsigned char contor_renew_square[2048];
  unsigned char contor_renew_square_temp[2048];
  unsigned char contor_renew_square_idx;
  unsigned char contor_renew_square_idx_temp;
  int q0;
  unsigned short tempi;
  bool exitg1;
  unsigned char change_flag;
  int tempn;
  bool exitg2;
  int i64;
  bool guard1 = false;
  bool guard2 = false;
  bool guard3 = false;
  int i65;
  int i66;
  int i67;
  int i68;
  int i69;
  int i70;
  unsigned int u4;
  unsigned int qY;

  /*     %% make_new_goal_sh 現在位置から新規のゴール生成する。(最短経路探索用) */
  /* 新規ゴール座標格納用変数 */
  new_goal[0] = 0U;
  new_goal[1] = 0U;

  /* コンター更新マス保管用 */
  /* 更新座標 */
  memset(&contor_renew_square[0], 0, sizeof(unsigned char) << 11);
  memset(&contor_renew_square_temp[0], 0, sizeof(unsigned char) << 11);

  /* 更新座標更新用 */
  contor_renew_square_idx = 1U;

  /* 更新座標 */
  contor_renew_square_idx_temp = 1U;

  /* 更新座標更新用 */
  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* 32マス分mapを保持 */
  /* 16bitにすべき */
  for (q0 = 0; q0 < 1024; q0++) {
    contour_map[q0] = MAX_uint16_T;
  }

  /* スタート地点の歩数を1だけ減らし、判別可能な状態にする。 */
  contour_map[(current_y + ((current_x - 1) << 5)) - 1] = 65534U;

  /* 初回の更新座標 = 現在位置　を入力 */
  contor_renew_square[0] = current_y;
  contor_renew_square[1024] = current_x;

  /* 現在の位置からコンターを展開。 */
  /* 最短経路にコンターが展開されればそこを新規ゴールとし、終了する。 */
  tempi = 0U;
  exitg1 = false;
  while ((!exitg1) && (tempi < 65535)) {
    /* 歩数カウントは0~max_length */
    /* map更新確認用フラグ */
    change_flag = 0U;

    /* 更新された座標に対し、コンターマップを展開 */
    tempn = 0;
    exitg2 = false;
    while ((!exitg2) && (tempn <= contor_renew_square_idx - 1)) {
      /* 北側 */
      q0 = maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn +
        1024] - 1) << 5)) - 1];
      if (g_direction.North <= 7) {
        i64 = (unsigned char)(1 << g_direction.North);
      } else {
        i64 = 0;
      }

      guard1 = false;
      guard2 = false;
      guard3 = false;
      if ((q0 & i64) == wall->contents.nowall) {
        /* 北側のMAPが更新されているか判断、されていなければ書き込み */
        i65 = (int)(contor_renew_square[tempn] + 1U);
        if ((unsigned int)i65 > 255U) {
          i65 = 255;
        }

        i66 = (contor_renew_square[tempn + 1024] - 1) << 5;
        if (contour_map[(i65 + i66) - 1] == 65535) {
          i65 = (int)(contor_renew_square[tempn] + 1U);
          i68 = i65;
          if ((unsigned int)i65 > 255U) {
            i68 = 255;
          }

          u4 = tempi + 2U;
          if (u4 > 65535U) {
            u4 = 65535U;
          }

          contour_map[(i68 + i66) - 1] = (unsigned short)(65535 - (int)u4);
          change_flag = 1U;

          /* 更新マスを更新 */
          i66 = i65;
          if ((unsigned int)i65 > 255U) {
            i66 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)i66;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square[tempn + 1024];

          /* 更新マス用インデックスを増加 */
          i66 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i66 > 255U) {
            i66 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i66;

          /* 更新した地点が最短経路未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
          i66 = i65;
          if ((unsigned int)i65 > 255U) {
            i66 = 255;
          }

          if (sh_route_unexp_sq_jud(unexp_square, unexp_square_idx, (unsigned
                char)i66, contor_renew_square[tempn + 1024]) == 1) {
            new_goal[0] = contor_renew_square[tempn + 1024];
            if ((unsigned int)i65 > 255U) {
              i65 = 255;
            }

            new_goal[1] = (unsigned char)i65;
            exitg2 = true;
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }

      if (guard3) {
        /* 東側 */
        if (g_direction.East <= 7) {
          i67 = (unsigned char)(1 << g_direction.East);
        } else {
          i67 = 0;
        }

        if ((q0 & i67) == wall->contents.nowall) {
          /* 東側のMAPが更新されているか判断、されていなければ書き込み */
          q0 = (int)(contor_renew_square[tempn + 1024] + 1U);
          i65 = q0;
          if ((unsigned int)q0 > 255U) {
            i65 = 255;
          }

          if (contour_map[(contor_renew_square[tempn] + ((i65 - 1) << 5)) - 1] ==
              65535) {
            i65 = q0;
            if ((unsigned int)q0 > 255U) {
              i65 = 255;
            }

            u4 = tempi + 2U;
            if (u4 > 65535U) {
              u4 = 65535U;
            }

            contour_map[(contor_renew_square[tempn] + ((i65 - 1) << 5)) - 1] =
              (unsigned short)(65535 - (int)u4);
            change_flag = 1U;

            /* 更新マスを更新 */
            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              contor_renew_square[tempn];
            i65 = q0;
            if ((unsigned int)q0 > 255U) {
              i65 = 255;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              (unsigned char)i65;

            /* 更新マス用インデックスを増加 */
            i65 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i65 > 255U) {
              i65 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i65;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            i65 = q0;
            if ((unsigned int)q0 > 255U) {
              i65 = 255;
            }

            if (sh_route_unexp_sq_jud(unexp_square, unexp_square_idx,
                 contor_renew_square[tempn], (unsigned char)i65) == 1) {
              if ((unsigned int)q0 > 255U) {
                q0 = 255;
              }

              new_goal[0] = (unsigned char)q0;
              new_goal[1] = contor_renew_square[tempn];
              exitg2 = true;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
      }

      if (guard2) {
        /* 南側 */
        q0 = maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
          + 1024] - 1) << 5)) - 1];
        if (g_direction.South <= 7) {
          i69 = (unsigned char)(1 << g_direction.South);
        } else {
          i69 = 0;
        }

        if ((q0 & i69) == wall->contents.nowall) {
          /* 南側のMAPが更新されているか判断、されていなければ書き込み */
          qY = contor_renew_square[tempn] - 1U;
          if (qY > contor_renew_square[tempn]) {
            qY = 0U;
          }

          if (contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) <<
                5)) - 1] == 65535) {
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            u4 = tempi + 2U;
            if (u4 > 65535U) {
              u4 = 65535U;
            }

            contour_map[((int)qY + ((contor_renew_square[tempn + 1024] - 1) << 5))
              - 1] = (unsigned short)(65535 - (int)u4);
            change_flag = 1U;

            /* 更新マスを更新 */
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              (unsigned char)qY;
            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              contor_renew_square[tempn + 1024];

            /* 更新マス用インデックスを増加 */
            i65 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i65 > 255U) {
              i65 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i65;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            qY = contor_renew_square[tempn] - 1U;
            if (qY > contor_renew_square[tempn]) {
              qY = 0U;
            }

            if (sh_route_unexp_sq_jud(unexp_square, unexp_square_idx, (unsigned
                  char)qY, contor_renew_square[tempn + 1024]) == 1) {
              new_goal[0] = contor_renew_square[tempn + 1024];
              qY = contor_renew_square[tempn] - 1U;
              if (qY > contor_renew_square[tempn]) {
                qY = 0U;
              }

              new_goal[1] = (unsigned char)qY;
              exitg2 = true;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      }

      if (guard1) {
        /* 西側 */
        if (g_direction.West <= 7) {
          i70 = (unsigned char)(1 << g_direction.West);
        } else {
          i70 = 0;
        }

        if ((q0 & i70) == wall->contents.nowall) {
          /* 西側のMAPが更新されているか判断、されていなければ書き込み */
          q0 = contor_renew_square[tempn + 1024];
          qY = q0 - 1U;
          if (qY > (unsigned int)q0) {
            qY = 0U;
          }

          if (contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) -
              1] == 65535) {
            q0 = contor_renew_square[tempn + 1024];
            qY = q0 - 1U;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            u4 = tempi + 2U;
            if (u4 > 65535U) {
              u4 = 65535U;
            }

            contour_map[(contor_renew_square[tempn] + (((int)qY - 1) << 5)) - 1]
              = (unsigned short)(65535 - (int)u4);
            change_flag = 1U;

            /* 更新マスを更新 */
            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              contor_renew_square[tempn];
            q0 = contor_renew_square[tempn + 1024];
            qY = q0 - 1U;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              (unsigned char)qY;

            /* 更新マス用インデックスを増加 */
            q0 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)q0;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            q0 = contor_renew_square[tempn + 1024];
            qY = q0 - 1U;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            if (sh_route_unexp_sq_jud(unexp_square, unexp_square_idx,
                 contor_renew_square[tempn], (unsigned char)qY) == 1) {
              q0 = contor_renew_square[tempn + 1024];
              qY = q0 - 1U;
              if (qY > (unsigned int)q0) {
                qY = 0U;
              }

              new_goal[0] = (unsigned char)qY;
              new_goal[1] = contor_renew_square[tempn];
              exitg2 = true;
            } else {
              tempn++;
            }
          } else {
            tempn++;
          }
        } else {
          tempn++;
        }
      }
    }

    /* ゴール更新マスの更新とインデックスのクリア */
    for (q0 = 0; q0 < 2048; q0++) {
      contor_renew_square[q0] = contor_renew_square_temp[q0];
      contor_renew_square_temp[q0] = 0U;
    }

    contor_renew_square_idx = (unsigned char)(contor_renew_square_idx_temp - 1);
    contor_renew_square_idx_temp = 1U;

    /* 更新がない、もしくはゴールが設定されていれば終了 */
    if ((change_flag == 0) || (new_goal[0] != 0)) {
      /* disp(tempi) */
      exitg1 = true;
    } else {
      tempi++;
    }
  }
}

/*
 * スタートノードの初期化
 * Arguments    : const coder_internal_ref *start_flg
 *                const coder_internal_ref *wall_flg
 *                const unsigned short row_num_node[1056]
 *                const unsigned short col_num_node[1056]
 *                const unsigned char goal_section[2]
 *                const unsigned char goal_node2[2]
 *                unsigned char goal_node_property
 * Return Type  : void
 */
static void make_route_diagonal(const coder_internal_ref *start_flg, const
  coder_internal_ref *wall_flg, const unsigned short row_num_node[1056], const
  unsigned short col_num_node[1056], const unsigned char goal_section[2], const
  unsigned char goal_node2[2], unsigned char goal_node_property)
{
  unsigned char current_node[2];
  unsigned char current_node_property;
  unsigned char current_move_dir;
  unsigned char current_move_mode;
  unsigned char straight_count;
  unsigned char ref_move_dir;
  unsigned char ref_move_mode;
  int goal_flag;
  unsigned char next_node[2];
  unsigned char next_move_dir;
  unsigned char b_next_node[2];
  unsigned char next_node_property;
  int exitg1;
  int i;
  double move_dir_buffer[3];
  unsigned char ref_node_property;
  bool exitg2;
  bool p;
  bool b_p;
  int k;
  unsigned char turn_pattern_num;
  unsigned char b_next_node_property;
  unsigned char b_next_move_dir;
  unsigned int qY;

  /*     %% make_route_diagonal 斜め有での最短ルート生成、走行 */
  current_node[0] = 1U;
  current_node[1] = 1U;

  /* 初期マスの南のノード */
  current_node_property = matrix_dir.Row;

  /* 南のノードの属性は行方向 */
  current_move_dir = g_d_direction.North;

  /* 初期の進行方向は北 */
  current_move_mode = move_dir_property.straight;

  /* 初期の進行方向属性は直進（斜めでない） */
  straight_count = 0U;

  /* 直進用のカウンタ */
  /* 初回のみ次のノードを固定 */
  /* 初期マスの北のノード */
  /* 北のノードの属性は行方向 */
  /* 基準ノード座標を次のノードに */
  /* 基準ノードの移動方向、モードを現在のノードに設定 */
  ref_move_dir = g_d_direction.North;
  ref_move_mode = move_dir_property.straight;

  /* ゴールフラグがたつまでループ */
  goal_flag = 0;

  /* 斜め込みの進行方法選択 */
  next_node[0] = 2U;
  next_node[1] = 1U;
  get_next_dir_diagonal(row_num_node, col_num_node, g_d_direction.North,
                        next_node, matrix_dir.Row, goal_node2,
                        goal_node_property, goal_section, &next_move_dir,
                        b_next_node, &next_node_property);
  do {
    exitg1 = 0;

    /*          disp(["next_move_dir=",num2str(next_move_dir)]) */
    /*          disp(["ref_move_dir=",num2str(ref_move_dir)]) */
    /* 基準ノードから見て、進行方向が同一のとき */
    if (ref_move_dir == next_move_dir) {
      /*              disp("直進") */
      i = (int)(straight_count + 1U);
      if ((unsigned int)i > 255U) {
        i = 255;
      }

      straight_count = (unsigned char)i;

      /* 直進カウンタをインクリメント */
      /* 基準ノードを移動する。 */
      ref_node_property = next_node_property;
      ref_move_dir = next_move_dir;

      /* 基準ノードがゴールであるか判定 */
      p = false;
      b_p = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 2)) {
        if (b_next_node[k] != goal_node2[k]) {
          b_p = false;
          exitg2 = true;
        } else {
          k++;
        }
      }

      if (b_p) {
        p = true;
      }

      if ((p && (next_node_property == goal_node_property)) ||
          ((next_node_property == matrix_dir.section) && (goal_section[1] ==
            b_next_node[0]) && (goal_section[0] == b_next_node[1]))) {
        /* 現在位置がノードである時、ゴール進入時のパターンを決定する。 */
        /*                  if (isequal(ref_node,goal_node2) && (ref_node_property == goal_node_property)) */
        /* ゴールの場合、ゴール進入時のパターンを決定する。 */
        get_next_dir_diagonal(row_num_node, col_num_node, next_move_dir,
                              b_next_node, next_node_property, goal_node2,
                              goal_node_property, goal_section, &b_next_move_dir,
                              next_node, &b_next_node_property);

        /* 直進の場合（直進侵入） */
        if (next_move_dir == b_next_move_dir) {
          /* ゴール分カウンタを増加 */
          i = (int)((unsigned char)i + 1U);
          if ((unsigned int)i > 255U) {
            i = 255;
          }

          /* 軌跡をプロット */
          /* 直進走行 */
          m_move_front_long((unsigned char)i, start_flg->contents,
                            wall_flg->contents, current_move_mode);

          /* 現在ノードから直進カウンタ分移動する関数。 */
          move_straight(current_node, &current_node_property, &current_move_dir,
                        &current_move_mode, (unsigned char)i);
          straight_count = 0U;

          /*                          disp("ゴール直線侵入") */
          /*                          disp(current_node) */
          /* ターンの場合（斜め侵入） */
          /* 45度ターンのみ想定(一つ先の移動方向でターンの種類が決定) */
        } else {
          /*                          disp("ゴール時直進前") */
          /*                          disp(current_node) */
          /* 直進カウンタがあれば移動 */
          /* 軌跡をプロット */
          /* 直進走行 */
          m_move_front_long((unsigned char)i, start_flg->contents,
                            wall_flg->contents, current_move_mode);

          /* 現在ノードから直進カウンタ分移動する関数。 */
          move_straight(current_node, &current_node_property, &current_move_dir,
                        &current_move_mode, (unsigned char)i);
          straight_count = 0U;

          /*                          disp("ゴール時直進後") */
          /*                          disp(current_node) */
          /* 進行方向のバッファをクリア */
          move_dir_buffer[1] = 0.0;
          move_dir_buffer[2] = 0.0;

          /* 1つ先までの進行方向から、ターンのパターンを決定 */
          /* 現在進行方向からの相対的な移動方向をバッファ */
          if (b_next_move_dir > 127) {
            b_next_move_dir = 127U;
          }

          turn_pattern_num = current_move_dir;
          if (current_move_dir > 127) {
            turn_pattern_num = 127U;
          }

          move_dir_buffer[0] = (b_next_move_dir - turn_pattern_num) & 7;

          /* ターンの種類を判定する */
          turn_pattern_num = get_turn_pattern_num(move_dir_buffer, ref_move_mode);

          /* ターンの軌跡を描画する */
          /* パターンが決まらなかった場合、エラー */
          /* パターンに応じた移動処理 */
          if (turn_pattern_num == turn_pattern.r_45) {
            /* 右45度ターン移動 */
            m_turn_45_r(start_flg->contents, wall_flg->contents,
                        current_move_mode);
            turn_r_45(current_node, &current_node_property, &current_move_dir,
                      &current_move_mode);
          } else {
            if (turn_pattern_num == turn_pattern.l_45) {
              /* 左45度ターン移動 */
              m_turn_45_l(start_flg->contents, wall_flg->contents,
                          current_move_mode);
              turn_l_45(current_node, &current_node_property, &current_move_dir,
                        &current_move_mode);
            }
          }
        }

        /* 参照位置がセクションであるとき */
        /*                  else %セクションでここに入ることはない。はず。 */
        /*                      %直進カウンタがあれば、移動する。 */
        /*                      %                     disp("ゴール時直進（セクション）") */
        /*                      if straight_count > 0 */
        /*                          %軌跡をプロット */
        /*                          if coder.target('MATLAB') */
        /*                              straight_plot(current_node,current_node_property,current_move_dir,current_move_mode,straight_count); */
        /*                          end */
        /*                          %現在ノードから直進カウンタ分移動する関数。 */
        /*                          [current_node,current_node_property,current_move_dir,current_move_mode,straight_count]... */
        /*                              = move_straight(current_node,current_node_property,current_move_dir,current_move_mode,straight_count); */
        /*                      end */
        /*                  end */
        /* ゴール処理がおわったらフラグを立てる */
        /*                  disp("ゴール後") */
        /*                  disp(straight_count) */
        /*                  disp(current_node) */
        goal_flag = 1;
      }

      /* 基準ノードから見て、進行方向が異なる(曲がる)とき */
    } else {
      /*              disp("ターン") */
      /* 直進カウンタあるとき */
      if (straight_count > 0) {
        /* 軌跡をプロット */
        /* 直進走行 */
        m_move_front_long(straight_count, start_flg->contents,
                          wall_flg->contents, current_move_mode);

        /* 現在ノードから直進カウンタ分移動する関数。 */
        move_straight(current_node, &current_node_property, &current_move_dir,
                      &current_move_mode, straight_count);
        straight_count = 0U;

        /*                  disp("ターン前直進カウンタ") */
        /*                  disp(straight_count) */
        /*                  disp(current_node) */
      }

      /* 進行方向のバッファをクリア */
      move_dir_buffer[0] = 0.0;
      move_dir_buffer[1] = 0.0;
      move_dir_buffer[2] = 0.0;

      /* 3つ先までの進行方向から、ターンのパターンを決定 */
      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i < 3)) {
        /* 現在進行方向からの相対的な移動方向をバッファ */
        /*                  disp(["current_move_dir",num2str(current_move_dir)]) */
        /*                  disp(["next_move_dir",num2str(next_move_dir)]) */
        turn_pattern_num = next_move_dir;
        if (next_move_dir > 127) {
          turn_pattern_num = 127U;
        }

        b_next_node_property = current_move_dir;
        if (current_move_dir > 127) {
          b_next_node_property = 127U;
        }

        move_dir_buffer[i] = (turn_pattern_num - b_next_node_property) & 7;

        /* ターンの種類を判定する */
        turn_pattern_num = get_turn_pattern_num(move_dir_buffer, ref_move_mode);

        /* パターンが確定していれば終了 */
        if (turn_pattern_num != 0) {
          /* パターンの軌跡を描画 */
          exitg2 = true;
        } else {
          /* 次の進行方向を決定する */
          next_node[0] = b_next_node[0];
          next_node[1] = b_next_node[1];
          get_next_dir_diagonal(row_num_node, col_num_node, next_move_dir,
                                next_node, next_node_property, goal_node2,
                                goal_node_property, goal_section, &next_move_dir,
                                b_next_node, &next_node_property);

          /* 3回目でパターンが決まらなかった場合、エラー */
          i++;
        }
      }

      /* パターンに応じて移動 */
      if (turn_pattern_num == turn_pattern.r_45) {
        /* 右45度ターン移動 */
        m_turn_45_r(start_flg->contents, wall_flg->contents, current_move_mode);
        turn_r_45(current_node, &current_node_property, &current_move_dir,
                  &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.l_45) {
        /* 左45度ターン移動 */
        m_turn_45_l(start_flg->contents, wall_flg->contents, current_move_mode);
        turn_l_45(current_node, &current_node_property, &current_move_dir,
                  &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.r_90) {
        /* 右90度ターン移動 */
        if (current_move_mode == move_dir_property.straight) {
          m_turn_90_r(start_flg->contents, wall_flg->contents, current_move_mode);
        } else {
          if (current_move_mode == move_dir_property.diagonal) {
            m_turn_V90_r(start_flg->contents, wall_flg->contents,
                         current_move_mode);
          }
        }

        /* 直進パターンの時 */
        /*  右90度 */
        if (current_move_mode == move_dir_property.straight) {
          if (current_move_dir == g_d_direction.North) {
            i = (int)(current_node[0] + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            current_node[0] = (unsigned char)i;
            i = (int)(current_node[1] + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            current_node[1] = (unsigned char)i;
            current_node_property = matrix_dir.Col;
            current_move_dir = g_d_direction.East;
            current_move_mode = move_dir_property.straight;
          } else if (current_move_dir == g_d_direction.East) {
            i = (int)(current_node[1] + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            current_node[1] = (unsigned char)i;
            current_node_property = matrix_dir.Row;
            current_move_dir = g_d_direction.South;
            current_move_mode = move_dir_property.straight;
          } else if (current_move_dir == g_d_direction.South) {
            qY = current_node[0] - 2U;
            if (qY > current_node[0]) {
              qY = 0U;
            }

            current_node[0] = (unsigned char)qY;
            current_node_property = matrix_dir.Col;
            current_move_dir = g_d_direction.West;
            current_move_mode = move_dir_property.straight;
          } else {
            if (current_move_dir == g_d_direction.West) {
              i = (int)(current_node[0] + 1U);
              if ((unsigned int)i > 255U) {
                i = 255;
              }

              current_node[0] = (unsigned char)i;
              qY = current_node[1] - 2U;
              if (qY > current_node[1]) {
                qY = 0U;
              }

              current_node[1] = (unsigned char)qY;
              current_node_property = matrix_dir.Row;
              current_move_dir = g_d_direction.North;
              current_move_mode = move_dir_property.straight;
            }
          }

          /* 斜めパターンの時（V90） */
        } else {
          if (current_move_mode == move_dir_property.diagonal) {
            if (current_move_dir == g_d_direction.North_East) {
              i = (int)(current_node[1] + 1U);
              if ((unsigned int)i > 255U) {
                i = 255;
              }

              current_node[1] = (unsigned char)i;
              current_node_property = matrix_dir.Row;
              current_move_dir = g_d_direction.South_East;
              current_move_mode = move_dir_property.diagonal;
            } else if (current_move_dir == g_d_direction.South_East) {
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              current_node[0] = (unsigned char)qY;
              current_node_property = matrix_dir.Col;
              current_move_dir = g_d_direction.South_West;
              current_move_mode = move_dir_property.diagonal;
            } else if (current_move_dir == g_d_direction.South_West) {
              qY = current_node[1] - 1U;
              if (qY > current_node[1]) {
                qY = 0U;
              }

              current_node[1] = (unsigned char)qY;
              current_node_property = matrix_dir.Row;
              current_move_dir = g_d_direction.North_West;
              current_move_mode = move_dir_property.diagonal;
            } else {
              if (current_move_dir == g_d_direction.North_West) {
                i = (int)(current_node[0] + 1U);
                if ((unsigned int)i > 255U) {
                  i = 255;
                }

                current_node[0] = (unsigned char)i;
                current_node_property = matrix_dir.Col;
                current_move_dir = g_d_direction.North_East;
                current_move_mode = move_dir_property.diagonal;
              }
            }
          }
        }
      } else if (turn_pattern_num == turn_pattern.l_90) {
        /* 左90度ターン移動 */
        if (current_move_mode == move_dir_property.straight) {
          m_turn_90_l(start_flg->contents, wall_flg->contents, current_move_mode);
        } else {
          if (current_move_mode == move_dir_property.diagonal) {
            m_turn_V90_l(start_flg->contents, wall_flg->contents,
                         current_move_mode);
          }
        }

        turn_l_90(current_node, &current_node_property, &current_move_dir,
                  &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.r_135) {
        /* 右135度ターン移動 */
        m_turn_135_r(start_flg->contents, wall_flg->contents, current_move_mode);
        turn_r_135(current_node, &current_node_property, &current_move_dir,
                   &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.l_135) {
        /* 左135度ターン移動 */
        m_turn_135_l(start_flg->contents, wall_flg->contents, current_move_mode);
        turn_l_135(current_node, &current_node_property, &current_move_dir,
                   &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.r_180) {
        /* 右180度ターン移動 */
        m_turn_180_r(start_flg->contents, wall_flg->contents, current_move_mode);

        /* 直進パターンの時 */
        /*  右180度 */
        if (current_move_mode == move_dir_property.straight) {
          if (current_move_dir == g_d_direction.North) {
            i = (int)(current_node[0] + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            current_node[0] = (unsigned char)i;
            i = (int)(current_node[1] + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            current_node[1] = (unsigned char)i;
            current_node_property = matrix_dir.Row;
            current_move_dir = g_d_direction.South;
            current_move_mode = move_dir_property.straight;
          } else if (current_move_dir == g_d_direction.East) {
            qY = current_node[0] - 1U;
            if (qY > current_node[0]) {
              qY = 0U;
            }

            current_node[0] = (unsigned char)qY;
            i = (int)(current_node[1] + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            current_node[1] = (unsigned char)i;
            current_node_property = matrix_dir.Col;
            current_move_dir = g_d_direction.West;
            current_move_mode = move_dir_property.straight;
          } else if (current_move_dir == g_d_direction.South) {
            qY = current_node[0] - 1U;
            if (qY > current_node[0]) {
              qY = 0U;
            }

            current_node[0] = (unsigned char)qY;
            qY = current_node[1] - 1U;
            if (qY > current_node[1]) {
              qY = 0U;
            }

            current_node[1] = (unsigned char)qY;
            current_node_property = matrix_dir.Row;
            current_move_dir = g_d_direction.North;
            current_move_mode = move_dir_property.straight;
          } else {
            if (current_move_dir == g_d_direction.West) {
              i = (int)(current_node[0] + 1U);
              if ((unsigned int)i > 255U) {
                i = 255;
              }

              current_node[0] = (unsigned char)i;
              qY = current_node[1] - 1U;
              if (qY > current_node[1]) {
                qY = 0U;
              }

              current_node[1] = (unsigned char)qY;
              current_node_property = matrix_dir.Col;
              current_move_dir = g_d_direction.East;
              current_move_mode = move_dir_property.straight;
            }
          }

          /* 斜めパターンの時 */
        }
      } else {
        if (turn_pattern_num == turn_pattern.l_180) {
          /* 左180度ターン移動 */
          m_turn_180_l(start_flg->contents, wall_flg->contents,
                       current_move_mode);
          turn_l_180(current_node, &current_node_property, &current_move_dir,
                     &current_move_mode);
        }
      }

      /*              disp("ターン終了時、ノード") */
      /*              disp(current_node) */
      /*              disp(goal_node2) */
      /* 移動後、ゴールを判定 */
      p = false;
      b_p = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 2)) {
        if (current_node[k] != goal_node2[k]) {
          b_p = false;
          exitg2 = true;
        } else {
          k++;
        }
      }

      if (b_p) {
        p = true;
      }

      if (p && (current_node_property == goal_node_property)) {
        goal_flag = 1;
      }

      /* 基準ノードを移動 */
      ref_node_property = next_node_property;
      ref_move_dir = current_move_dir;
      ref_move_mode = current_move_mode;
    }

    /* ゴールフラグが立っていれば終了 */
    if (goal_flag == 1) {
      exitg1 = 1;
    } else {
      /* 次の移動方向を取得 */
      /* 斜め込みの進行方法選択 */
      next_node[0] = b_next_node[0];
      next_node[1] = b_next_node[1];
      get_next_dir_diagonal(row_num_node, col_num_node, ref_move_dir, next_node,
                            ref_node_property, goal_node2, goal_node_property,
                            goal_section, &next_move_dir, b_next_node,
                            &next_node_property);

      /*          disp("次進行方向") */
      /*          disp(next_move_dir) */
    }
  } while (exitg1 == 0);

  m_goal_movement(start_flg->contents, wall_flg->contents, current_move_mode);
}

/*
 * 入力 現在位置x,y,現在方向
 * 出力 現在位置x,y
 * Arguments    : unsigned char *temp_x
 *                unsigned char *temp_y
 *                unsigned char temp_dir
 * Return Type  : void
 */
static void move_step(unsigned char *temp_x, unsigned char *temp_y, unsigned
                      char temp_dir)
{
  int q0;
  unsigned int qY;

  /*     %% move_step １マスだけ前進する関数 */
  /* 北に一マス */
  if (temp_dir == g_direction.North) {
    q0 = (int)(*temp_y + 1U);
    if ((unsigned int)q0 > 255U) {
      q0 = 255;
    }

    *temp_y = (unsigned char)q0;

    /* disp("north_step") */
  }

  /* 東に一マス */
  if (temp_dir == g_direction.East) {
    q0 = (int)(*temp_x + 1U);
    if ((unsigned int)q0 > 255U) {
      q0 = 255;
    }

    *temp_x = (unsigned char)q0;

    /* disp("east_step") */
  }

  /* 南に一マス */
  if (temp_dir == g_direction.South) {
    q0 = *temp_y;
    qY = q0 - 1U;
    if (qY > (unsigned int)q0) {
      qY = 0U;
    }

    *temp_y = (unsigned char)qY;

    /* disp("south_step") */
  }

  /* 西に一マス */
  if (temp_dir == g_direction.West) {
    q0 = *temp_x;
    qY = q0 - 1U;
    if (qY > (unsigned int)q0) {
      qY = 0U;
    }

    *temp_x = (unsigned char)qY;

    /* disp("west_step") */
  }
}

/*
 * 直進時
 * Arguments    : unsigned char current_node[2]
 *                unsigned char *current_node_property
 *                unsigned char *current_move_dir
 *                unsigned char *current_move_mode
 *                unsigned char straight_count
 * Return Type  : void
 */
static void move_straight(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode, unsigned char straight_count)
{
  unsigned char u11;
  int q0;
  unsigned char temp_quotient;
  unsigned char temp_remainder;
  unsigned char temp_qr;
  unsigned int qY;

  /*     %% 移動用関数 */
  /*  直進 */
  if (*current_move_mode == move_dir_property.straight) {
    if (*current_move_dir == g_d_direction.North) {
      u11 = current_node[1];
      q0 = (int)((unsigned int)current_node[0] + straight_count);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      current_node[1] = u11;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.North;
      *current_move_mode = move_dir_property.straight;
    } else if (*current_move_dir == g_d_direction.East) {
      q0 = (int)((unsigned int)current_node[1] + straight_count);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.East;
      *current_move_mode = move_dir_property.straight;
    } else if (*current_move_dir == g_d_direction.South) {
      u11 = current_node[1];
      q0 = current_node[0];
      qY = (unsigned int)q0 - straight_count;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      current_node[1] = u11;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.South;
      *current_move_mode = move_dir_property.straight;
    } else {
      if (*current_move_dir == g_d_direction.West) {
        u11 = current_node[1];
        qY = (unsigned int)u11 - straight_count;
        if (qY > u11) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.West;
        *current_move_mode = move_dir_property.straight;
      }
    }

    /* 斜め直進のとき */
  } else {
    if (*current_move_mode == move_dir_property.diagonal) {
      /* 直進カウンタを2で割った商とあまり、その合計を計算 */
      /* (移動先ノード座標の一般化用) */
      temp_quotient = (unsigned char)trunc((double)straight_count / 2.0);
      temp_remainder = (unsigned char)(straight_count % 2);
      temp_qr = (unsigned char)(temp_quotient + temp_remainder);
      if (*current_move_dir == g_d_direction.North_East) {
        if (*current_node_property == matrix_dir.Row) {
          u11 = current_node[1];
          q0 = (int)((unsigned int)current_node[0] + temp_quotient);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          current_node[0] = (unsigned char)q0;
          q0 = (int)((unsigned int)u11 + temp_qr);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          current_node[1] = (unsigned char)q0;
          q0 = (int)((unsigned int)*current_node_property + temp_remainder);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          *current_node_property = (unsigned char)q0;
          *current_move_dir = g_d_direction.North_East;
          *current_move_mode = move_dir_property.diagonal;
        } else {
          if (*current_node_property == matrix_dir.Col) {
            u11 = current_node[1];
            q0 = (int)((unsigned int)current_node[0] + temp_qr);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            current_node[0] = (unsigned char)q0;
            q0 = (int)((unsigned int)u11 + temp_quotient);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            current_node[1] = (unsigned char)q0;
            q0 = *current_node_property;
            qY = (unsigned int)q0 - temp_remainder;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            *current_node_property = (unsigned char)qY;
            *current_move_dir = g_d_direction.North_East;
            *current_move_mode = move_dir_property.diagonal;
          }
        }
      } else if (*current_move_dir == g_d_direction.South_East) {
        if (*current_node_property == matrix_dir.Row) {
          u11 = current_node[1];
          q0 = current_node[0];
          qY = (unsigned int)q0 - temp_qr;
          if (qY > (unsigned int)q0) {
            qY = 0U;
          }

          current_node[0] = (unsigned char)qY;
          q0 = (int)((unsigned int)u11 + temp_qr);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          current_node[1] = (unsigned char)q0;
          q0 = (int)((unsigned int)*current_node_property + temp_remainder);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          *current_node_property = (unsigned char)q0;
          *current_move_dir = g_d_direction.South_East;
          *current_move_mode = move_dir_property.diagonal;
        } else {
          if (*current_node_property == matrix_dir.Col) {
            u11 = current_node[1];
            q0 = current_node[0];
            qY = (unsigned int)q0 - temp_quotient;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            current_node[0] = (unsigned char)qY;
            q0 = (int)((unsigned int)u11 + temp_quotient);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            current_node[1] = (unsigned char)q0;
            q0 = *current_node_property;
            qY = (unsigned int)q0 - temp_remainder;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            *current_node_property = (unsigned char)qY;
            *current_move_dir = g_d_direction.South_East;
            *current_move_mode = move_dir_property.diagonal;
          }
        }
      } else if (*current_move_dir == g_d_direction.South_West) {
        if (*current_node_property == matrix_dir.Row) {
          u11 = current_node[1];
          q0 = current_node[0];
          qY = (unsigned int)q0 - temp_qr;
          if (qY > (unsigned int)q0) {
            qY = 0U;
          }

          current_node[0] = (unsigned char)qY;
          qY = (unsigned int)u11 - temp_quotient;
          if (qY > u11) {
            qY = 0U;
          }

          current_node[1] = (unsigned char)qY;
          q0 = (int)((unsigned int)*current_node_property + temp_remainder);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          *current_node_property = (unsigned char)q0;
          *current_move_dir = g_d_direction.South_West;
          *current_move_mode = move_dir_property.diagonal;
        } else {
          if (*current_node_property == matrix_dir.Col) {
            u11 = current_node[1];
            q0 = current_node[0];
            qY = (unsigned int)q0 - temp_quotient;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            current_node[0] = (unsigned char)qY;
            qY = (unsigned int)u11 - temp_qr;
            if (qY > u11) {
              qY = 0U;
            }

            current_node[1] = (unsigned char)qY;
            q0 = *current_node_property;
            qY = (unsigned int)q0 - temp_remainder;
            if (qY > (unsigned int)q0) {
              qY = 0U;
            }

            *current_node_property = (unsigned char)qY;
            *current_move_dir = g_d_direction.South_West;
            *current_move_mode = move_dir_property.diagonal;
          }
        }
      } else {
        if (*current_move_dir == g_d_direction.North_West) {
          if (*current_node_property == matrix_dir.Row) {
            u11 = current_node[1];
            q0 = (int)((unsigned int)current_node[0] + temp_quotient);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            current_node[0] = (unsigned char)q0;
            qY = (unsigned int)u11 - temp_quotient;
            if (qY > u11) {
              qY = 0U;
            }

            current_node[1] = (unsigned char)qY;
            q0 = (int)((unsigned int)*current_node_property + temp_remainder);
            if ((unsigned int)q0 > 255U) {
              q0 = 255;
            }

            *current_node_property = (unsigned char)q0;
            *current_move_dir = g_d_direction.North_West;
            *current_move_mode = move_dir_property.diagonal;
          } else {
            if (*current_node_property == matrix_dir.Col) {
              u11 = current_node[1];
              q0 = (int)((unsigned int)current_node[0] + temp_qr);
              if ((unsigned int)q0 > 255U) {
                q0 = 255;
              }

              current_node[0] = (unsigned char)q0;
              qY = (unsigned int)u11 - temp_qr;
              if (qY > u11) {
                qY = 0U;
              }

              current_node[1] = (unsigned char)qY;
              q0 = *current_node_property;
              qY = (unsigned int)q0 - temp_remainder;
              if (qY > (unsigned int)q0) {
                qY = 0U;
              }

              *current_node_property = (unsigned char)qY;
              *current_move_dir = g_d_direction.North_West;
              *current_move_mode = move_dir_property.diagonal;
            }
          }
        }
      }
    }
  }

  /* 移動完了時、直進カウンタをクリア */
}

/*
 * 入力　現在位置x,y,現在方向,迷路行方向サイズ,迷路列方向サイズ,迷路壁情報,迷路壁の探索情報,ゴール座標
 * 出力  現在位置x,y,現在方向,壁情報,探索情報
 * Arguments    : const coder_internal_ref_6 *wall
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_5 *search
 *                const coder_internal_ref_2 *maze_goal
 *                const coder_internal_ref_1 *maze_wall
 *                const coder_internal_ref_4 *adachi_search_mode
 *                unsigned char *current_x
 *                unsigned char *current_y
 *                unsigned char *current_dir
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char b_maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 *                const unsigned char exploration_goal[18]
 *                unsigned char l_goal_size
 *                unsigned char *start_flg
 *                unsigned char adachi_s_mode
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void search_adachi(const coder_internal_ref_6 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_5 *search, const coder_internal_ref_2
  *maze_goal, const coder_internal_ref_1 *maze_wall, const coder_internal_ref_4 *
  adachi_search_mode, unsigned char *current_x, unsigned char *current_y,
  unsigned char *current_dir, unsigned char maze_row_size, unsigned char
  maze_col_size, unsigned char b_maze_wall[1024], unsigned char
  maze_wall_search[1024], const unsigned char exploration_goal[18], unsigned
  char l_goal_size, unsigned char *start_flg, unsigned char adachi_s_mode,
  unsigned short contour_map[1024])
{
  unsigned char straight_count;
  unsigned char goal_flg;
  unsigned char contour_flg;
  int i226;
  int exitg1;
  unsigned char temp_maze_wall_tmp;
  int i;
  bool exitg2;
  unsigned char error_flg;
  unsigned int qY;

  /*     %% search_adachi 足立法での探索 */
  /* local変数宣言 */
  straight_count = 0U;
  goal_flg = 0U;

  /* ゴール判定フラグ */
  /* 壁情報更新確認用変数 */
  contour_flg = 0U;

  /*      search_start_x = current_x %探索開始時x */
  /*      search_start_y = current_y %探索開始時y */
  /* 初回のコンターマップ作製 */
  make_map_find(wall, exploration_goal, l_goal_size, b_maze_wall, *current_x,
                *current_y, contour_map);
  i226 = l_goal_size;
  do {
    exitg1 = 0;

    /* 壁情報取得 */
    /* ゴール直後は壁情報を更新しない */
    /* 既知区間加速時でなければ、壁情報を取得しない */
    if (straight_count == 0) {
      temp_maze_wall_tmp = b_maze_wall[(*current_y + ((*current_x - 1) << 5)) -
        1];
      wall_set(wall, wall_flg, search, maze_goal, maze_row_size, maze_col_size, *
               current_x, *current_y, *current_dir, b_maze_wall,
               maze_wall_search);

      /* 壁情報が更新されれば、コンター更新のフラグを立てる。 */
      contour_flg = (unsigned char)(temp_maze_wall_tmp != b_maze_wall
        [(*current_y + ((*current_x - 1) << 5)) - 1]);
    }

    /*  等高線MAP生成 */
    /* 壁情報に変更があった場合のみ */
    if (contour_flg != 0) {
      make_map_find(wall, exploration_goal, l_goal_size, b_maze_wall, *current_x,
                    *current_y, contour_map);
    }

    /* 現在位置がゴールか判定 */
    for (i = 0; i < i226; i++) {
      if ((*current_x == exploration_goal[i]) && (*current_y ==
           exploration_goal[i + 9])) {
        goal_flg = 1U;
      }
    }

    /* 探索モードの場合、対象のマスがすべて探索済みのとき、ゴールフラグを立てる */
    if (adachi_s_mode == adachi_search_mode->contents.search) {
      goal_flg = 1U;
      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i <= l_goal_size - 1)) {
        /* ゴール座標が未探索であれば、フラグをおろし、ブレイク */
        if (maze_wall_search[(exploration_goal[i + 9] + ((exploration_goal[i] -
               1) << 5)) - 1] != 15) {
          goal_flg = 0U;
          exitg2 = true;
        } else {
          i++;
        }
      }
    }

    /* ゴール時処理 */
    if (goal_flg == 1) {
      exitg1 = 1;
    } else {
      /*  進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      get_nextdir2(*current_x, *current_y, b_maze_wall, contour_map,
                   &temp_maze_wall_tmp, &error_flg);

      /* 進行方向が見つからなかったときのエラー処理 */
      if (error_flg == 1) {
        m_error_movement(1);
      }

      /*  現在方向と進行方向に応じた処理 */
      i = (int)(4U + temp_maze_wall_tmp);
      if ((unsigned int)i > 255U) {
        i = 255;
      }

      qY = (unsigned int)i - *current_dir;
      if (qY > (unsigned int)i) {
        qY = 0U;
      }

      temp_maze_wall_tmp = b_rem((unsigned char)qY);
      if (l_direction.front == temp_maze_wall_tmp) {
        i = 0;
      } else if (l_direction.right == temp_maze_wall_tmp) {
        i = 1;
      } else if (l_direction.back == temp_maze_wall_tmp) {
        i = 2;
      } else if (l_direction.left == temp_maze_wall_tmp) {
        i = 3;
      } else {
        i = -1;
      }

      switch (i) {
       case 0:
        /* 直進制御時、壁フラグを現在参照位置で更新 */
        if (straight_count > 0) {
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* 参照位置を次回の参照位置に変更 */
        move_step(current_x, current_y, *current_dir);

        /* 移動先が探索済みであるなら、ストレートカウンタをインクリメント */
        if (maze_wall_search[(*current_y + ((*current_x - 1) << 5)) - 1] == 15)
        {
          i = (int)(straight_count + 1U);
          if ((unsigned int)i > 255U) {
            i = 255;
          }

          straight_count = (unsigned char)i;

          /* 探索済みでない場合 */
        } else {
          /* 直線カウンタがある場合、移動 */
          /* 現在方向も直進なので、直進カウンタ+1だけ移動する。 */
          if (straight_count > 0) {
            i = (int)(straight_count + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            m_move_front_long((unsigned char)i, *start_flg, wall_flg->contents,
                              move_dir_property.straight);
            straight_count = 0U;

            /* スタート直後フラグをクリア */
            *start_flg = 0U;

            /* 壁フラグをクリア */
            wall_flg->contents = 0U;
          } else {
            /* 直進カウンタがない場合 */
            m_move_front(*start_flg, wall_flg->contents,
                         move_dir_property.straight);

            /* スタート直後フラグをクリア */
            *start_flg = 0U;

            /* 壁フラグをクリア */
            wall_flg->contents = 0U;
          }
        }
        break;

       case 1:
        /* 直進カウンタがある場合、移動 */
        if (straight_count > 0) {
          /* 前回位置での壁フラグを使用し、マウスを移動させる（C) */
          m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                            move_dir_property.straight);
          straight_count = 0U;

          /* スタート直後フラグをクリア */
          *start_flg = 0U;

          /* 壁フラグをクリア */
          wall_flg->contents = 0U;

          /* 現在参照位置で壁フラグを更新 */
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* マウスを移動させる（C) */
        m_move_right(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* 参照位置、方向を更新 */
        turn_clk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 2:
        /* disp("back") */
        /* 直進カウンタがある場合、移動 */
        if (straight_count > 0) {
          /* 前回位置での壁フラグを使用し、マウスを移動させる（C) */
          m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                            move_dir_property.straight);
          straight_count = 0U;

          /* スタート直後フラグをクリア */
          *start_flg = 0U;

          /* 壁フラグをクリア */
          wall_flg->contents = 0U;

          /* 現在参照位置で壁フラグを更新 */
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* マウスを移動させる（C） */
        m_move_back(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* 参照位置、方向を更新 */
        turn_180deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 3:
        /* 直進カウンタがある場合、移動 */
        if (straight_count > 0) {
          /* 前回位置での壁フラグを使用し、マウスを移動させる（C) */
          m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                            move_dir_property.straight);
          straight_count = 0U;

          /* スタート直後フラグをクリア */
          *start_flg = 0U;

          /* 壁フラグをクリア */
          wall_flg->contents = 0U;

          /* 現在参照位置で壁フラグを更新 */
          wall_flg->contents = fust_run_wallset(maze_wall, *current_y,
            *current_x, *current_dir);
        }

        /* マウスを移動させる（C） */
        m_move_left(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* 参照位置、方向を更新 */
        turn_conclk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;
      }

      /* for code generation */
    }
  } while (exitg1 == 0);

  /* 直進カウンタ分移動 */
  if (straight_count > 0) {
    m_move_front_long(straight_count, *start_flg, wall_flg->contents,
                      move_dir_property.straight);

    /* スタート直後フラグをクリア */
    *start_flg = 0U;

    /* 壁フラグをクリア */
    wall_flg->contents = 0U;
  }

  /* ゴール時停止フラグが立っているとき */
  /* 停止動作を実施 */
  m_goal_movement(*start_flg, wall_flg->contents, move_dir_property.straight);

  /* ゴール時停止フラグが立っていなければ、動作させたまま終了 */
}

/*
 * Arguments    : const unsigned char temp_unexp_square[1024]
 *                unsigned char temp_unexp_square_idx
 *                unsigned char temp_y
 *                unsigned char temp_x
 * Return Type  : unsigned char
 */
static unsigned char sh_route_unexp_sq_jud(const unsigned char
  temp_unexp_square[1024], unsigned char temp_unexp_square_idx, unsigned char
  temp_y, unsigned char temp_x)
{
  unsigned char result;
  int i;
  bool exitg1;

  /* 最短経路未探索マス合致判定関数(入れ子) */
  /* 出力:判定結果　0,合致なし　1,合致あり */
  result = 0U;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= temp_unexp_square_idx - 1)) {
    /* 最短経路の未探索のマスと入力座標が一致すればフラグを立ててブレイク */
    if ((temp_unexp_square[i] == temp_y) && (temp_unexp_square[i + 512] ==
         temp_x)) {
      result = 1U;
      exitg1 = true;
    } else {
      i++;
    }
  }

  return result;
}

/*
 * 入力 現在方向
 * 出力 現在方向
 * Arguments    : unsigned char *current_dir
 * Return Type  : void
 */
static void turn_180deg(unsigned char *current_dir)
{
  int i320;

  /*     %% turn_180deg 180度ターンする関数 */
  i320 = (int)(4U + *current_dir);
  if ((unsigned int)i320 > 255U) {
    i320 = 255;
  }

  *current_dir = (unsigned char)((i320 - 2) % 4);
}

/*
 * 入力 現在方向
 * 出力 現在方向
 * Arguments    : unsigned char *current_dir
 * Return Type  : void
 */
static void turn_clk_90deg(unsigned char *current_dir)
{
  int i319;

  /*     %% turn_clk_90deg 時計周りに90度ターンする関数 */
  i319 = (int)(4U + *current_dir);
  if ((unsigned int)i319 > 255U) {
    i319 = 255;
  }

  i319++;
  if ((unsigned int)i319 > 255U) {
    i319 = 255;
  }

  *current_dir = (unsigned char)(i319 % 4);
}

/*
 * 入力　現在方向
 * 出力　現在方向
 * Arguments    : unsigned char *current_dir
 * Return Type  : void
 */
static void turn_conclk_90deg(unsigned char *current_dir)
{
  int i321;

  /*     %% turn_conclk_90deg 反時計周りに90度回る関数 */
  i321 = (int)(4U + *current_dir);
  if ((unsigned int)i321 > 255U) {
    i321 = 255;
  }

  *current_dir = (unsigned char)((i321 - 1) % 4);
}

/*
 * 直進パターンの時
 * Arguments    : unsigned char current_node[2]
 *                unsigned char *current_node_property
 *                unsigned char *current_move_dir
 *                unsigned char *current_move_mode
 * Return Type  : void
 */
static void turn_l_135(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode)
{
  unsigned char u16;
  int q0;
  unsigned int qY;

  /*  左135度 */
  if (*current_move_mode == move_dir_property.straight) {
    if (*current_move_dir == g_d_direction.North) {
      u16 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      qY = u16 - 1U;
      if (qY > u16) {
        qY = 0U;
      }

      current_node[1] = (unsigned char)qY;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.South_West;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.East) {
      u16 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      q0 = (int)(u16 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.North_West;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.South) {
      u16 = current_node[1];
      q0 = current_node[0];
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      q0 = (int)(u16 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.North_East;
      *current_move_mode = move_dir_property.diagonal;
    } else {
      if (*current_move_dir == g_d_direction.West) {
        u16 = current_node[1];
        q0 = current_node[0];
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        current_node[0] = (unsigned char)qY;
        qY = u16 - 1U;
        if (qY > u16) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.South_East;
        *current_move_mode = move_dir_property.diagonal;
      }
    }

    /* 斜めパターンの時 */
  } else {
    if (*current_move_mode == move_dir_property.diagonal) {
      if (*current_move_dir == g_d_direction.North_East) {
        u16 = current_node[1];
        q0 = (int)(current_node[0] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[0] = (unsigned char)q0;
        current_node[1] = u16;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.West;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_East) {
        q0 = (int)(current_node[1] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[1] = (unsigned char)q0;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.North;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_West) {
        u16 = current_node[1];
        q0 = current_node[0];
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        current_node[0] = (unsigned char)qY;
        current_node[1] = u16;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.East;
        *current_move_mode = move_dir_property.straight;
      } else {
        if (*current_move_dir == g_d_direction.North_West) {
          u16 = current_node[1];
          qY = u16 - 1U;
          if (qY > u16) {
            qY = 0U;
          }

          current_node[1] = (unsigned char)qY;
          *current_node_property = matrix_dir.Row;
          *current_move_dir = g_d_direction.South;
          *current_move_mode = move_dir_property.straight;
        }
      }
    }
  }
}

/*
 * 直進パターンの時
 * Arguments    : unsigned char current_node[2]
 *                unsigned char *current_node_property
 *                unsigned char *current_move_dir
 *                unsigned char *current_move_mode
 * Return Type  : void
 */
static void turn_l_180(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode)
{
  unsigned char u17;
  int q0;
  unsigned int qY;

  /*  左180度 */
  if (*current_move_mode == move_dir_property.straight) {
    if (*current_move_dir == g_d_direction.North) {
      u17 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      qY = u17 - 1U;
      if (qY > u17) {
        qY = 0U;
      }

      current_node[1] = (unsigned char)qY;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.South;
      *current_move_mode = move_dir_property.straight;
    } else if (*current_move_dir == g_d_direction.East) {
      u17 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      q0 = (int)(u17 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.West;
      *current_move_mode = move_dir_property.straight;
    } else if (*current_move_dir == g_d_direction.South) {
      u17 = current_node[1];
      q0 = current_node[0];
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      q0 = (int)(u17 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.North;
      *current_move_mode = move_dir_property.straight;
    } else {
      if (*current_move_dir == g_d_direction.West) {
        u17 = current_node[1];
        q0 = current_node[0];
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        current_node[0] = (unsigned char)qY;
        qY = u17 - 1U;
        if (qY > u17) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.East;
        *current_move_mode = move_dir_property.straight;
      }
    }

    /* 斜めパターンの時 */
  }
}

/*
 * 直進パターンの時
 * Arguments    : unsigned char current_node[2]
 *                unsigned char *current_node_property
 *                unsigned char *current_move_dir
 *                unsigned char *current_move_mode
 * Return Type  : void
 */
static void turn_l_45(unsigned char current_node[2], unsigned char
                      *current_node_property, unsigned char *current_move_dir,
                      unsigned char *current_move_mode)
{
  unsigned char u13;
  int q0;
  unsigned int qY;

  /*  左45度 */
  if (*current_move_mode == move_dir_property.straight) {
    if (*current_move_dir == g_d_direction.North) {
      u13 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      current_node[1] = u13;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.North_West;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.East) {
      u13 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      q0 = (int)(u13 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.North_East;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.South) {
      u13 = current_node[1];
      q0 = current_node[0];
      qY = q0 - 2U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      q0 = (int)(u13 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.South_East;
      *current_move_mode = move_dir_property.diagonal;
    } else {
      if (*current_move_dir == g_d_direction.West) {
        u13 = current_node[1];
        qY = u13 - 2U;
        if (qY > u13) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.South_West;
        *current_move_mode = move_dir_property.diagonal;
      }
    }
  } else {
    if (*current_move_mode == move_dir_property.diagonal) {
      if (*current_move_dir == g_d_direction.North_East) {
        u13 = current_node[1];
        q0 = (int)(current_node[0] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[0] = (unsigned char)q0;
        current_node[1] = u13;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.North;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_East) {
        u13 = current_node[1];
        q0 = current_node[0];
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        current_node[0] = (unsigned char)qY;
        q0 = (int)(u13 + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[1] = (unsigned char)q0;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.East;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_West) {
        u13 = current_node[1];
        qY = u13 - 1U;
        if (qY > u13) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.South;
        *current_move_mode = move_dir_property.straight;
      } else {
        if (*current_move_dir == g_d_direction.North_West) {
          *current_node_property = matrix_dir.Col;
          *current_move_dir = g_d_direction.West;
          *current_move_mode = move_dir_property.straight;
        }
      }
    }
  }
}

/*
 * 直進パターンの時
 * Arguments    : unsigned char current_node[2]
 *                unsigned char *current_node_property
 *                unsigned char *current_move_dir
 *                unsigned char *current_move_mode
 * Return Type  : void
 */
static void turn_l_90(unsigned char current_node[2], unsigned char
                      *current_node_property, unsigned char *current_move_dir,
                      unsigned char *current_move_mode)
{
  unsigned char u14;
  int q0;
  unsigned int qY;

  /*  左90度 */
  if (*current_move_mode == move_dir_property.straight) {
    if (*current_move_dir == g_d_direction.North) {
      u14 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      current_node[1] = u14;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.West;
      *current_move_mode = move_dir_property.straight;
    } else if (*current_move_dir == g_d_direction.East) {
      u14 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      q0 = (int)(u14 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.North;
      *current_move_mode = move_dir_property.straight;
    } else if (*current_move_dir == g_d_direction.South) {
      u14 = current_node[1];
      q0 = current_node[0];
      qY = q0 - 2U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      q0 = (int)(u14 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.East;
      *current_move_mode = move_dir_property.straight;
    } else {
      if (*current_move_dir == g_d_direction.West) {
        u14 = current_node[1];
        qY = u14 - 2U;
        if (qY > u14) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.South;
        *current_move_mode = move_dir_property.straight;
      }
    }

    /* 斜めパターンの時（V90） */
  } else {
    if (*current_move_mode == move_dir_property.diagonal) {
      if (*current_move_dir == g_d_direction.North_East) {
        u14 = current_node[1];
        q0 = (int)(current_node[0] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[0] = (unsigned char)q0;
        current_node[1] = u14;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.North_West;
        *current_move_mode = move_dir_property.diagonal;
      } else if (*current_move_dir == g_d_direction.South_East) {
        q0 = (int)(current_node[1] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[1] = (unsigned char)q0;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.North_East;
        *current_move_mode = move_dir_property.diagonal;
      } else if (*current_move_dir == g_d_direction.South_West) {
        u14 = current_node[1];
        q0 = current_node[0];
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        current_node[0] = (unsigned char)qY;
        current_node[1] = u14;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.South_East;
        *current_move_mode = move_dir_property.diagonal;
      } else {
        if (*current_move_dir == g_d_direction.North_West) {
          u14 = current_node[1];
          qY = u14 - 1U;
          if (qY > u14) {
            qY = 0U;
          }

          current_node[1] = (unsigned char)qY;
          *current_node_property = matrix_dir.Row;
          *current_move_dir = g_d_direction.South_West;
          *current_move_mode = move_dir_property.diagonal;
        }
      }
    }
  }
}

/*
 * 直進パターンの時
 * Arguments    : unsigned char current_node[2]
 *                unsigned char *current_node_property
 *                unsigned char *current_move_dir
 *                unsigned char *current_move_mode
 * Return Type  : void
 */
static void turn_r_135(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode)
{
  unsigned char u15;
  int q0;
  unsigned int qY;

  /*  右135度 */
  if (*current_move_mode == move_dir_property.straight) {
    if (*current_move_dir == g_d_direction.North) {
      u15 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      q0 = (int)(u15 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.South_East;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.East) {
      u15 = current_node[1];
      q0 = current_node[0];
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      q0 = (int)(u15 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.South_West;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.South) {
      u15 = current_node[1];
      q0 = current_node[0];
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      qY = u15 - 1U;
      if (qY > u15) {
        qY = 0U;
      }

      current_node[1] = (unsigned char)qY;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.North_West;
      *current_move_mode = move_dir_property.diagonal;
    } else {
      if (*current_move_dir == g_d_direction.West) {
        u15 = current_node[1];
        q0 = (int)(current_node[0] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[0] = (unsigned char)q0;
        qY = u15 - 1U;
        if (qY > u15) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.North_East;
        *current_move_mode = move_dir_property.diagonal;
      }
    }

    /* 斜めパターンの時 */
  } else {
    if (*current_move_mode == move_dir_property.diagonal) {
      if (*current_move_dir == g_d_direction.North_East) {
        q0 = (int)(current_node[1] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[1] = (unsigned char)q0;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.South;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_East) {
        u15 = current_node[1];
        q0 = current_node[0];
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        current_node[0] = (unsigned char)qY;
        current_node[1] = u15;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.West;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_West) {
        u15 = current_node[1];
        qY = u15 - 1U;
        if (qY > u15) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.North;
        *current_move_mode = move_dir_property.straight;
      } else {
        if (*current_move_dir == g_d_direction.North_West) {
          u15 = current_node[1];
          q0 = (int)(current_node[0] + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          current_node[0] = (unsigned char)q0;
          current_node[1] = u15;
          *current_node_property = matrix_dir.Col;
          *current_move_dir = g_d_direction.East;
          *current_move_mode = move_dir_property.straight;
        }
      }
    }
  }
}

/*
 * 直進パターンの時
 * Arguments    : unsigned char current_node[2]
 *                unsigned char *current_node_property
 *                unsigned char *current_move_dir
 *                unsigned char *current_move_mode
 * Return Type  : void
 */
static void turn_r_45(unsigned char current_node[2], unsigned char
                      *current_node_property, unsigned char *current_move_dir,
                      unsigned char *current_move_mode)
{
  unsigned char u12;
  int q0;
  unsigned int qY;

  /*  右45度 */
  if (*current_move_mode == move_dir_property.straight) {
    if (*current_move_dir == g_d_direction.North) {
      u12 = current_node[1];
      q0 = (int)(current_node[0] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[0] = (unsigned char)q0;
      q0 = (int)(u12 + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.North_East;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.East) {
      q0 = (int)(current_node[1] + 1U);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      current_node[1] = (unsigned char)q0;
      *current_node_property = matrix_dir.Row;
      *current_move_dir = g_d_direction.South_East;
      *current_move_mode = move_dir_property.diagonal;
    } else if (*current_move_dir == g_d_direction.South) {
      u12 = current_node[1];
      q0 = current_node[0];
      qY = q0 - 2U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      current_node[0] = (unsigned char)qY;
      current_node[1] = u12;
      *current_node_property = matrix_dir.Col;
      *current_move_dir = g_d_direction.South_West;
      *current_move_mode = move_dir_property.diagonal;
    } else {
      if (*current_move_dir == g_d_direction.West) {
        u12 = current_node[1];
        q0 = (int)(current_node[0] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[0] = (unsigned char)q0;
        qY = u12 - 2U;
        if (qY > u12) {
          qY = 0U;
        }

        current_node[1] = (unsigned char)qY;
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.North_West;
        *current_move_mode = move_dir_property.diagonal;
      }
    }
  } else {
    if (*current_move_mode == move_dir_property.diagonal) {
      if (*current_move_dir == g_d_direction.North_East) {
        q0 = (int)(current_node[1] + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_node[1] = (unsigned char)q0;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.East;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_East) {
        *current_node_property = matrix_dir.Row;
        *current_move_dir = g_d_direction.South;
        *current_move_mode = move_dir_property.straight;
      } else if (*current_move_dir == g_d_direction.South_West) {
        u12 = current_node[1];
        q0 = current_node[0];
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        current_node[0] = (unsigned char)qY;
        current_node[1] = u12;
        *current_node_property = matrix_dir.Col;
        *current_move_dir = g_d_direction.West;
        *current_move_mode = move_dir_property.straight;
      } else {
        if (*current_move_dir == g_d_direction.North_West) {
          u12 = current_node[1];
          q0 = (int)(current_node[0] + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          current_node[0] = (unsigned char)q0;
          qY = u12 - 1U;
          if (qY > u12) {
            qY = 0U;
          }

          current_node[1] = (unsigned char)qY;
          *current_node_property = matrix_dir.Row;
          *current_move_dir = g_d_direction.North;
          *current_move_mode = move_dir_property.straight;
        }
      }
    }
  }
}

/*
 * matlab上では画像から取得した壁情報を参照する。
 * 入力:画像から得た迷路情報,迷路行方向壁枚数,迷路列方向壁枚数,
 *      現在地座標x,y,現在進行方向,迷路壁情報,迷路壁探索情報
 * 出力:迷路壁情報,迷路壁探索情報
 * Arguments    : const coder_internal_ref_6 *wall
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_5 *search
 *                const coder_internal_ref_2 *maze_goal
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned char current_dir
 *                unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 * Return Type  : void
 */
static void wall_set(const coder_internal_ref_6 *wall, coder_internal_ref
                     *wall_flg, const coder_internal_ref_5 *search, const
                     coder_internal_ref_2 *maze_goal, unsigned char
                     maze_row_size, unsigned char maze_col_size, unsigned char
                     current_x, unsigned char current_y, unsigned char
                     current_dir, unsigned char maze_wall[1024], unsigned char
                     maze_wall_search[1024])
{
  unsigned char wall_write[4];
  unsigned char serch_write[4];
  short wall_sensor_front;
  int b_current_dir;
  int i227;
  unsigned char k;
  int b_k;
  int c_k;
  int ex;
  int i228;
  int i229;
  int d_k;
  int i230;
  int i231;
  int i232;
  int i233;
  int i234;
  int i235;
  int i236;
  int i237;
  unsigned int qY;
  int i238;
  int i239;
  int i240;
  int i241;
  signed char varargin_1[9];
  int i242;
  int i243;
  int i244;
  int i245;
  int i246;
  int i247;
  int i248;
  int i249;
  int i250;
  int i251;
  int i252;
  int i253;
  int i254;
  int i255;
  int i256;
  int i257;
  int i258;
  int i259;
  int i260;
  int i261;
  int i262;
  int i263;
  int i264;
  int i265;
  int i266;
  int i267;
  int i268;
  int i269;
  int i270;
  int i271;
  int i272;
  int i273;
  int i274;
  int i275;
  unsigned int b_qY;
  int i276;
  int i277;
  int i278;
  int i279;
  int i280;
  int i281;
  int i282;
  int i283;
  int i284;
  int i285;
  int i286;
  int i287;
  int i288;
  int i289;
  int i290;
  int i291;
  int i292;
  int i293;
  int i294;
  int i295;
  int i296;
  int i297;
  int i298;
  int i299;
  int i300;
  int i301;
  int i302;
  int i303;
  int i304;
  int i305;
  int i306;
  int i307;
  int i308;
  unsigned int c_qY;
  unsigned int d_qY;
  int i309;
  int i310;
  int i311;
  int i312;
  int i313;
  int i314;
  int i315;
  int i316;
  int i317;
  int i318;

  /*     %%  wall_set 壁情報取得 */
  /* グローバル変数(matlabでは迷路データを、Cでは壁センサ値を参照) */
  /* for matlab */
  /* for C gen */
  /* 壁センサ閾値 */
  /* ローカル変数宣言 */
  /* 壁情報書き込み用バッファ(N,E,S,W) */
  wall_write[0] = 0U;
  serch_write[0] = 0U;
  wall_write[1] = 0U;
  serch_write[1] = 0U;
  wall_write[2] = 0U;
  serch_write[2] = 0U;
  wall_write[3] = 0U;
  serch_write[3] = 0U;

  /* 探索情報書き込み用バッファ(N,E,S,W) */
  /* 壁センサAD値格納変数 */
  /* マウスの方向に基づく壁情報取得 */
  /* マウスの方向と絶対方向の差=マウスの方向となることを利用し、 */
  /* 絶対角度と整合をとる。 */
  /* 前方の壁判定 */
  /* for Cgen */
  /* センサ値取得 */
  wall_sensor_front = m_get_front_sensor();

  /* センサ値をもとに、壁の有無を判定 */
  if (wall_sensor_front > wall_sensor_front_th) {
    /* 壁情報取得 */
    i227 = (int)(b_rem(current_dir) + 1U);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    wall_write[i227 - 1] = wall->contents.wall;

    /* 壁フラグセット */
    wall_flg->contents |= 1;
  } else {
    if (current_dir <= 7) {
      b_current_dir = (unsigned char)(1 << current_dir);
    } else {
      b_current_dir = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & b_current_dir) !=
        0) {
      /* すでに壁情報がある場合も壁フラグを立てる。%見落としたとき用 */
      /* 壁フラグセット */
      wall_flg->contents |= 1;
    }
  }

  /* 探索情報取更新 */
  i227 = (int)(b_rem(current_dir) + 1U);
  if ((unsigned int)i227 > 255U) {
    i227 = 255;
  }

  serch_write[i227 - 1] = search->contents.known;

  /* 右壁判定 */
  /* for Cgen */
  /* センサ値取得 */
  wall_sensor_front = m_get_right_sensor();

  /* センサ値をもとに、壁の有無を判定 */
  if (wall_sensor_front > wall_sensor_right_th) {
    /* 壁情報取得 */
    i227 = (int)(current_dir + 1U);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    i227 = (int)(b_rem((unsigned char)i227) + 1U);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    wall_write[i227 - 1] = wall->contents.wall;

    /* 壁フラグセット */
    wall_flg->contents = (unsigned char)(wall_flg->contents | 2);
  } else {
    i227 = (int)(current_dir + 1U);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    k = b_rem((unsigned char)i227);
    if (k <= 7) {
      b_k = (unsigned char)(1 << k);
    } else {
      b_k = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & b_k) != 0) {
      /* すでに壁情報がある場合も壁フラグを立てる。%見落としたとき用 */
      /* 壁フラグセット */
      wall_flg->contents = (unsigned char)(wall_flg->contents | 2);
    }
  }

  /* 探索情報取更新 */
  i227 = (int)(current_dir + 1U);
  if ((unsigned int)i227 > 255U) {
    i227 = 255;
  }

  i227 = (int)(b_rem((unsigned char)i227) + 1U);
  if ((unsigned int)i227 > 255U) {
    i227 = 255;
  }

  serch_write[i227 - 1] = search->contents.known;

  /* 後方は情報を得ることができないので処理しない。 */
  /* 左壁判定 */
  /* for Cgen */
  /* センサ値取得 */
  wall_sensor_front = m_get_left_sensor();

  /* センサ値をもとに、壁の有無を判定 */
  if (wall_sensor_front > wall_sensor_left_th) {
    /* 壁情報取得 */
    i227 = (int)(current_dir + 3U);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    i227 = (int)(b_rem((unsigned char)i227) + 1U);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    wall_write[i227 - 1] = wall->contents.wall;

    /* 壁フラグセット */
    wall_flg->contents = (unsigned char)(wall_flg->contents | 8);
  } else {
    i227 = (int)(current_dir + 3U);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    k = b_rem((unsigned char)i227);
    if (k <= 7) {
      c_k = (unsigned char)(1 << k);
    } else {
      c_k = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & c_k) != 0) {
      /* すでに壁情報がある場合も壁フラグを立てる。%見落としたとき用 */
      /* 壁フラグセット */
      wall_flg->contents = (unsigned char)(wall_flg->contents | 8);
    }
  }

  /* 探索情報取更新 */
  i227 = (int)(current_dir + 3U);
  if ((unsigned int)i227 > 255U) {
    i227 = 255;
  }

  i227 = (int)(b_rem((unsigned char)i227) + 1U);
  if ((unsigned int)i227 > 255U) {
    i227 = 255;
  }

  serch_write[i227 - 1] = search->contents.known;

  /* ここまで */
  /* 壁情報,探索情報を入力 */
  /* 北側 */
  i227 = (int)(g_direction.North + 1U);
  ex = i227;
  if ((unsigned int)i227 > 255U) {
    ex = 255;
  }

  if (g_direction.North <= 7) {
    i228 = (unsigned char)(1 << g_direction.North);
  } else {
    i228 = 0;
  }

  ex = (int)((unsigned int)i228 * wall_write[ex - 1]);
  if ((unsigned int)ex > 255U) {
    ex = 255;
  }

  maze_wall[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall[(current_y + ((current_x - 1) << 5)) - 1] | ex);
  ex = i227;
  if ((unsigned int)i227 > 255U) {
    ex = 255;
  }

  if (g_direction.North <= 7) {
    i229 = (unsigned char)(1 << g_direction.North);
  } else {
    i229 = 0;
  }

  ex = (int)((unsigned int)i229 * serch_write[ex - 1]);
  if ((unsigned int)ex > 255U) {
    ex = 255;
  }

  maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] | ex);

  /* 東側 */
  ex = (int)(g_direction.East + 1U);
  d_k = ex;
  if ((unsigned int)ex > 255U) {
    d_k = 255;
  }

  if (g_direction.East <= 7) {
    i230 = (unsigned char)(1 << g_direction.East);
  } else {
    i230 = 0;
  }

  d_k = (int)((unsigned int)i230 * wall_write[d_k - 1]);
  if ((unsigned int)d_k > 255U) {
    d_k = 255;
  }

  maze_wall[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall[(current_y + ((current_x - 1) << 5)) - 1] | d_k);
  d_k = ex;
  if ((unsigned int)ex > 255U) {
    d_k = 255;
  }

  if (g_direction.East <= 7) {
    i231 = (unsigned char)(1 << g_direction.East);
  } else {
    i231 = 0;
  }

  d_k = (int)((unsigned int)i231 * serch_write[d_k - 1]);
  if ((unsigned int)d_k > 255U) {
    d_k = 255;
  }

  maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] | d_k);

  /* 南側 */
  d_k = (int)(g_direction.South + 1U);
  i232 = d_k;
  if ((unsigned int)d_k > 255U) {
    i232 = 255;
  }

  if (g_direction.South <= 7) {
    i233 = (unsigned char)(1 << g_direction.South);
  } else {
    i233 = 0;
  }

  i232 = (int)((unsigned int)i233 * wall_write[i232 - 1]);
  if ((unsigned int)i232 > 255U) {
    i232 = 255;
  }

  maze_wall[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall[(current_y + ((current_x - 1) << 5)) - 1] | i232);
  i232 = d_k;
  if ((unsigned int)d_k > 255U) {
    i232 = 255;
  }

  if (g_direction.South <= 7) {
    i234 = (unsigned char)(1 << g_direction.South);
  } else {
    i234 = 0;
  }

  i232 = (int)((unsigned int)i234 * serch_write[i232 - 1]);
  if ((unsigned int)i232 > 255U) {
    i232 = 255;
  }

  maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] | i232);

  /* 西側 */
  i232 = (int)(g_direction.West + 1U);
  i235 = i232;
  if ((unsigned int)i232 > 255U) {
    i235 = 255;
  }

  if (g_direction.West <= 7) {
    i236 = (unsigned char)(1 << g_direction.West);
  } else {
    i236 = 0;
  }

  i235 = (int)((unsigned int)i236 * wall_write[i235 - 1]);
  if ((unsigned int)i235 > 255U) {
    i235 = 255;
  }

  maze_wall[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall[(current_y + ((current_x - 1) << 5)) - 1] | i235);
  i235 = i232;
  if ((unsigned int)i232 > 255U) {
    i235 = 255;
  }

  if (g_direction.West <= 7) {
    i237 = (unsigned char)(1 << g_direction.West);
  } else {
    i237 = 0;
  }

  i235 = (int)((unsigned int)i237 * serch_write[i235 - 1]);
  if ((unsigned int)i235 > 255U) {
    i235 = 255;
  }

  maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] = (unsigned char)
    (maze_wall_search[(current_y + ((current_x - 1) << 5)) - 1] | i235);

  /* 隣り合うマスの情報にも入力 */
  /* 北側のマスの南側の壁情報 */
  qY = maze_row_size - 1U;
  if (qY > maze_row_size) {
    qY = 0U;
  }

  if (current_y < (int)qY) {
    i235 = i227;
    if ((unsigned int)i227 > 255U) {
      i235 = 255;
    }

    if (g_direction.South <= 7) {
      i238 = (unsigned char)(1 << g_direction.South);
    } else {
      i238 = 0;
    }

    i235 = (int)((unsigned int)i238 * wall_write[i235 - 1]);
    if ((unsigned int)i235 > 255U) {
      i235 = 255;
    }

    maze_wall[current_y + ((current_x - 1) << 5)] = (unsigned char)
      (maze_wall[current_y + ((current_x - 1) << 5)] | i235);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    if (g_direction.South <= 7) {
      i241 = (unsigned char)(1 << g_direction.South);
    } else {
      i241 = 0;
    }

    i227 = (int)((unsigned int)i241 * serch_write[i227 - 1]);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    maze_wall_search[current_y + ((current_x - 1) << 5)] = (unsigned char)
      (maze_wall_search[current_y + ((current_x - 1) << 5)] | i227);
  }

  /* 東側のマスの西側の壁情報 */
  qY = maze_col_size - 1U;
  if (qY > maze_col_size) {
    qY = 0U;
  }

  if (current_x < (int)qY) {
    i227 = ex;
    if ((unsigned int)ex > 255U) {
      i227 = 255;
    }

    if (g_direction.West <= 7) {
      i239 = (unsigned char)(1 << g_direction.West);
    } else {
      i239 = 0;
    }

    i227 = (int)((unsigned int)i239 * wall_write[i227 - 1]);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    i235 = (current_y + (current_x << 5)) - 1;
    maze_wall[i235] = (unsigned char)(maze_wall[i235] | i227);
    if ((unsigned int)ex > 255U) {
      ex = 255;
    }

    if (g_direction.West <= 7) {
      i243 = (unsigned char)(1 << g_direction.West);
    } else {
      i243 = 0;
    }

    i227 = (int)((unsigned int)i243 * serch_write[ex - 1]);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    maze_wall_search[i235] = (unsigned char)(maze_wall_search[i235] | i227);
  }

  /* 南側のマスの北側の壁情報 */
  if (current_y > 1) {
    i227 = d_k;
    if ((unsigned int)d_k > 255U) {
      i227 = 255;
    }

    if (g_direction.North <= 7) {
      i240 = (unsigned char)(1 << g_direction.North);
    } else {
      i240 = 0;
    }

    i227 = (int)((unsigned int)i240 * wall_write[i227 - 1]);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    ex = (current_y + ((current_x - 1) << 5)) - 2;
    maze_wall[ex] = (unsigned char)(maze_wall[ex] | i227);
    if ((unsigned int)d_k > 255U) {
      d_k = 255;
    }

    if (g_direction.North <= 7) {
      i245 = (unsigned char)(1 << g_direction.North);
    } else {
      i245 = 0;
    }

    i227 = (int)((unsigned int)i245 * serch_write[d_k - 1]);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    maze_wall_search[ex] = (unsigned char)(maze_wall_search[ex] | i227);
  }

  /* 西側のマスの東側の壁情報 */
  if (current_x > 1) {
    i227 = i232;
    if ((unsigned int)i232 > 255U) {
      i227 = 255;
    }

    if (g_direction.East <= 7) {
      i242 = (unsigned char)(1 << g_direction.East);
    } else {
      i242 = 0;
    }

    i227 = (int)((unsigned int)i242 * wall_write[i227 - 1]);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    ex = (current_y + ((current_x - 2) << 5)) - 1;
    maze_wall[ex] = (unsigned char)(maze_wall[ex] | i227);
    if ((unsigned int)i232 > 255U) {
      i232 = 255;
    }

    if (g_direction.East <= 7) {
      i247 = (unsigned char)(1 << g_direction.East);
    } else {
      i247 = 0;
    }

    i227 = (int)((unsigned int)i247 * serch_write[i232 - 1]);
    if ((unsigned int)i227 > 255U) {
      i227 = 255;
    }

    maze_wall_search[ex] = (unsigned char)(maze_wall_search[ex] | i227);
  }

  /* 現在地がゴールでない場合 */
  for (i227 = 0; i227 < 9; i227++) {
    varargin_1[i227] = (signed char)((maze_goal->contents[i227] == current_x) *
      (maze_goal->contents[9 + i227] == current_y));
  }

  ex = varargin_1[0];
  for (d_k = 0; d_k < 8; d_k++) {
    i227 = varargin_1[d_k + 1];
    if (ex < i227) {
      ex = i227;
    }
  }

  if (ex == 0) {
    /* 柱に対し、3方向探索済みかつ、すべて壁がない場合、もう一方向の壁を確定させる。 */
    /* 北,東に壁がない場合 */
    if (g_direction.North <= 7) {
      i244 = (unsigned char)(1 << g_direction.North);
    } else {
      i244 = 0;
    }

    if (g_direction.North <= 7) {
      i246 = (unsigned char)(1 << g_direction.North);
    } else {
      i246 = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i244) != i246) {
      if (g_direction.East <= 7) {
        i248 = (unsigned char)(1 << g_direction.East);
      } else {
        i248 = 0;
      }

      if (g_direction.East <= 7) {
        i250 = (unsigned char)(1 << g_direction.East);
      } else {
        i250 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i248) != i250)
      {
        /* 北のマスの東が探索済み　かつ　壁がないとき */
        i227 = (int)(current_y + 1U);
        ex = i227;
        if ((unsigned int)i227 > 255U) {
          ex = 255;
        }

        if (g_direction.East <= 7) {
          i255 = (unsigned char)(1 << g_direction.East);
        } else {
          i255 = 0;
        }

        if ((maze_wall_search[(ex + ((current_x - 1) << 5)) - 1] & i255) != 0) {
          ex = i227;
          if ((unsigned int)i227 > 255U) {
            ex = 255;
          }

          if (g_direction.East <= 7) {
            i258 = (unsigned char)(1 << g_direction.East);
          } else {
            i258 = 0;
          }

          if (g_direction.East <= 7) {
            i263 = (unsigned char)(1 << g_direction.East);
          } else {
            i263 = 0;
          }

          if ((maze_wall[(ex + ((current_x - 1) << 5)) - 1] & i258) != i263) {
            /* 東のマスの北の壁が確定、探索済みとする。 */
            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            d_k = (int)(current_x + 1U);
            if ((unsigned int)d_k > 255U) {
              d_k = 255;
            }

            if (g_direction.North <= 7) {
              i273 = (unsigned char)(1 << g_direction.North);
            } else {
              i273 = 0;
            }

            maze_wall[(current_y + ((ex - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + ((d_k - 1) << 5)) - 1] | i273);
            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            d_k = (int)(current_x + 1U);
            if ((unsigned int)d_k > 255U) {
              d_k = 255;
            }

            if (g_direction.North <= 7) {
              i282 = (unsigned char)(1 << g_direction.North);
            } else {
              i282 = 0;
            }

            maze_wall_search[(current_y + ((ex - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(current_y + ((d_k - 1) << 5)) - 1] | i282);

            /* 隣り合うマス（東北マス）の南の壁も確定 */
            ex = i227;
            if ((unsigned int)i227 > 255U) {
              ex = 255;
            }

            d_k = (int)(current_x + 1U);
            if ((unsigned int)d_k > 255U) {
              d_k = 255;
            }

            i232 = i227;
            if ((unsigned int)i227 > 255U) {
              i232 = 255;
            }

            i235 = (int)(current_x + 1U);
            if ((unsigned int)i235 > 255U) {
              i235 = 255;
            }

            if (g_direction.South <= 7) {
              i298 = (unsigned char)(1 << g_direction.South);
            } else {
              i298 = 0;
            }

            maze_wall[(ex + ((d_k - 1) << 5)) - 1] = (unsigned char)(maze_wall
              [(i232 + ((i235 - 1) << 5)) - 1] | i298);
            ex = i227;
            if ((unsigned int)i227 > 255U) {
              ex = 255;
            }

            d_k = (int)(current_x + 1U);
            if ((unsigned int)d_k > 255U) {
              d_k = 255;
            }

            i232 = i227;
            if ((unsigned int)i227 > 255U) {
              i232 = 255;
            }

            i235 = (int)(current_x + 1U);
            if ((unsigned int)i235 > 255U) {
              i235 = 255;
            }

            if (g_direction.South <= 7) {
              i305 = (unsigned char)(1 << g_direction.South);
            } else {
              i305 = 0;
            }

            maze_wall_search[(ex + ((d_k - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(i232 + ((i235 - 1) << 5)) - 1] | i305);
          }
        }

        /* 東のマスの北が探索済み　かつ　壁がないとき */
        ex = (int)(current_x + 1U);
        d_k = ex;
        if ((unsigned int)ex > 255U) {
          d_k = 255;
        }

        if (g_direction.North <= 7) {
          i262 = (unsigned char)(1 << g_direction.North);
        } else {
          i262 = 0;
        }

        if ((maze_wall_search[(current_y + ((d_k - 1) << 5)) - 1] & i262) != 0)
        {
          d_k = ex;
          if ((unsigned int)ex > 255U) {
            d_k = 255;
          }

          if (g_direction.North <= 7) {
            i269 = (unsigned char)(1 << g_direction.North);
          } else {
            i269 = 0;
          }

          if (g_direction.North <= 7) {
            i272 = (unsigned char)(1 << g_direction.North);
          } else {
            i272 = 0;
          }

          if ((maze_wall[(current_y + ((d_k - 1) << 5)) - 1] & i269) != i272) {
            /* 北のマスの東の壁が確定、探索済みとする。 */
            d_k = i227;
            if ((unsigned int)i227 > 255U) {
              d_k = 255;
            }

            i232 = i227;
            if ((unsigned int)i227 > 255U) {
              i232 = 255;
            }

            if (g_direction.East <= 7) {
              i281 = (unsigned char)(1 << g_direction.East);
            } else {
              i281 = 0;
            }

            maze_wall[(d_k + ((current_x - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(i232 + ((current_x - 1) << 5)) - 1] | i281);
            d_k = i227;
            if ((unsigned int)i227 > 255U) {
              d_k = 255;
            }

            i232 = i227;
            if ((unsigned int)i227 > 255U) {
              i232 = 255;
            }

            if (g_direction.East <= 7) {
              i291 = (unsigned char)(1 << g_direction.East);
            } else {
              i291 = 0;
            }

            maze_wall_search[(d_k + ((current_x - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[(i232 + ((current_x - 1) << 5)) - 1] | i291);

            /* 隣り合うマス（東北マス）の西の壁も確定 */
            d_k = i227;
            if ((unsigned int)i227 > 255U) {
              d_k = 255;
            }

            i232 = ex;
            if ((unsigned int)ex > 255U) {
              i232 = 255;
            }

            i235 = i227;
            if ((unsigned int)i227 > 255U) {
              i235 = 255;
            }

            i299 = ex;
            if ((unsigned int)ex > 255U) {
              i299 = 255;
            }

            if (g_direction.West <= 7) {
              i304 = (unsigned char)(1 << g_direction.West);
            } else {
              i304 = 0;
            }

            maze_wall[(d_k + ((i232 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(i235 + ((i299 - 1) << 5)) - 1] | i304);
            d_k = i227;
            if ((unsigned int)i227 > 255U) {
              d_k = 255;
            }

            i232 = ex;
            if ((unsigned int)ex > 255U) {
              i232 = 255;
            }

            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i310 = (unsigned char)(1 << g_direction.West);
            } else {
              i310 = 0;
            }

            maze_wall_search[(d_k + ((i232 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(i227 + ((ex - 1) << 5)) - 1] | i310);
          }
        }
      }
    }

    /* 北,西に壁がない場合 */
    if (g_direction.North <= 7) {
      i249 = (unsigned char)(1 << g_direction.North);
    } else {
      i249 = 0;
    }

    if (g_direction.North <= 7) {
      i251 = (unsigned char)(1 << g_direction.North);
    } else {
      i251 = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i249) != i251) {
      if (g_direction.West <= 7) {
        i252 = (unsigned char)(1 << g_direction.West);
      } else {
        i252 = 0;
      }

      if (g_direction.West <= 7) {
        i254 = (unsigned char)(1 << g_direction.West);
      } else {
        i254 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i252) != i254)
      {
        /* 北のマスの西が探索済み　かつ　壁がないとき */
        i227 = (int)(current_y + 1U);
        if ((unsigned int)i227 > 255U) {
          i227 = 255;
        }

        if (g_direction.West <= 7) {
          i260 = (unsigned char)(1 << g_direction.West);
        } else {
          i260 = 0;
        }

        if ((maze_wall_search[(i227 + ((current_x - 1) << 5)) - 1] & i260) != 0)
        {
          i227 = (int)(current_y + 1U);
          if ((unsigned int)i227 > 255U) {
            i227 = 255;
          }

          if (g_direction.West <= 7) {
            i266 = (unsigned char)(1 << g_direction.West);
          } else {
            i266 = 0;
          }

          if (g_direction.West <= 7) {
            i271 = (unsigned char)(1 << g_direction.West);
          } else {
            i271 = 0;
          }

          if ((maze_wall[(i227 + ((current_x - 1) << 5)) - 1] & i266) != i271) {
            /* 西のマスの北の壁が確定、探索済みとする。 */
            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.North <= 7) {
              i280 = (unsigned char)(1 << g_direction.North);
            } else {
              i280 = 0;
            }

            maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + (((int)b_qY - 1) << 5)) - 1] | i280);
            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.North <= 7) {
              i290 = (unsigned char)(1 << g_direction.North);
            } else {
              i290 = 0;
            }

            maze_wall_search[(current_y + (((int)qY - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[(current_y + (((int)b_qY - 1) << 5)) - 1] |
                    i290);

            /* 隣り合うマス（北西マス）の南の壁も確定 */
            i227 = (int)(current_y + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.South <= 7) {
              i301 = (unsigned char)(1 << g_direction.South);
            } else {
              i301 = 0;
            }

            maze_wall[(i227 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(ex + (((int)b_qY - 1) << 5)) - 1] | i301);
            i227 = (int)(current_y + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.South <= 7) {
              i309 = (unsigned char)(1 << g_direction.South);
            } else {
              i309 = 0;
            }

            maze_wall_search[(i227 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(ex + (((int)b_qY - 1) << 5)) - 1] | i309);
          }
        }

        /* 西のマスの北が探索済み　かつ　壁がないとき */
        qY = current_x - 1U;
        if (qY > current_x) {
          qY = 0U;
        }

        if (g_direction.North <= 7) {
          i268 = (unsigned char)(1 << g_direction.North);
        } else {
          i268 = 0;
        }

        if ((maze_wall_search[(current_y + (((int)qY - 1) << 5)) - 1] & i268) !=
            0) {
          qY = current_x - 1U;
          if (qY > current_x) {
            qY = 0U;
          }

          if (g_direction.North <= 7) {
            i275 = (unsigned char)(1 << g_direction.North);
          } else {
            i275 = 0;
          }

          if (g_direction.North <= 7) {
            i279 = (unsigned char)(1 << g_direction.North);
          } else {
            i279 = 0;
          }

          if ((maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] & i275) != i279)
          {
            /* 北のマスの西の壁が確定、探索済みとする。 */
            i227 = (int)(current_y + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i289 = (unsigned char)(1 << g_direction.West);
            } else {
              i289 = 0;
            }

            maze_wall[(i227 + ((current_x - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(ex + ((current_x - 1) << 5)) - 1] | i289);
            i227 = (int)(current_y + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i297 = (unsigned char)(1 << g_direction.West);
            } else {
              i297 = 0;
            }

            maze_wall_search[(i227 + ((current_x - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[(ex + ((current_x - 1) << 5)) - 1] | i297);

            /* 隣り合うマス（北西マス）の東の壁も確定 */
            i227 = (int)(current_y + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.East <= 7) {
              i307 = (unsigned char)(1 << g_direction.East);
            } else {
              i307 = 0;
            }

            maze_wall[(i227 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(ex + (((int)b_qY - 1) << 5)) - 1] | i307);
            i227 = (int)(current_y + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.East <= 7) {
              i314 = (unsigned char)(1 << g_direction.East);
            } else {
              i314 = 0;
            }

            maze_wall_search[(i227 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(ex + (((int)b_qY - 1) << 5)) - 1] | i314);
          }
        }
      }
    }

    /* 南,東に壁がない場合 */
    if (g_direction.East <= 7) {
      i253 = (unsigned char)(1 << g_direction.East);
    } else {
      i253 = 0;
    }

    if (g_direction.East <= 7) {
      i256 = (unsigned char)(1 << g_direction.East);
    } else {
      i256 = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i253) != i256) {
      if (g_direction.South <= 7) {
        i257 = (unsigned char)(1 << g_direction.South);
      } else {
        i257 = 0;
      }

      if (g_direction.South <= 7) {
        i261 = (unsigned char)(1 << g_direction.South);
      } else {
        i261 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i257) != i261)
      {
        /* 南のマスの東が探索済み　かつ　壁がないとき */
        qY = current_y - 1U;
        if (qY > current_y) {
          qY = 0U;
        }

        if (g_direction.East <= 7) {
          i267 = (unsigned char)(1 << g_direction.East);
        } else {
          i267 = 0;
        }

        if ((maze_wall_search[((int)qY + ((current_x - 1) << 5)) - 1] & i267) !=
            0) {
          qY = current_y - 1U;
          if (qY > current_y) {
            qY = 0U;
          }

          if (g_direction.East <= 7) {
            i274 = (unsigned char)(1 << g_direction.East);
          } else {
            i274 = 0;
          }

          if (g_direction.East <= 7) {
            i278 = (unsigned char)(1 << g_direction.East);
          } else {
            i278 = 0;
          }

          if ((maze_wall[((int)qY + ((current_x - 1) << 5)) - 1] & i274) != i278)
          {
            /* 東のマスの南の壁が確定、探索済みとする。 */
            i227 = (int)(current_x + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.South <= 7) {
              i288 = (unsigned char)(1 << g_direction.South);
            } else {
              i288 = 0;
            }

            maze_wall[(current_y + ((i227 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + ((ex - 1) << 5)) - 1] | i288);
            i227 = (int)(current_x + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.South <= 7) {
              i296 = (unsigned char)(1 << g_direction.South);
            } else {
              i296 = 0;
            }

            maze_wall_search[(current_y + ((i227 - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[(current_y + ((ex - 1) << 5)) - 1] | i296);

            /* 隣り合うマス（南東マス）の北の壁も確定 */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i227 = (int)(current_x + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.North <= 7) {
              i306 = (unsigned char)(1 << g_direction.North);
            } else {
              i306 = 0;
            }

            maze_wall[((int)qY + ((i227 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)b_qY + ((ex - 1) << 5)) - 1] | i306);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i227 = (int)(current_x + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.North <= 7) {
              i313 = (unsigned char)(1 << g_direction.North);
            } else {
              i313 = 0;
            }

            maze_wall_search[((int)qY + ((i227 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[((int)b_qY + ((ex - 1) << 5)) - 1] | i313);
          }
        }

        /* 東のマスの南が探索済み　かつ　壁がないとき */
        i227 = (int)(current_x + 1U);
        if ((unsigned int)i227 > 255U) {
          i227 = 255;
        }

        if (g_direction.South <= 7) {
          i277 = (unsigned char)(1 << g_direction.South);
        } else {
          i277 = 0;
        }

        if ((maze_wall_search[(current_y + ((i227 - 1) << 5)) - 1] & i277) != 0)
        {
          i227 = (int)(current_x + 1U);
          if ((unsigned int)i227 > 255U) {
            i227 = 255;
          }

          if (g_direction.South <= 7) {
            i284 = (unsigned char)(1 << g_direction.South);
          } else {
            i284 = 0;
          }

          if (g_direction.South <= 7) {
            i287 = (unsigned char)(1 << g_direction.South);
          } else {
            i287 = 0;
          }

          if ((maze_wall[(current_y + ((i227 - 1) << 5)) - 1] & i284) != i287) {
            /* 南のマスの東の壁が確定、探索済みとする。 */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            if (g_direction.East <= 7) {
              i295 = (unsigned char)(1 << g_direction.East);
            } else {
              i295 = 0;
            }

            maze_wall[((int)qY + ((current_x - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)b_qY + ((current_x - 1) << 5)) - 1] | i295);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            if (g_direction.East <= 7) {
              i303 = (unsigned char)(1 << g_direction.East);
            } else {
              i303 = 0;
            }

            maze_wall_search[((int)qY + ((current_x - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[((int)b_qY + ((current_x - 1) << 5)) - 1] |
                    i303);

            /* 隣り合うマス（南東マス）の西の壁が確定。探索済みとする */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i227 = (int)(current_x + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i312 = (unsigned char)(1 << g_direction.West);
            } else {
              i312 = 0;
            }

            maze_wall[((int)qY + ((i227 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)b_qY + ((ex - 1) << 5)) - 1] | i312);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i227 = (int)(current_x + 1U);
            if ((unsigned int)i227 > 255U) {
              i227 = 255;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i317 = (unsigned char)(1 << g_direction.West);
            } else {
              i317 = 0;
            }

            maze_wall_search[((int)qY + ((i227 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[((int)b_qY + ((ex - 1) << 5)) - 1] | i317);
          }
        }
      }
    }

    /* 南,西に壁がない場合 */
    if (g_direction.West <= 7) {
      i259 = (unsigned char)(1 << g_direction.West);
    } else {
      i259 = 0;
    }

    if (g_direction.West <= 7) {
      i264 = (unsigned char)(1 << g_direction.West);
    } else {
      i264 = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i259) != i264) {
      if (g_direction.South <= 7) {
        i265 = (unsigned char)(1 << g_direction.South);
      } else {
        i265 = 0;
      }

      if (g_direction.South <= 7) {
        i270 = (unsigned char)(1 << g_direction.South);
      } else {
        i270 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i265) != i270)
      {
        /* 南のマスの西が探索済み　かつ　壁がないとき */
        qY = current_y - 1U;
        if (qY > current_y) {
          qY = 0U;
        }

        if (g_direction.West <= 7) {
          i276 = (unsigned char)(1 << g_direction.West);
        } else {
          i276 = 0;
        }

        if ((maze_wall_search[((int)qY + ((current_x - 1) << 5)) - 1] & i276) !=
            0) {
          qY = current_y - 1U;
          if (qY > current_y) {
            qY = 0U;
          }

          if (g_direction.West <= 7) {
            i283 = (unsigned char)(1 << g_direction.West);
          } else {
            i283 = 0;
          }

          if (g_direction.West <= 7) {
            i286 = (unsigned char)(1 << g_direction.West);
          } else {
            i286 = 0;
          }

          if ((maze_wall[((int)qY + ((current_x - 1) << 5)) - 1] & i283) != i286)
          {
            /* 西のマスの南の壁が確定、探索済みとする。 */
            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.South <= 7) {
              i294 = (unsigned char)(1 << g_direction.South);
            } else {
              i294 = 0;
            }

            maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + (((int)b_qY - 1) << 5)) - 1] | i294);
            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.South <= 7) {
              i302 = (unsigned char)(1 << g_direction.South);
            } else {
              i302 = 0;
            }

            maze_wall_search[(current_y + (((int)qY - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[(current_y + (((int)b_qY - 1) << 5)) - 1] |
                    i302);

            /* 隣り合うマス（南西マス）の北の壁が確定。探索済みとする */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            c_qY = current_y - 1U;
            if (c_qY > current_y) {
              c_qY = 0U;
            }

            d_qY = current_x - 1U;
            if (d_qY > current_x) {
              d_qY = 0U;
            }

            if (g_direction.North <= 7) {
              i311 = (unsigned char)(1 << g_direction.North);
            } else {
              i311 = 0;
            }

            maze_wall[((int)qY + (((int)b_qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)c_qY + (((int)d_qY - 1) << 5)) - 1] | i311);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            c_qY = current_y - 1U;
            if (c_qY > current_y) {
              c_qY = 0U;
            }

            d_qY = current_x - 1U;
            if (d_qY > current_x) {
              d_qY = 0U;
            }

            if (g_direction.North <= 7) {
              i316 = (unsigned char)(1 << g_direction.North);
            } else {
              i316 = 0;
            }

            maze_wall_search[((int)qY + (((int)b_qY - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[((int)c_qY + (((int)d_qY - 1) << 5)) - 1] |
                    i316);
          }
        }

        /* 西のマスの南が探索済み　かつ　壁がないとき */
        qY = current_x - 1U;
        if (qY > current_x) {
          qY = 0U;
        }

        if (g_direction.South <= 7) {
          i285 = (unsigned char)(1 << g_direction.South);
        } else {
          i285 = 0;
        }

        if ((maze_wall_search[(current_y + (((int)qY - 1) << 5)) - 1] & i285) !=
            0) {
          qY = current_x - 1U;
          if (qY > current_x) {
            qY = 0U;
          }

          if (g_direction.South <= 7) {
            i292 = (unsigned char)(1 << g_direction.South);
          } else {
            i292 = 0;
          }

          if (g_direction.South <= 7) {
            i293 = (unsigned char)(1 << g_direction.South);
          } else {
            i293 = 0;
          }

          if ((maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] & i292) != i293)
          {
            /* 南のマスの西の壁が確定、探索済みとする。 */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            if (g_direction.West <= 7) {
              i300 = (unsigned char)(1 << g_direction.West);
            } else {
              i300 = 0;
            }

            maze_wall[((int)qY + ((current_x - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)b_qY + ((current_x - 1) << 5)) - 1] | i300);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_y - 1U;
            if (b_qY > current_y) {
              b_qY = 0U;
            }

            if (g_direction.West <= 7) {
              i308 = (unsigned char)(1 << g_direction.West);
            } else {
              i308 = 0;
            }

            maze_wall_search[((int)qY + ((current_x - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[((int)b_qY + ((current_x - 1) << 5)) - 1] |
                    i308);

            /* 隣り合うマス（南西マス）の東の壁が確定。探索済みとする */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            c_qY = current_y - 1U;
            if (c_qY > current_y) {
              c_qY = 0U;
            }

            d_qY = current_x - 1U;
            if (d_qY > current_x) {
              d_qY = 0U;
            }

            if (g_direction.East <= 7) {
              i315 = (unsigned char)(1 << g_direction.East);
            } else {
              i315 = 0;
            }

            maze_wall[((int)qY + (((int)b_qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)c_qY + (((int)d_qY - 1) << 5)) - 1] | i315);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            c_qY = current_y - 1U;
            if (c_qY > current_y) {
              c_qY = 0U;
            }

            d_qY = current_x - 1U;
            if (d_qY > current_x) {
              d_qY = 0U;
            }

            if (g_direction.East <= 7) {
              i318 = (unsigned char)(1 << g_direction.East);
            } else {
              i318 = 0;
            }

            maze_wall_search[((int)qY + (((int)b_qY - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[((int)c_qY + (((int)d_qY - 1) << 5)) - 1] |
                    i318);
          }
        }
      }
    }
  }
}

/*
 * maze_solve 実機での迷路探索関数
 * 入力 迷路壁情報,迷路探索情報,迷路縦サイズ,迷路横サイズ,ゴール座標,
 * 出力 壁情報,探索情報
 * Arguments    : unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char goal_size
 *                const unsigned char maze_goal[18]
 *                unsigned char run_mode_1
 *                unsigned char run_mode_2
 *                unsigned short contour_map[1024]
 *                unsigned short row_num_node[1056]
 *                unsigned short col_num_node[1056]
 * Return Type  : void
 */
void maze_solve(unsigned char maze_wall[1024], unsigned char maze_wall_search
                [1024], unsigned char maze_row_size, unsigned char maze_col_size,
                unsigned char goal_size, const unsigned char maze_goal[18],
                unsigned char run_mode_1, unsigned char run_mode_2, unsigned
                short contour_map[1024], unsigned short row_num_node[1056],
                unsigned short col_num_node[1056])
{
  coder_internal_ref b_goal_size;
  int N;
  coder_internal_ref_1 b_maze_wall;
  coder_internal_ref_2 b_maze_goal;
  coder_internal_ref start_flg;
  coder_internal_ref wall_flg;
  unsigned char new_goal[2];
  coder_internal_ref_6 wall;
  coder_internal_ref_5 search;
  coder_internal_ref_4 adachi_search_mode;
  coder_internal_ref_3 max_length;
  unsigned short start_num;
  unsigned char goal_matrix_dir2;
  unsigned char search_flag;
  unsigned char goal_dir;
  unsigned char current_y;
  unsigned char goal_section[2];
  unsigned char goal_node2[2];
  unsigned char current_dir;
  unsigned short unusedExpr[1024];
  unsigned char current_x;
  unsigned char b_current_y;
  unsigned char stop_flg;
  int exitg1;
  bool exitg2;
  unsigned char unexp_square[1024];
  unsigned short b_unusedExpr[1024];
  unsigned short c_unusedExpr[1024];
  unsigned short d_unusedExpr[1024];
  unsigned short e_unusedExpr[1024];
  b_goal_size.contents = goal_size;
  for (N = 0; N < 18; N++) {
    b_maze_goal.contents[N] = maze_goal[N];
  }

  memcpy(&b_maze_wall.contents[0], &maze_wall[0], sizeof(unsigned char) << 10);

  /* C言語関数インクルード */
  /* ローカル変数定義 */
  /*  ゴール時ストップフラグ(0:移動継続　1:ストップ) */
  start_flg.contents = 0U;

  /*  スタートフラグ(0:動作中　1:停止からの移動開始) */
  /* ゴール直後フラグ(0:ゴール直後でない, 1:ゴール直後) */
  wall_flg.contents = 0U;

  /* 壁フラグ(1:前、2:右、（4:後ろ)、8:左) */
  /*  グローバル変数宣言 */
  /* メインfigure */
  /* メインaxes */
  /* ビデオ用 */
  /* ビデオ用 */
  /* プロット用変数 */
  /* 最短経路ラインオブジェクト保持用 */
  /* ゴールラインオブジェクト保持用 */
  /*  global maze_goal; */
  /* ローカル変数宣言 */
  new_goal[0] = 0U;
  new_goal[1] = 0U;
  memset(&contour_map[0], 0, sizeof(unsigned short) << 10);
  for (N = 0; N < 1056; N++) {
    row_num_node[N] = MAX_uint16_T;
    col_num_node[N] = MAX_uint16_T;
  }

  /* ノードの属性定義 */
  /* 0:行方向, 1:列方向,　2:セクション（マス） */
  matrix_dir.Row = 0U;
  matrix_dir.Col = 1U;
  matrix_dir.section = 2U;

  /* 絶対方角定義 */
  g_direction.North = 0U;
  g_direction.East = 1U;
  g_direction.South = 2U;
  g_direction.West = 3U;

  /* 斜め込みの絶対方角定義 */
  g_d_direction.North = 0U;
  g_d_direction.North_East = 1U;
  g_d_direction.East = 2U;
  g_d_direction.South_East = 3U;
  g_d_direction.South = 4U;
  g_d_direction.South_West = 5U;
  g_d_direction.West = 6U;
  g_d_direction.North_West = 7U;

  /* マウス方向定義 */
  l_direction.front = 0U;
  l_direction.right = 1U;
  l_direction.back = 2U;
  l_direction.left = 3U;

  /* 移動方向属性定義 */
  /* 直進or斜め */
  move_dir_property.straight = 0U;
  move_dir_property.diagonal = 1U;

  /* パターン番号定義 */
  turn_pattern.b_default = 0U;
  turn_pattern.r_45 = 1U;
  turn_pattern.l_45 = 2U;
  turn_pattern.r_90 = 3U;
  turn_pattern.l_90 = 4U;
  turn_pattern.r_135 = 5U;
  turn_pattern.l_135 = 6U;
  turn_pattern.r_180 = 7U;
  turn_pattern.l_180 = 8U;

  /* 壁情報定義 */
  wall.contents.nowall = 0U;
  wall.contents.wall = 1U;

  /* 探索情報定義 */
  search.contents.unknown = 0U;
  search.contents.known = 1U;

  /* 走行モード定義 */
  /* 探索モード */
  /* 未探索壁扱いモード定義 */
  /* 最短経路導出モード */
  /* 足立法による探索モード　ゴールor探索 */
  adachi_search_mode.contents.goal = 0U;
  adachi_search_mode.contents.search = 1U;

  /*  探索 */
  if (run_mode_1 == 0) {
    /* マウスの初期位置設定 */
    /* for C gen */
    /* 各フラグを定義 */
    /* 停止直後 */
    /* 停止処理を実施する */
    /* ゴール直後フラグはクリア */
    /* 一マス前進 */
    goal_matrix_dir2 = 1U;
    current_y = 1U;
    move_step(&goal_matrix_dir2, &current_y, g_direction.North);

    /* C言語でのスタート処理 */
    m_start_movement(1, 0, move_dir_property.straight);

    /* 停止直後フラグをクリア */
    /* ゴールをプロット */
    /* 足立法による探索 */
    current_dir = g_direction.North;
    search_flag = 0U;
    search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &b_maze_wall,
                  &adachi_search_mode, &goal_matrix_dir2, &current_y,
                  &current_dir, maze_row_size, maze_col_size, maze_wall,
                  maze_wall_search, maze_goal, goal_size, &search_flag, 0U,
                  unusedExpr);
    current_x = goal_matrix_dir2;
    b_current_y = current_y;
    memcpy(&b_maze_wall.contents[0], &maze_wall[0], sizeof(unsigned char) << 10);

    /* ひとまづゴール(停止させる) */
    /* 各フラグを定義 */
    start_flg.contents = 1U;

    /* 停止直後 */
    stop_flg = 0U;

    /* 停止処理を実施しない */
    /* ゴール直後フラグをたてる */
    /* 進入時のゴールを記憶する */
    /* ゴールをすべて探索 */
    do {
      exitg1 = 0;
      search_flag = 0U;
      N = 0;
      exitg2 = false;
      while ((!exitg2) && (N <= goal_size - 1)) {
        goal_dir = b_maze_goal.contents[N + 9];
        if (maze_wall_search[(goal_dir + ((b_maze_goal.contents[N] - 1) << 5)) -
            1] != 15) {
          new_goal[0] = b_maze_goal.contents[N];
          new_goal[1] = goal_dir;
          search_flag = 1U;
          exitg2 = true;
        } else {
          N++;
        }
      }

      if (search_flag == 1) {
        memcpy(&unexp_square[0], &b_maze_wall.contents[0], sizeof(unsigned char)
               << 10);
        b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &b_maze_wall,
                        &adachi_search_mode, &current_x, &b_current_y,
                        &current_dir, maze_row_size, maze_col_size, unexp_square,
                        maze_wall_search, new_goal, &start_flg.contents, 0U, 1U,
                        1U, b_unusedExpr);
        memcpy(&b_maze_wall.contents[0], &unexp_square[0], sizeof(unsigned char)
               << 10);

        /* ゴール直後フラグをたてる */
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* 未探索のゴールマスがなく、現在地がゴール進入マスでない（ゴールが一マスでない） */
    /* ゴール進入マスで停止させる。 */
    if ((current_x == goal_matrix_dir2) && (b_current_y == current_y)) {
    } else {
      stop_flg = 1U;

      /* 停止処理を実施する */
      memcpy(&unexp_square[0], &b_maze_wall.contents[0], sizeof(unsigned char) <<
             10);
      goal_section[0] = goal_matrix_dir2;
      goal_section[1] = current_y;
      b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &b_maze_wall,
                      &adachi_search_mode, &current_x, &b_current_y,
                      &current_dir, maze_row_size, maze_col_size, unexp_square,
                      maze_wall_search, goal_section, &start_flg.contents, 1U,
                      1U, 0U, c_unusedExpr);
      memcpy(&b_maze_wall.contents[0], &unexp_square[0], sizeof(unsigned char) <<
             10);

      /* ゴール直後フラグをたてる */
    }

    /* 未探索ゴールがなくなったとき、マウスは停止しているはず。 */
    /* 停止直後フラグを立てる。 */
    start_flg.contents = 1U;

    /* 帰路探索 */
    /* 帰路探索前に今までの迷路情報を書き込む */
    m_rom_write();

    /* 全面探索 */
    /* 未探索マスがなくなるまで。 */
    /* 現地点から一番近い未探索マスを探索 */
    /* 現在地からコンターマップを展開、探索済みでないマスが見つかれば、そこをゴールとする。 */
    switch (run_mode_2) {
     case 1:
      do {
        exitg1 = 0;
        make_new_goal_all(&wall, b_maze_wall.contents, maze_wall_search,
                          current_x, b_current_y, contour_map, new_goal);
        if (new_goal[0] == 0) {
          exitg1 = 1;
        } else {
          /* ゴールをプロット */
          /* 既存のコンターを使用し、探索。 */
          memcpy(&unexp_square[0], &b_maze_wall.contents[0], sizeof(unsigned
                  char) << 10);
          b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &b_maze_wall,
                          &adachi_search_mode, &current_x, &b_current_y,
                          &current_dir, maze_row_size, maze_col_size,
                          unexp_square, maze_wall_search, new_goal,
                          &start_flg.contents, stop_flg, 1U, 1U, d_unusedExpr);
          memcpy(&b_maze_wall.contents[0], &unexp_square[0], sizeof(unsigned
                  char) << 10);

          /* ゴール直後フラグをたてる */
        }
      } while (exitg1 == 0);

      /* 新規ゴールが見つからないとき、停止処理を実施 */
      m_goal_movement(start_flg.contents, wall_flg.contents,
                      move_dir_property.straight);

      /* 探索終了時、停止させているため、フラグをたてる。 */
      start_flg.contents = 1U;

      /* 停止直後 */
      /* 最短経路探索 */
      /* 最短となりうるマスの未探索のみ探索 */
      break;

     case 2:
      do {
        exitg1 = 0;

        /* 未探索壁はないものとして、ゴールからスタートまで、コンターマップを展開（直線の重みあり） */
        make_map_fustrun(&b_goal_size, &wall, &search, b_maze_goal.contents,
                         b_maze_wall.contents, maze_wall_search, 0U, contour_map);

        /* マップをもとに、最短経路を導出。経路上の未探索マスとその数を出力 */
        fust_run(&b_maze_wall, &b_goal_size, &wall_flg, &wall,
                 b_maze_wall.contents, maze_wall_search, contour_map,
                 b_maze_goal.contents, MAX_uint16_T, unexp_square, &search_flag);

        /* 未探索マスがなければ、ブレイク */
        if (search_flag == 0) {
          exitg1 = 1;
        } else {
          /* 未探索マスがある場合、探索する。 */
          /* 現在地点からコンターを展開し、該当の未探索が更新されれば、そこを新規ゴールとして出力 */
          make_new_goal_sh(&wall, b_maze_wall.contents, current_x, b_current_y,
                           unexp_square, search_flag, contour_map, new_goal);

          /* ゴールをプロット */
          /* 新規ゴールに向け、探索 */
          memcpy(&unexp_square[0], &b_maze_wall.contents[0], sizeof(unsigned
                  char) << 10);
          b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &b_maze_wall,
                          &adachi_search_mode, &current_x, &b_current_y,
                          &current_dir, maze_row_size, maze_col_size,
                          unexp_square, maze_wall_search, new_goal,
                          &start_flg.contents, stop_flg, 1U, 1U, e_unusedExpr);
          memcpy(&b_maze_wall.contents[0], &unexp_square[0], sizeof(unsigned
                  char) << 10);

          /* ゴール直後フラグをたてる */
        }
      } while (exitg1 == 0);

      /* 新規ゴールが見つからないとき、停止処理を実施 */
      m_goal_movement(start_flg.contents, wall_flg.contents,
                      move_dir_property.straight);

      /* 探索終了時、停止させているため、停止フラグを立てる。 */
      start_flg.contents = 1U;

      /* 停止直後 */
      /* その他の場合無視 */
      break;
    }

    /* スタートを目的地として足立法で再探索 */
    /* 各フラグを定義 */
    /* 停止処理を実施する */
    /* ゴール直後フラグをたてる */
    /* スタートをゴールに設定 */
    /* ゴールをプロット */
    /* 足立法で帰宅 */
    memcpy(&unexp_square[0], &b_maze_wall.contents[0], sizeof(unsigned char) <<
           10);
    goal_node2[0] = 1U;
    goal_node2[1] = 1U;
    b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &b_maze_wall,
                    &adachi_search_mode, &current_x, &b_current_y, &current_dir,
                    maze_row_size, maze_col_size, unexp_square, maze_wall_search,
                    goal_node2, &start_flg.contents, 1U, 1U, 0U, contour_map);
    memcpy(&b_maze_wall.contents[0], &unexp_square[0], sizeof(unsigned char) <<
           10);

    /* 走行完了時、迷路情報を記録する。 */
    m_rom_write();

    /* for code generation */
    /* 終了時、ゴールプロットを消す。 */
    /*     %% 最短走行 */
  } else {
    if (run_mode_1 == 1) {
      if (run_mode_2 == 0) {
        /* 探索情報をもとに等高線MAPを生成 */
        /* 未知壁は仮想壁を設置する。(w_mode.wall) */
        make_map_fustrun(&b_goal_size, &wall, &search, maze_goal, maze_wall,
                         maze_wall_search, 1U, contour_map);

        /* ゴールの描画 */
        /*      %コンターマップの描画 */
        /*      if coder.target('MATLAB') */
        /*          for l = 1:32 */
        /*              for j = 1:32 */
        /*                  text((j-1)*9+4.5,(l-1)*9+4.5,string(contour_map(l,j)),'HorizontalAlignment','center'); */
        /*              end */
        /*          end */
        /*      end */
        /* 各走行フラグを定義 */
        /* 停止直後 */
        /* 停止処理を実施する */
        /* ゴール直後フラグはクリア */
        /* 一マス前進 */
        goal_matrix_dir2 = 1U;
        current_y = 1U;
        move_step(&goal_matrix_dir2, &current_y, g_d_direction.North);

        /* C言語でのスタート処理 */
        m_start_movement(1, 0, move_dir_property.straight);

        /* 停止直後フラグをクリア */
        /* 最短距離走行 */
        b_fust_run(&b_maze_wall, &b_goal_size, &wall_flg, &wall, maze_wall,
                   contour_map, maze_goal, MAX_uint16_T, goal_matrix_dir2,
                   current_y, 0U);

        /*  斜めでの最短走行 */
      } else {
        if (run_mode_2 == 1) {
          /* 最短経路生成 */
          /* ゴールマスのノードをすべてゴールノードとし、マップ生成 */
          make_map_fustrun_diagonal(&max_length, &wall, &search, maze_goal,
            goal_size, maze_wall, maze_wall_search, row_num_node, col_num_node,
            &start_num);

          /* ゴール付近のルート最適化のため、マップ再生成 */
          /* スタートからゴールノードまで、ルート生成し、ゴールノード、方向を確定 */
          decide_goal_node_dir(maze_goal, goal_size, row_num_node, col_num_node,
                               new_goal, &search_flag, &goal_dir);

          /* 確定されたゴールノード、方向からゴールマス、ノードを再定義 */
          decide_goal_section(maze_goal, new_goal, search_flag, goal_dir,
                              goal_section, goal_node2, &goal_matrix_dir2);

          /* 確定されたゴールマスから、再度マップを生成 */
          new_goal[0] = goal_section[1];
          new_goal[1] = goal_section[0];

          /* x,yに変換 */
          b_make_map_fustrun_diagonal(&max_length, &wall, &search, new_goal,
            maze_wall, maze_wall_search, row_num_node, col_num_node, &start_num);

          /* 生成されたMAPをもとに最短走行 */
          make_route_diagonal(&start_flg, &wall_flg, row_num_node, col_num_node,
                              new_goal, goal_node2, goal_matrix_dir2);
        }
      }
    }
  }

  memcpy(&maze_wall[0], &b_maze_wall.contents[0], sizeof(unsigned char) << 10);
}

/*
 * File trailer for maze_solve.c
 *
 * [EOF]
 */
