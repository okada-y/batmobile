/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_solve.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 27-Aug-2020 23:54:33
 */

/* Include Files */
#include <string.h>
#include <math.h>
#include "maze_init.h"
#include "maze_solve.h"
#include "rem.h"
#include "isequal.h"
#include "maze_init_data.h"
#include "C:\work\matlab\maze_sim_git\src\C_src\matlab_movement.h"
#include "C:\work\matlab\maze_sim_git\src\C_src\matlab_IR_sensor.h"

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
  unsigned char contents[18];
} coder_internal_ref_1;

#endif                                 /*typedef_coder_internal_ref_1*/

#ifndef typedef_coder_internal_ref_2
#define typedef_coder_internal_ref_2

typedef struct {
  unsigned short contents;
} coder_internal_ref_2;

#endif                                 /*typedef_coder_internal_ref_2*/

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

typedef struct {
  unsigned char goal;
  unsigned char search;
} h_struct_T;

#endif                                 /*typedef_h_struct_T*/

#ifndef typedef_coder_internal_ref_3
#define typedef_coder_internal_ref_3

typedef struct {
  h_struct_T contents;
} coder_internal_ref_3;

#endif                                 /*typedef_coder_internal_ref_3*/

#ifndef typedef_g_struct_T
#define typedef_g_struct_T

typedef struct {
  unsigned char unknown;
  unsigned char known;
} g_struct_T;

#endif                                 /*typedef_g_struct_T*/

#ifndef typedef_coder_internal_ref_4
#define typedef_coder_internal_ref_4

typedef struct {
  g_struct_T contents;
} coder_internal_ref_4;

#endif                                 /*typedef_coder_internal_ref_4*/

#ifndef typedef_f_struct_T
#define typedef_f_struct_T

typedef struct {
  unsigned char nowall;
  unsigned char wall;
} f_struct_T;

#endif                                 /*typedef_f_struct_T*/

#ifndef typedef_coder_internal_ref_5
#define typedef_coder_internal_ref_5

typedef struct {
  f_struct_T contents;
} coder_internal_ref_5;

#endif                                 /*typedef_coder_internal_ref_5*/

/* Function Declarations */
static void b_fust_run(const coder_internal_ref *goal_size, coder_internal_ref
  *wall_flg, const coder_internal_ref_5 *wall, const unsigned char maze_wall
  [1024], const unsigned short contour_map[1024], const unsigned char maze_goal
  [18], unsigned short max_length, unsigned char start_x, unsigned char start_y);
static void b_make_map_find(const coder_internal_ref_5 *wall, const unsigned
  char maze_goal[2], const unsigned char maze_wall[1024], unsigned char
  current_x, unsigned char current_y, unsigned short contour_map[1024]);
static void b_make_map_fustrun_diagonal(coder_internal_ref_2 *max_length, const
  coder_internal_ref_5 *wall, const coder_internal_ref_4 *search, const unsigned
  char maze_goal[2], const unsigned char maze_wall[1024], const unsigned char
  maze_wall_search[1024], unsigned short row_num_node[1056], unsigned short
  col_num_node[1056], unsigned short *start_num);
static void b_search_adachi(const coder_internal_ref_5 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_4 *search, const coder_internal_ref_1
  *maze_goal, const coder_internal_ref_3 *adachi_search_mode, unsigned char
  *current_x, unsigned char *current_y, unsigned char *current_dir, unsigned
  char maze_row_size, unsigned char maze_col_size, unsigned char maze_wall[1024],
  unsigned char maze_wall_search[1024], const unsigned char exploration_goal[2],
  unsigned char *start_flg, unsigned char stop_flg, unsigned char goal_after_flg,
  unsigned char adachi_s_mode, unsigned short contour_map[1024]);
static void decide_goal_node_dir(const unsigned char maze_goal[18], unsigned
  char goal_size, const unsigned short row_num_node[1056], const unsigned short
  col_num_node[1056], unsigned char goal_node[2], unsigned char *goal_matrix_dir,
  unsigned char *goal_dir);
static void decide_goal_section(const unsigned char maze_goal[18], const
  unsigned char goal_node[2], unsigned char goal_matrix_dir, unsigned char
  goal_dir, unsigned char goal_section[2], unsigned char goal_node2[2], unsigned
  char *goal_matrix_dir2);
static void fust_run(const coder_internal_ref *goal_size, const
                     coder_internal_ref_5 *wall, const unsigned char maze_wall
                     [1024], const unsigned char maze_wall_search[1024], const
                     unsigned short contour_map[1024], const unsigned char
                     maze_goal[18], unsigned short max_length, unsigned char
                     unexp_square[1024], unsigned char *unexp_square_idx);
static void get_next_dir_diagonal(const unsigned short row_num_node[1056], const
  unsigned short col_num_node[1056], unsigned char current_move_dir, const
  unsigned char current_node[2], unsigned char current_matrix_dir, const
  unsigned char goal_node2[2], unsigned char goal_matrix_dir2, const unsigned
  char goal_section[2], unsigned char *next_dir, unsigned char next_node[2],
  unsigned char *next_node_property);
static unsigned char get_nextdir2(unsigned char current_x, unsigned char
  current_y, const unsigned char maze_wall[1024], const unsigned short
  contour_map[1024]);
static unsigned char get_turn_pattern_num(const double move_dir_buffer[3],
  unsigned char ref_move_mode);
static double goal_judge(const unsigned char maze_goal[18], unsigned char x,
  unsigned char y);
static void make_map_find(const coder_internal_ref_5 *wall, const unsigned char
  maze_goal[18], unsigned char l_goal_size, const unsigned char maze_wall[1024],
  unsigned char current_x, unsigned char current_y, unsigned short contour_map
  [1024]);
static void make_map_fustrun(const coder_internal_ref *goal_size, const
  coder_internal_ref_5 *wall, const coder_internal_ref_4 *search, const unsigned
  char maze_goal[18], const unsigned char maze_wall[1024], const unsigned char
  maze_wall_search[1024], unsigned char unknown_wall_flg, unsigned short
  contour_map[1024]);
static void make_map_fustrun_diagonal(coder_internal_ref_2 *max_length, const
  coder_internal_ref_5 *wall, const coder_internal_ref_4 *search, const unsigned
  char maze_goal[18], unsigned char goal_size, const unsigned char maze_wall
  [1024], const unsigned char maze_wall_search[1024], unsigned short
  row_num_node[1056], unsigned short col_num_node[1056], unsigned short
  *start_num);
static void make_new_goal_all(const coder_internal_ref_5 *wall, const unsigned
  char maze_wall[1024], const unsigned char maze_wall_search[1024], unsigned
  char current_x, unsigned char current_y, unsigned short contour_map[1024],
  unsigned char new_goal[2]);
static void make_new_goal_sh(const coder_internal_ref_5 *wall, const unsigned
  char maze_wall[1024], unsigned char current_x, unsigned char current_y, const
  unsigned char unexp_square[1024], unsigned char unexp_square_idx, unsigned
  short contour_map[1024], unsigned char new_goal[2]);
static void make_route_diagonal(const unsigned short row_num_node[1056], const
  unsigned short col_num_node[1056], const unsigned char goal_section[2], const
  unsigned char goal_node2[2], unsigned char goal_node_property);
static void move_step(unsigned char *temp_x, unsigned char *temp_y, unsigned
                      char temp_dir);
static void move_straight(unsigned char current_node[2], unsigned char
  *current_node_property, unsigned char *current_move_dir, unsigned char
  *current_move_mode, unsigned char straight_count);
static void search_adachi(const coder_internal_ref_5 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_4 *search, const coder_internal_ref_1
  *maze_goal, const coder_internal_ref_3 *adachi_search_mode, unsigned char
  *current_x, unsigned char *current_y, unsigned char *current_dir, unsigned
  char maze_row_size, unsigned char maze_col_size, unsigned char maze_wall[1024],
  unsigned char maze_wall_search[1024], const unsigned char exploration_goal[18],
  unsigned char l_goal_size, unsigned char *start_flg, unsigned char
  adachi_s_mode, unsigned short contour_map[1024]);
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
static void wall_set(const coder_internal_ref_5 *wall, coder_internal_ref
                     *wall_flg, const coder_internal_ref_4 *search, const
                     coder_internal_ref_1 *maze_goal, unsigned char
                     maze_row_size, unsigned char maze_col_size, unsigned char
                     current_x, unsigned char current_y, unsigned char
                     current_dir, unsigned char maze_wall[1024], unsigned char
                     maze_wall_search[1024]);

/* Function Definitions */

/*
 * 入力　壁情報,壁探索情報,等高線MAP,ゴール座標,最大経路長
 * 出力   最短経路上の未探索マスの座標、未探索マスの数
 * Arguments    : const coder_internal_ref *goal_size
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_5 *wall
 *                const unsigned char maze_wall[1024]
 *                const unsigned short contour_map[1024]
 *                const unsigned char maze_goal[18]
 *                unsigned short max_length
 *                unsigned char start_x
 *                unsigned char start_y
 * Return Type  : void
 */
static void b_fust_run(const coder_internal_ref *goal_size, coder_internal_ref
  *wall_flg, const coder_internal_ref_5 *wall, const unsigned char maze_wall
  [1024], const unsigned short contour_map[1024], const unsigned char maze_goal
  [18], unsigned short max_length, unsigned char start_x, unsigned char start_y)
{
  unsigned char goal_flag;
  unsigned short little;
  unsigned char temp_x;
  unsigned char temp_y;
  unsigned char temp_dir;
  unsigned char next_dir;
  int tempk;
  bool exitg1;
  int i67;
  int tempi;
  int i68;
  int i69;
  unsigned short u5;
  int i70;
  int i71;
  int b_temp_dir;
  int i72;
  int i73;
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
    /* 現在位置が未探索マスか判定 */
    /* 現在位置がゴールか判定 */
    i67 = goal_size->contents;
    for (tempi = 0; tempi < i67; tempi++) {
      if ((temp_x == maze_goal[tempi]) && (temp_y == maze_goal[tempi + 9])) {
        goal_flag = 1U;
      }
    }

    if (goal_flag == 1) {
      /* ゴールかつ走行時、停止処理を実施 */
      /* 走行モード時、Cの動作関数を呼び出し */
      m_goal_movement(0, wall_flg->contents, move_dir_property.straight);
      exitg1 = true;
    } else {
      /*         %%進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      /* 北側の壁のありなし */
      i67 = temp_y + ((temp_x - 1) << 5);
      tempi = maze_wall[i67 - 1];
      if (g_direction.North <= 7) {
        i68 = (unsigned char)(1 << g_direction.North);
      } else {
        i68 = 0;
      }

      if (((tempi & i68) == wall->contents.nowall) && (contour_map[i67] < little))
      {
        /* 北側の等高線mapが閾値より低ければ、 */
        /* 閾値を北側の等高map値に変更 */
        little = contour_map[temp_y + ((temp_x - 1) << 5)];

        /* 北側を進行方向に変更y */
        next_dir = g_direction.North;
      }

      /* 東側 */
      if (g_direction.East <= 7) {
        i69 = (unsigned char)(1 << g_direction.East);
      } else {
        i69 = 0;
      }

      if ((tempi & i69) == wall->contents.nowall) {
        u5 = contour_map[(temp_y + (temp_x << 5)) - 1];
        if (u5 < little) {
          little = u5;
          next_dir = g_direction.East;
        }
      }

      /* 南側 */
      if (g_direction.South <= 7) {
        i70 = (unsigned char)(1 << g_direction.South);
      } else {
        i70 = 0;
      }

      if ((tempi & i70) == wall->contents.nowall) {
        u5 = contour_map[i67 - 2];
        if (u5 < little) {
          little = u5;
          next_dir = g_direction.South;
        }
      }

      /* 西側 */
      if (g_direction.West <= 7) {
        i71 = (unsigned char)(1 << g_direction.West);
      } else {
        i71 = 0;
      }

      if ((tempi & i71) == wall->contents.nowall) {
        u5 = contour_map[(temp_y + ((temp_x - 2) << 5)) - 1];
        if (u5 < little) {
          little = u5;
          next_dir = g_direction.West;
        }
      }

      /* 走行時、探索壁情報に応じて、壁フラグをセット */
      /* 前 */
      if (temp_dir <= 7) {
        b_temp_dir = (unsigned char)(1 << temp_dir);
      } else {
        b_temp_dir = 0;
      }

      if ((tempi & (b_temp_dir % 15)) != 0) {
        wall_flg->contents |= 1;
      }

      /* 右 */
      i67 = (int)(temp_dir + 1U);
      if ((unsigned int)i67 > 255U) {
        i67 = 255;
      }

      if ((unsigned char)i67 <= 7) {
        i72 = (unsigned char)(1 << (unsigned char)i67);
      } else {
        i72 = 0;
      }

      if ((tempi & (i72 % 15)) != 0) {
        wall_flg->contents = (unsigned char)(wall_flg->contents | 2);
      }

      /* 左 */
      i67 = (int)(temp_dir + 3U);
      if ((unsigned int)i67 > 255U) {
        i67 = 255;
      }

      if ((unsigned char)i67 <= 7) {
        i73 = (unsigned char)(1 << (unsigned char)i67);
      } else {
        i73 = 0;
      }

      if ((tempi & (i73 % 15)) != 0) {
        wall_flg->contents = (unsigned char)(wall_flg->contents | 8);
      }

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
        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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

        /* disp("front") */
        /* 走行モード時、Cの動作関数を呼び出し */
        m_move_front(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 1:
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_clk_90deg 時計周りに90度ターンする関数 */
        i67 = (int)(4U + temp_dir);
        if ((unsigned int)i67 > 255U) {
          i67 = 255;
        }

        i67++;
        if ((unsigned int)i67 > 255U) {
          i67 = 255;
        }

        temp_dir = (unsigned char)(i67 % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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

        /* disp("right") */
        /* 走行モード時、Cの動作関数を呼び出し */
        m_move_right(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 2:
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_180deg 180度ターンする関数 */
        i67 = (int)(4U + temp_dir);
        if ((unsigned int)i67 > 255U) {
          i67 = 255;
        }

        temp_dir = (unsigned char)((i67 - 2) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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

        /* disp("back") */
        /* 走行モード時、Cの動作関数を呼び出し */
        m_move_back(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 3:
        /* 入力　現在方向 */
        /* 出力　現在方向 */
        /*     %% turn_conclk_90deg 反時計周りに90度回る関数 */
        i67 = (int)(4U + temp_dir);
        if ((unsigned int)i67 > 255U) {
          i67 = 255;
        }

        temp_dir = (unsigned char)((i67 - 1) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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
        m_move_left(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグと壁フラグをクリア */
        wall_flg->contents = 0U;
        break;
      }

      /* for code generation */
      tempk++;
    }
  }

  /*          pause(0.01) */
}

/*
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : const coder_internal_ref_5 *wall
 *                const unsigned char maze_goal[2]
 *                const unsigned char maze_wall[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void b_make_map_find(const coder_internal_ref_5 *wall, const unsigned
  char maze_goal[2], const unsigned char maze_wall[1024], unsigned char
  current_x, unsigned char current_y, unsigned short contour_map[1024])
{
  unsigned char contor_renew_square[2048];
  unsigned char contor_renew_square_temp[2048];
  unsigned char contor_renew_square_idx;
  unsigned char contor_renew_square_idx_temp;
  int i17;
  unsigned short tempi;
  bool exitg1;
  unsigned char change_flag;
  int tempn;
  int i18;
  int i19;
  int i20;
  int i21;
  int i22;
  int i23;
  int i24;
  unsigned int qY;
  int i25;

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
  for (i17 = 0; i17 < 1024; i17++) {
    contour_map[i17] = MAX_uint16_T;
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
    i17 = contor_renew_square_idx;
    for (tempn = 0; tempn < i17; tempn++) {
      /* 北側 */
      /* if (bitand(maze_wall(row(tempn),col(tempn)),bitshift(uint8(1),g_direction.North)) == wall.nowall) */
      if (g_direction.North <= 7) {
        i18 = (unsigned char)(1 << g_direction.North);
      } else {
        i18 = 0;
      }

      if ((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn +
              1024] - 1) << 5)) - 1] & i18) == wall->contents.nowall) {
        /* 北側のMAPが更新されているか判断、されていなければ書き込み */
        i19 = (int)(contor_renew_square[tempn] + 1U);
        i20 = i19;
        if ((unsigned int)i19 > 255U) {
          i20 = 255;
        }

        if (contour_map[(i20 + ((contor_renew_square[tempn + 1024] - 1) << 5)) -
            1] == 65535) {
          i20 = i19;
          if ((unsigned int)i19 > 255U) {
            i20 = 255;
          }

          contour_map[(i20 + ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
            = (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          if ((unsigned int)i19 > 255U) {
            i19 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)i19;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square[tempn + 1024];

          /* 更新マス用インデックスを増加 */
          i19 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i19 > 255U) {
            i19 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i19;
        }
      }

      /* 東側 */
      contor_renew_square_idx = contor_renew_square[tempn + 1024];
      i19 = (contor_renew_square_idx - 1) << 5;
      i20 = maze_wall[(contor_renew_square[tempn] + i19) - 1];
      if (g_direction.East <= 7) {
        i21 = (unsigned char)(1 << g_direction.East);
      } else {
        i21 = 0;
      }

      if ((i20 & i21) == wall->contents.nowall) {
        /* 東側のMAPが更新されているか判断、されていなければ書き込み */
        i22 = (int)(contor_renew_square[tempn + 1024] + 1U);
        i23 = i22;
        if ((unsigned int)i22 > 255U) {
          i23 = 255;
        }

        if (contour_map[(contor_renew_square[tempn] + ((i23 - 1) << 5)) - 1] ==
            65535) {
          i23 = i22;
          if ((unsigned int)i22 > 255U) {
            i23 = 255;
          }

          contour_map[(contor_renew_square[tempn] + ((i23 - 1) << 5)) - 1] =
            (unsigned short)(tempi + 1);
          change_flag = 1U;

          /* 更新マスを更新 */
          contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
            contor_renew_square[tempn];
          if ((unsigned int)i22 > 255U) {
            i22 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            (unsigned char)i22;

          /* 更新マス用インデックスを増加 */
          i22 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i22 > 255U) {
            i22 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i22;
        }
      }

      /* 南側 */
      if (g_direction.South <= 7) {
        i24 = (unsigned char)(1 << g_direction.South);
      } else {
        i24 = 0;
      }

      if ((i20 & i24) == wall->contents.nowall) {
        /* 南側のMAPが更新されているか判断、されていなければ書き込み */
        qY = contor_renew_square[tempn] - 1U;
        if (qY > contor_renew_square[tempn]) {
          qY = 0U;
        }

        if (contour_map[((int)qY + i19) - 1] == 65535) {
          qY = contor_renew_square[tempn] - 1U;
          if (qY > contor_renew_square[tempn]) {
            qY = 0U;
          }

          contour_map[((int)qY + i19) - 1] = (unsigned short)(tempi + 1);
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
          i19 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i19 > 255U) {
            i19 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i19;
        }
      }

      /* 西側 */
      if (g_direction.West <= 7) {
        i25 = (unsigned char)(1 << g_direction.West);
      } else {
        i25 = 0;
      }

      if ((i20 & i25) == wall->contents.nowall) {
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
          i19 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i19 > 255U) {
            i19 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i19;
        }
      }
    }

    /* ゴール更新マスの更新とインデックスのクリア */
    for (i17 = 0; i17 < 2048; i17++) {
      contor_renew_square[i17] = contor_renew_square_temp[i17];
      contor_renew_square_temp[i17] = 0U;
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
 * Arguments    : coder_internal_ref_2 *max_length
 *                const coder_internal_ref_5 *wall
 *                const coder_internal_ref_4 *search
 *                const unsigned char maze_goal[2]
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned short row_num_node[1056]
 *                unsigned short col_num_node[1056]
 *                unsigned short *start_num
 * Return Type  : void
 */
static void b_make_map_fustrun_diagonal(coder_internal_ref_2 *max_length, const
  coder_internal_ref_5 *wall, const coder_internal_ref_4 *search, const unsigned
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
  int i152;
  unsigned char row_dir_node[1056];
  unsigned char col_dir_node[1056];
  int i153;
  int i154;
  int i155;
  int i156;
  int i157;
  int row_num_node_tmp;
  int i158;
  int i159;
  int i160;
  unsigned int qY;
  int i161;
  unsigned short i;
  bool exitg1;
  unsigned char change_flag;
  int i162;
  int i163;
  int i164;
  int i165;
  unsigned int b_qY;
  int i166;
  int i167;
  int i168;
  int i169;
  int i170;
  unsigned int c_qY;
  int i171;
  int i172;
  int i173;
  int i174;
  unsigned int u9;
  int i175;
  int i176;
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
  for (i152 = 0; i152 < 1056; i152++) {
    row_num_node[i152] = MAX_uint16_T;
    col_num_node[i152] = MAX_uint16_T;
    row_dir_node[i152] = 0U;
    col_dir_node[i152] = 0U;
  }

  /* ゴールセクションが確定している場合 */
  *start_num = MAX_uint16_T;

  /* ゴールマスから、東西南北にマップを展開 */
  /* 北壁 */
  i152 = maze_goal[0] - 1;
  i153 = (maze_goal[1] + (i152 << 5)) - 1;
  if (g_direction.North <= 7) {
    i154 = (unsigned char)(1 << g_direction.North);
  } else {
    i154 = 0;
  }

  if ((maze_wall[i153] & i154) == 0) {
    /* 歩数更新 */
    i155 = (int)(maze_goal[1] + 1U);
    i156 = i155;
    if ((unsigned int)i155 > 255U) {
      i156 = 255;
    }

    row_num_node_tmp = 33 * i152;
    row_num_node[(i156 + row_num_node_tmp) - 1] = 3U;

    /* 方向追加 */
    i152 = i155;
    if ((unsigned int)i155 > 255U) {
      i152 = 255;
    }

    if (g_d_direction.North <= 7) {
      row_dir_node[(i152 + row_num_node_tmp) - 1] = (unsigned char)(1 <<
        g_d_direction.North);
    } else {
      row_dir_node[(i152 + row_num_node_tmp) - 1] = 0U;
    }

    /* 更新ノードを更新 */
    if ((unsigned int)i155 > 255U) {
      i155 = 255;
    }

    contor_renew_node_row[0] = (unsigned char)i155;
    contor_renew_node_row[1024] = maze_goal[0];

    /* 更新マス用インデックスを増加 */
    contor_renew_node_row_idx = 2U;
  }

  /* 東壁 */
  if (g_direction.East <= 7) {
    i157 = (unsigned char)(1 << g_direction.East);
  } else {
    i157 = 0;
  }

  if ((maze_wall[i153] & i157) == 0) {
    /* 歩数更新 */
    i152 = (int)(maze_goal[0] + 1U);
    i155 = i152;
    if ((unsigned int)i152 > 255U) {
      i155 = 255;
    }

    col_num_node[(maze_goal[1] + ((i155 - 1) << 5)) - 1] = 3U;

    /* 方向追加 */
    i155 = i152;
    if ((unsigned int)i152 > 255U) {
      i155 = 255;
    }

    if (g_d_direction.East <= 7) {
      col_dir_node[(maze_goal[1] + ((i155 - 1) << 5)) - 1] = (unsigned char)(1 <<
        g_d_direction.East);
    } else {
      col_dir_node[(maze_goal[1] + ((i155 - 1) << 5)) - 1] = 0U;
    }

    /* 更新ノードを更新 */
    contor_renew_node_col[0] = maze_goal[1];
    if ((unsigned int)i152 > 255U) {
      i152 = 255;
    }

    contor_renew_node_col[1024] = (unsigned char)i152;

    /* 更新マス用インデックスを増加 */
    contor_renew_node_col_idx = 2U;
  }

  /* 南壁 */
  if (g_direction.South <= 7) {
    i158 = (unsigned char)(1 << g_direction.South);
  } else {
    i158 = 0;
  }

  if ((maze_wall[i153] & i158) == 0) {
    /* 歩数更新 */
    row_num_node_tmp = (maze_goal[1] + 33 * (maze_goal[0] - 1)) - 1;
    row_num_node[row_num_node_tmp] = 3U;

    /* 方向追加 */
    if (g_d_direction.South <= 7) {
      i160 = (unsigned char)(1 << g_d_direction.South);
    } else {
      i160 = 0;
    }

    row_dir_node[row_num_node_tmp] = (unsigned char)
      (row_dir_node[row_num_node_tmp] | i160);

    /* 更新ノードを更新 */
    contor_renew_node_row[contor_renew_node_row_idx - 1] = maze_goal[1];
    contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[0];

    /* 更新マス用インデックスを増加 */
    contor_renew_node_row_idx++;
  }

  /* 西壁 */
  if (g_direction.West <= 7) {
    i159 = (unsigned char)(1 << g_direction.West);
  } else {
    i159 = 0;
  }

  if ((maze_wall[i153] & i159) == 0) {
    /* 歩数更新 */
    col_num_node[i153] = 3U;

    /* 方向追加 */
    if (g_d_direction.West <= 7) {
      i161 = (unsigned char)(1 << g_d_direction.West);
    } else {
      i161 = 0;
    }

    col_dir_node[i153] = (unsigned char)(col_dir_node[i153] | i161);

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
    i152 = contor_renew_node_row_idx;
    for (row_num_node_tmp = 0; row_num_node_tmp <= i152 - 2; row_num_node_tmp++)
    {
      /* 北側 */
      /* 壁が無い & 探索済みであるとき */
      i153 = contor_renew_node_row[row_num_node_tmp + 1024] - 1;
      i155 = (contor_renew_node_row[row_num_node_tmp] + (i153 << 5)) - 1;
      if (g_direction.North <= 7) {
        i162 = (unsigned char)(1 << g_direction.North);
      } else {
        i162 = 0;
      }

      if (((maze_wall[i155] & i162) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i163 = (unsigned char)(1 << g_direction.North);
        } else {
          i163 = 0;
        }

        if (((maze_wall_search[i155] & i163) != 0) == search->contents.known) {
          /* かつ進行方向が北向きである時 */
          i153 *= 33;
          i155 = (contor_renew_node_row[row_num_node_tmp] + i153) - 1;
          if (g_d_direction.North <= 7) {
            i168 = (unsigned char)(1 << g_d_direction.North);
          } else {
            i168 = 0;
          }

          if ((row_dir_node[i155] & i168) != 0) {
            /* かつ北のノードが更新予定値よりも大きな値の場合 */
            i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i155 + i153) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i153 + 33 * (contor_renew_node_row[row_num_node_tmp
                + 1024] - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i153 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.North);
              } else {
                row_dir_node[(i153 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i153;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i153;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i153 + 33 *
                                (contor_renew_node_row[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
                if ((unsigned int)i153 > 255U) {
                  i153 = 255;
                }

                i155 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i184 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i184 = 0;
                }

                row_dir_node[(i153 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[(i155 +
                  33 * (contor_renew_node_row[row_num_node_tmp + 1024] - 1)) - 1]
                  | i184);
              }
            }

            /* かつ進行方向が北向きでないとき */
          } else {
            /* かつ北のノードの歩数MAP値が、更新予定値より大きい場合 */
            i156 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
            if ((unsigned int)i156 > 255U) {
              i156 = 255;
            }

            b_qY = row_num_node[i155] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i156 + i153) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i153 + 33 * (contor_renew_node_row[row_num_node_tmp
                + 1024] - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i153 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.North);
              } else {
                row_dir_node[(i153 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i153;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i153;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i153 = (int)(contor_renew_node_row[row_num_node_tmp] + 1U);
              i155 = i153;
              if ((unsigned int)i153 > 255U) {
                i155 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i155 + 33 *
                                (contor_renew_node_row[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i155 = i153;
                if ((unsigned int)i153 > 255U) {
                  i155 = 255;
                  i153 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i182 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i182 = 0;
                }

                row_dir_node[(i155 + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[(i153 +
                  33 * (contor_renew_node_row[row_num_node_tmp + 1024] - 1)) - 1]
                  | i182);
              }
            }
          }
        }
      }

      /* 北東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.East <= 7) {
        i165 = (unsigned char)(1 << g_direction.East);
      } else {
        i165 = 0;
      }

      if (((maze_wall[(contor_renew_node_row[row_num_node_tmp] +
                       ((contor_renew_node_row[row_num_node_tmp + 1024] - 1) <<
                        5)) - 1] & i165) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i167 = (unsigned char)(1 << g_direction.East);
        } else {
          i167 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_row[row_num_node_tmp] +
                                ((contor_renew_node_row[row_num_node_tmp + 1024]
                 - 1) << 5)) - 1] & i167) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i171 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i171 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i171) != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i153 > 255U) {
              i153 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i153;

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i153;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i153
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i153 > 255U) {
                  i153 = 255;
                }

                i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i192 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i192 = 0;
                }

                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_row[row_num_node_tmp] + ((i155 - 1) << 5))
                  - 1] | i192);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i153 > 255U) {
              i153 = 255;
            }

            i155 = (contor_renew_node_row[row_num_node_tmp] + 33 *
                    (contor_renew_node_row[row_num_node_tmp + 1024] - 1)) - 1;
            b_qY = row_num_node[i155] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i153 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i153;

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i153;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i153 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              i156 = i153;
              if ((unsigned int)i153 > 255U) {
                i156 = 255;
              }

              b_qY = row_num_node[i155] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[row_num_node_tmp] + ((i156
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i155 = i153;
                if ((unsigned int)i153 > 255U) {
                  i155 = 255;
                  i153 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i190 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i190 = 0;
                }

                col_dir_node[(contor_renew_node_row[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_row[row_num_node_tmp] + ((i153 - 1) << 5))
                  - 1] | i190);
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

      i153 = (contor_renew_node_row[row_num_node_tmp + 1024] - 1) << 5;
      if (g_direction.East <= 7) {
        i173 = (unsigned char)(1 << g_direction.East);
      } else {
        i173 = 0;
      }

      if (((maze_wall[((int)c_qY + i153) - 1] & i173) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
        if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
          c_qY = 0U;
        }

        if (g_direction.East <= 7) {
          i177 = (unsigned char)(1 << g_direction.East);
        } else {
          i177 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i153) - 1] & i177) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i180 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i180 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i180) != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 4U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i155 - 1) << 5)) - 1] > (int)u9) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = (unsigned
                short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = 0U;
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

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i155 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                i156 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i156 > 255U) {
                  i156 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i211 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i211 = 0;
                }

                col_dir_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i156 - 1) << 5)) - 1] | i211);
              }
            }

            /* かつ進行方向が南東向きでないとき */
          } else {
            /* かつ南東のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 18U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i155 - 1) << 5)) - 1] > (int)u9) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = (unsigned
                short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = 0U;
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

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i155 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                  c_qY = 0U;
                }

                i155 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                b_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
                if (b_qY > contor_renew_node_row[row_num_node_tmp]) {
                  b_qY = 0U;
                }

                i156 = (int)(contor_renew_node_row[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i156 > 255U) {
                  i156 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i210 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i210 = 0;
                }

                col_dir_node[((int)c_qY + ((i155 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i156 - 1) << 5)) - 1] | i210);
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
        i178 = (unsigned char)(1 << g_direction.South);
      } else {
        i178 = 0;
      }

      if (((maze_wall[((int)c_qY + i153) - 1] & i178) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
        if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i183 = (unsigned char)(1 << g_direction.South);
        } else {
          i183 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i153) - 1] & i183) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南向きである時 */
          if (g_d_direction.South <= 7) {
            i189 = (unsigned char)(1 << g_d_direction.South);
          } else {
            i189 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i189) != 0) {
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
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

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
                  i209 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i209 = 0;
                }

                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[((int)
                  b_qY + 33 * (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] | i209);
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
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

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
                  i208 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i208 = 0;
                }

                row_dir_node[((int)c_qY + 33 *
                              (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[((int)
                  b_qY + 33 * (contor_renew_node_row[row_num_node_tmp + 1024] -
                               1)) - 1] | i208);
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
        i187 = (unsigned char)(1 << g_direction.West);
      } else {
        i187 = 0;
      }

      if (((maze_wall[((int)c_qY + i153) - 1] & i187) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
        if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i196 = (unsigned char)(1 << g_direction.West);
        } else {
          i196 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i153) - 1] & i196) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i202 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i202 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i202) != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
            if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[((int)c_qY + i153) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + i153) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i153) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i153) - 1] = 0U;
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
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + i153) - 1] == (int)b_qY) {
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
                  i217 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i217 = 0;
                }

                col_dir_node[((int)c_qY + i153) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i153) - 1] | i217);
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

            if (col_num_node[((int)c_qY + i153) - 1] > (int)b_qY) {
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

              col_num_node[((int)c_qY + i153) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[row_num_node_tmp] - 1U;
              if (c_qY > contor_renew_node_row[row_num_node_tmp]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i153) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i153) - 1] = 0U;
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
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

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

              if (col_num_node[((int)c_qY + i153) - 1] == (int)b_qY) {
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
                  i216 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i216 = 0;
                }

                col_dir_node[((int)c_qY + i153) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i153) - 1] | i216);
              }
            }
          }
        }
      }

      /* 北西側 */
      /* 壁が無い & 探索済みであるとき */
      i153 = (contor_renew_node_row[row_num_node_tmp] + i153) - 1;
      if (g_direction.West <= 7) {
        i198 = (unsigned char)(1 << g_direction.West);
      } else {
        i198 = 0;
      }

      if (((maze_wall[i153] & i198) != 0) == wall->contents.nowall) {
        if (g_direction.West <= 7) {
          i201 = (unsigned char)(1 << g_direction.West);
        } else {
          i201 = 0;
        }

        if (((maze_wall_search[i153] & i201) != 0) == search->contents.known) {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i205 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i205 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                             (contor_renew_node_row[row_num_node_tmp + 1024] - 1))
               - 1] & i205) != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                 (contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1)) - 1] + 4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[row_num_node_tmp] +
                              ((contor_renew_node_row[row_num_node_tmp + 1024] -
                                1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i153] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i153] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i153] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i153;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[row_num_node_tmp] +
                                ((contor_renew_node_row[row_num_node_tmp + 1024]
                                  - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i215 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i215 = 0;
                }

                col_dir_node[i153] = (unsigned char)(col_dir_node[i153] | i215);
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

            if (col_num_node[i153] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i153] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i153] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i153] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[row_num_node_tmp];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i153;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[row_num_node_tmp] + 33 *
                                   (contor_renew_node_row[row_num_node_tmp +
                                    1024] - 1)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[i153] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i214 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i214 = 0;
                }

                col_dir_node[i153] = (unsigned char)(col_dir_node[i153] | i214);
              }
            }
          }
        }
      }
    }

    /* Col_Edgeの処理[32行,33列] */
    /* 検索した座標に対し、歩数mapを更新 */
    i152 = contor_renew_node_col_idx;
    for (row_num_node_tmp = 0; row_num_node_tmp <= i152 - 2; row_num_node_tmp++)
    {
      /* 北側は壁 */
      /* 北東側 */
      /* 壁が無い & 探索済みであるとき */
      i153 = (contor_renew_node_col[row_num_node_tmp] +
              ((contor_renew_node_col[row_num_node_tmp + 1024] - 1) << 5)) - 1;
      if (g_direction.North <= 7) {
        i164 = (unsigned char)(1 << g_direction.North);
      } else {
        i164 = 0;
      }

      if (((maze_wall[i153] & i164) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i166 = (unsigned char)(1 << g_direction.North);
        } else {
          i166 = 0;
        }

        if (((maze_wall_search[i153] & i166) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i170 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i170 = 0;
          }

          if ((col_dir_node[i153] & i170) != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            if ((unsigned int)i153 > 255U) {
              i153 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 4U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            if (row_num_node[(i153 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] > (int)u9) {
              /* 歩数MAP更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i153 + 33 * (contor_renew_node_col[row_num_node_tmp
                + 1024] - 1)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i153 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(1 <<
                  g_d_direction.North_East);
              } else {
                row_dir_node[(i153 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i153;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i153;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i153 + 33 *
                                (contor_renew_node_col[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i153 > 255U) {
                  i153 = 255;
                }

                i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i188 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i188 = 0;
                }

                row_dir_node[(i153 + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node[(i155 +
                  33 * (contor_renew_node_col[row_num_node_tmp + 1024] - 1)) - 1]
                  | i188);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            i155 = i153;
            if ((unsigned int)i153 > 255U) {
              i155 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 18U;
            u9 = b_qY;
            if (b_qY > 65535U) {
              u9 = 65535U;
            }

            i156 = 33 * (contor_renew_node_col[row_num_node_tmp + 1024] - 1);
            if (row_num_node[(i155 + i156) - 1] > (int)u9) {
              /* 歩数MAP更新(重みづけあり) */
              i155 = i153;
              if ((unsigned int)i153 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i155 + i156) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i155 = i153;
              if ((unsigned int)i153 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i155 + i156) - 1] = (unsigned char)(1 <<
                  g_d_direction.North_East);
              } else {
                row_dir_node[(i155 + i156) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i153;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[row_num_node_tmp + 1024];

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i153;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i155 = i153;
              if ((unsigned int)i153 > 255U) {
                i155 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i155 + i156) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i155 = i153;
                if ((unsigned int)i153 > 255U) {
                  i155 = 255;
                  i153 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i186 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i186 = 0;
                }

                row_dir_node[(i155 + i156) - 1] = (unsigned char)(row_dir_node
                  [(i153 + i156) - 1] | i186);
              }
            }
          }
        }
      }

      /* 東側 */
      /* 壁が無い & 探索済みであるとき */
      contor_renew_node_row_idx = contor_renew_node_col[row_num_node_tmp + 1024];
      i153 = (contor_renew_node_col[row_num_node_tmp] +
              ((contor_renew_node_row_idx - 1) << 5)) - 1;
      if (g_direction.East <= 7) {
        i169 = (unsigned char)(1 << g_direction.East);
      } else {
        i169 = 0;
      }

      if (((maze_wall[i153] & i169) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i172 = (unsigned char)(1 << g_direction.East);
        } else {
          i172 = 0;
        }

        if (((maze_wall_search[i153] & i172) != 0) == search->contents.known) {
          /* かつ進行方向が東向きである時 */
          if (g_d_direction.East <= 7) {
            i175 = (unsigned char)(1 << g_d_direction.East);
          } else {
            i175 = 0;
          }

          if ((col_dir_node[i153] & i175) != 0) {
            /* かつ東のノードが更新予定値よりも大きな値の場合 */
            i155 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i155 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i155 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i155 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              i155 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
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

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i155 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i155
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i155 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                i156 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
                if ((unsigned int)i156 > 255U) {
                  i156 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i200 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i200 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i155 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + ((i156 - 1) << 5))
                  - 1] | i200);
              }
            }

            /* かつ進行方向が東向きでないとき */
          } else {
            /* かつ東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i155 = (int)(contor_renew_node_col[row_num_node_tmp + 1024] + 1U);
            i156 = i155;
            if ((unsigned int)i155 > 255U) {
              i156 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i156 -
                   1) << 5)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i156 = i155;
              if ((unsigned int)i155 > 255U) {
                i156 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i156 - 1)
                << 5)) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i156 = i155;
              if ((unsigned int)i155 > 255U) {
                i156 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i156 -
                  1) << 5)) - 1] = (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i156 -
                  1) << 5)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
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

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i156 = i155;
              if ((unsigned int)i155 > 255U) {
                i156 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[row_num_node_tmp] + ((i156
                     - 1) << 5)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i156 = i155;
                if ((unsigned int)i155 > 255U) {
                  i156 = 255;
                  i155 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i197 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i197 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + ((i156 -
                  1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + ((i155 - 1) << 5))
                  - 1] | i197);
              }
            }
          }
        }
      }

      /* 南東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.South <= 7) {
        i174 = (unsigned char)(1 << g_direction.South);
      } else {
        i174 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] +
                       ((contor_renew_node_col[row_num_node_tmp + 1024] - 1) <<
                        5)) - 1] & i174) != 0) == wall->contents.nowall) {
        if (g_direction.South <= 7) {
          i176 = (unsigned char)(1 << g_direction.South);
        } else {
          i176 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] +
                                ((contor_renew_node_col[row_num_node_tmp + 1024]
                 - 1) << 5)) - 1] & i176) != 0) == search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i179 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i179 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[row_num_node_tmp] +
                             ((contor_renew_node_col[row_num_node_tmp + 1024] -
                               1) << 5)) - 1] & i179) != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 4U;
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
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                (contor_renew_node_col[row_num_node_tmp + 1024]
                                 - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i194 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i194 = 0;
                }

                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              (contor_renew_node_col[row_num_node_tmp + 1024] -
                               1)) - 1] = (unsigned char)(row_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + 33 *
                    (contor_renew_node_col[row_num_node_tmp + 1024] - 1)) - 1] |
                  i194);
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

            i155 = (contor_renew_node_col[row_num_node_tmp] + 33 *
                    (contor_renew_node_col[row_num_node_tmp + 1024] - 1)) - 1;
            if (row_num_node[i155] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[i155] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.South_East <= 7) {
                row_dir_node[i155] = (unsigned char)(1 <<
                  g_d_direction.South_East);
              } else {
                row_dir_node[i155] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[row_num_node_tmp];
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row_idx;

              /* 更新マス用インデックスを増加 */
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[i153] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[i155] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i195 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i195 = 0;
                }

                row_dir_node[i155] = (unsigned char)(row_dir_node[i155] | i195);
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
        i181 = (unsigned char)(1 << g_direction.South);
      } else {
        i181 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] + (((int)c_qY - 1)
              << 5)) - 1] & i181) != 0) == wall->contents.nowall) {
        c_qY = contor_renew_node_row_idx - 1U;
        if (c_qY > contor_renew_node_row_idx) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i185 = (unsigned char)(1 << g_direction.South);
        } else {
          i185 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] + (((int)
                 c_qY - 1) << 5)) - 1] & i185) != 0) == search->contents.known)
        {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i193 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i193 = 0;
          }

          if ((col_dir_node[i153] & i193) != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 4U;
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
                1024] - 1) << 5)) - 1] + 4U;
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
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 4U;
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
                  i213 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i213 = 0;
                }

                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                 ((int)b_qY - 1)) - 1] | i213);
              }
            }

            /* かつ進行方向が南西向きでないとき */
          } else {
            /* かつ南西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i153] + 18U;
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
              i155 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i155;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i153] + 18U;
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
                  i212 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i212 = 0;
                }

                row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                              ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(contor_renew_node_col[row_num_node_tmp] + 33 *
                                 ((int)b_qY - 1)) - 1] | i212);
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
        i191 = (unsigned char)(1 << g_direction.West);
      } else {
        i191 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] + (((int)c_qY - 1)
              << 5)) - 1] & i191) != 0) == wall->contents.nowall) {
        c_qY = contor_renew_node_row_idx - 1U;
        if (c_qY > contor_renew_node_row_idx) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i199 = (unsigned char)(1 << g_direction.West);
        } else {
          i199 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] + (((int)
                 c_qY - 1) << 5)) - 1] & i199) != 0) == search->contents.known)
        {
          /* かつ進行方向が西向きである時 */
          if (g_d_direction.West <= 7) {
            i204 = (unsigned char)(1 << g_d_direction.West);
          } else {
            i204 = 0;
          }

          if ((col_dir_node[i153] & i204) != 0) {
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
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

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
                  i219 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i219 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + (((int)b_qY - 1) <<
                  5)) - 1] | i219);
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
              i155 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i155 > 255U) {
                i155 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i155;

              /* かつ西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i153] + 18U;
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
                  i218 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i218 = 0;
                }

                col_dir_node[(contor_renew_node_col[row_num_node_tmp] + (((int)
                  c_qY - 1) << 5)) - 1] = (unsigned char)(col_dir_node
                  [(contor_renew_node_col[row_num_node_tmp] + (((int)b_qY - 1) <<
                  5)) - 1] | i218);
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
        i203 = (unsigned char)(1 << g_direction.North);
      } else {
        i203 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[row_num_node_tmp] + (((int)c_qY - 1)
              << 5)) - 1] & i203) != 0) == wall->contents.nowall) {
        c_qY = contor_renew_node_row_idx - 1U;
        if (c_qY > contor_renew_node_row_idx) {
          c_qY = 0U;
        }

        if (g_direction.North <= 7) {
          i206 = (unsigned char)(1 << g_direction.North);
        } else {
          i206 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[row_num_node_tmp] + (((int)
                 c_qY - 1) << 5)) - 1] & i206) != 0) == search->contents.known)
        {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i207 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i207 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[row_num_node_tmp] +
                             ((contor_renew_node_col[row_num_node_tmp + 1024] -
                               1) << 5)) - 1] & i207) != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            if ((unsigned int)i153 > 255U) {
              i153 = 255;
            }

            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                 ((contor_renew_node_col[row_num_node_tmp + 1024]
              - 1) << 5)) - 1] + 4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i153 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i153;
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i153;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[row_num_node_tmp] +
                                   ((contor_renew_node_col[row_num_node_tmp +
                1024] - 1) << 5)) - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i153 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i153 > 255U) {
                  i153 = 255;
                }

                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i221 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i221 = 0;
                }

                row_dir_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i155 + 33 * ((int)b_qY - 1)) - 1] | i221);
              }
            }

            /* かつ進行方向が北西向きでないとき */
          } else {
            /* かつ北西のノードの歩数MAP値が、更新予定値より大きい場合 */
            i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
            if ((unsigned int)i155 > 255U) {
              i155 = 255;
            }

            c_qY = contor_renew_node_row_idx - 1U;
            if (c_qY > contor_renew_node_row_idx) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i153] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i155 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
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

              row_num_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i153;
              c_qY = contor_renew_node_row_idx - 1U;
              if (c_qY > contor_renew_node_row_idx) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i153 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i153 > 255U) {
                i153 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i153;

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

              b_qY = col_num_node[i153] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i155 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                i153 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i153 > 255U) {
                  i153 = 255;
                }

                c_qY = contor_renew_node_row_idx - 1U;
                if (c_qY > contor_renew_node_row_idx) {
                  c_qY = 0U;
                }

                i155 = (int)(contor_renew_node_col[row_num_node_tmp] + 1U);
                if ((unsigned int)i155 > 255U) {
                  i155 = 255;
                }

                b_qY = contor_renew_node_row_idx - 1U;
                if (b_qY > contor_renew_node_row_idx) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i220 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i220 = 0;
                }

                row_dir_node[(i153 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i155 + 33 * ((int)b_qY - 1)) - 1] | i220);
              }
            }
          }
        }
      }
    }

    /* ゴール更新ノードの更新とインデックスのクリア */
    contor_renew_node_col_idx = contor_renew_node_col_idx_temp;
    contor_renew_node_col_idx_temp = 1U;
    for (i152 = 0; i152 < 2048; i152++) {
      contor_renew_node_col[i152] = contor_renew_node_col_temp[i152];
      contor_renew_node_col_temp[i152] = 0U;
      contor_renew_node_row[i152] = contor_renew_node_row_temp[i152];
      contor_renew_node_row_temp[i152] = 0U;
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
 * Arguments    : const coder_internal_ref_5 *wall
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_4 *search
 *                const coder_internal_ref_1 *maze_goal
 *                const coder_internal_ref_3 *adachi_search_mode
 *                unsigned char *current_x
 *                unsigned char *current_y
 *                unsigned char *current_dir
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 *                const unsigned char exploration_goal[2]
 *                unsigned char *start_flg
 *                unsigned char stop_flg
 *                unsigned char goal_after_flg
 *                unsigned char adachi_s_mode
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void b_search_adachi(const coder_internal_ref_5 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_4 *search, const coder_internal_ref_1
  *maze_goal, const coder_internal_ref_3 *adachi_search_mode, unsigned char
  *current_x, unsigned char *current_y, unsigned char *current_dir, unsigned
  char maze_row_size, unsigned char maze_col_size, unsigned char maze_wall[1024],
  unsigned char maze_wall_search[1024], const unsigned char exploration_goal[2],
  unsigned char *start_flg, unsigned char stop_flg, unsigned char goal_after_flg,
  unsigned char adachi_s_mode, unsigned short contour_map[1024])
{
  unsigned char goal_flg;
  unsigned char contour_flg;
  int exitg1;
  unsigned char next_dir;
  int q0;
  unsigned int qY;

  /*     %% search_adachi 足立法での探索 */
  /* local変数宣言 */
  goal_flg = 0U;

  /* ゴール判定フラグ */
  /* 壁情報更新確認用変数 */
  contour_flg = 0U;

  /*      search_start_x = current_x %探索開始時x */
  /*      search_start_y = current_y %探索開始時y */
  /* 初回のコンターマップ作製 */
  b_make_map_find(wall, exploration_goal, maze_wall, *current_x, *current_y,
                  contour_map);
  do {
    exitg1 = 0;

    /* 壁情報取得 */
    /* ゴール直後は壁情報を更新しない */
    if (goal_after_flg != 1) {
      next_dir = maze_wall[(*current_y + ((*current_x - 1) << 5)) - 1];
      wall_set(wall, wall_flg, search, maze_goal, maze_row_size, maze_col_size, *
               current_x, *current_y, *current_dir, maze_wall, maze_wall_search);

      /* 壁情報が更新されれば、コンター更新のフラグを立てる。 */
      if (next_dir != maze_wall[(*current_y + ((*current_x - 1) << 5)) - 1]) {
        contour_flg = 1U;
      }
    } else {
      /* ゴール直後のとき */
      goal_after_flg = 0U;

      /* ゴール直後フラグをクリア */
    }

    /*  等高線MAP生成 */
    /* 壁情報に変更があった場合のみ */
    if (contour_flg != 0) {
      b_make_map_find(wall, exploration_goal, maze_wall, *current_x, *current_y,
                      contour_map);
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
      next_dir = get_nextdir2(*current_x, *current_y, maze_wall, contour_map);

      /*  現在方向と進行方向に応じた処理 */
      q0 = (int)(4U + next_dir);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      qY = (unsigned int)q0 - *current_dir;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      next_dir = b_rem((unsigned char)qY);
      if (l_direction.front == next_dir) {
        q0 = 0;
      } else if (l_direction.right == next_dir) {
        q0 = 1;
      } else if (l_direction.back == next_dir) {
        q0 = 2;
      } else if (l_direction.left == next_dir) {
        q0 = 3;
      } else {
        q0 = -1;
      }

      switch (q0) {
       case 0:
        move_step(current_x, current_y, *current_dir);

        /* disp("front") */
        m_move_front(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 1:
        turn_clk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* disp("right") */
        m_move_right(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 2:
        turn_180deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* disp("back") */
        m_move_back(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 3:
        turn_conclk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* disp("left") */
        m_move_left(*start_flg, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;
      }

      /* for code generation */
    }
  } while (exitg1 == 0);

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
  int i148;
  int exitg1;
  unsigned char goal_flag;
  int i;
  bool guard1 = false;
  int i149;
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
  i148 = goal_size;
  do {
    exitg1 = 0;
    goal_flag = 0U;
    for (i = 0; i < i148; i++) {
      /* 現在のノードが行方向の時 */
      if (*goal_matrix_dir == matrix_dir.Row) {
        /*  %x */
        if (maze_goal[i] == goal_node[1]) {
          guard1 = false;
          if (maze_goal[i + 9] == goal_node[0]) {
            guard1 = true;
          } else {
            i149 = (int)(maze_goal[i + 9] + 1U);
            if ((unsigned int)i149 > 255U) {
              i149 = 255;
            }

            if (i149 == goal_node[0]) {
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
              i149 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              if (col_num_node[(goal_node[0] + ((i149 - 1) << 5)) - 1] == 65535)
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

                i149 = (int)(goal_node[1] + 1U);
                if ((unsigned int)i149 > 255U) {
                  i149 = 255;
                }

                if (col_num_node[((int)qY + ((i149 - 1) << 5)) - 1] == 65535) {
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
          i149 = (int)(maze_goal[i] + 1U);
          if ((unsigned int)i149 > 255U) {
            i149 = 255;
          }

          if (i149 == goal_node[1]) {
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
            i149 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i149 > 255U) {
              i149 = 255;
            }

            if (row_num_node[(i149 + 33 * (goal_node[1] - 1)) - 1] == 65535) {
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
            i149 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i149 > 255U) {
              i149 = 255;
            }

            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            if (row_num_node[(i149 + 33 * ((int)qY - 1)) - 1] == 65535) {
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
        i149 = *goal_dir + i;
        if (i149 > 255) {
          i149 = 255;
        }

        goal_flag = (unsigned char)(i149 % 8);

        /* 現在のノードが行方向の時 */
        if (*goal_matrix_dir == matrix_dir.Row) {
          if (goal_flag == g_d_direction.North) {
            i149 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i149 > 255U) {
              i149 = 255;
            }

            if (row_num_node[(i149 + 33 * (goal_node[1] - 1)) - 1] < map_min) {
              /* 最小値を更新 */
              i149 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              map_min = row_num_node[(i149 + 33 * (goal_node[1] - 1)) - 1];

              /* 現在ノードの進行方向を北向きに */
              *goal_dir = g_d_direction.North;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Row;
              i149 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              next_node_idx_0 = (unsigned char)i149;
              next_node_idx_1 = goal_node[1];
            }
          } else if (goal_flag == g_d_direction.North_East) {
            i149 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i149 > 255U) {
              i149 = 255;
            }

            if (col_num_node[(goal_node[0] + ((i149 - 1) << 5)) - 1] < map_min)
            {
              /* 最小値を更新 */
              i149 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              map_min = col_num_node[(goal_node[0] + ((i149 - 1) << 5)) - 1];

              /* 現在ノードの進行方向を北向きに */
              *goal_dir = g_d_direction.North_East;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Col;
              next_node_idx_0 = goal_node[0];
              i149 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              next_node_idx_1 = (unsigned char)i149;
            }
          } else if (goal_flag != g_d_direction.East) {
            if (goal_flag == g_d_direction.South_East) {
              qY = goal_node[0] - 1U;
              if (qY > goal_node[0]) {
                qY = 0U;
              }

              i149 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              if (col_num_node[((int)qY + ((i149 - 1) << 5)) - 1] < map_min) {
                /* 最小値を更新 */
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                i149 = (int)(goal_node[1] + 1U);
                if ((unsigned int)i149 > 255U) {
                  i149 = 255;
                }

                map_min = col_num_node[((int)qY + ((i149 - 1) << 5)) - 1];

                /* 現在ノードの進行方向を北向きに */
                *goal_dir = g_d_direction.South_East;

                /* 進行方向先の座標、行列の方向を更新 */
                next_matrix_dir = matrix_dir.Col;
                qY = goal_node[0] - 1U;
                if (qY > goal_node[0]) {
                  qY = 0U;
                }

                next_node_idx_0 = (unsigned char)qY;
                i149 = (int)(goal_node[1] + 1U);
                if ((unsigned int)i149 > 255U) {
                  i149 = 255;
                }

                next_node_idx_1 = (unsigned char)i149;
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
            i149 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i149 > 255U) {
              i149 = 255;
            }

            if (row_num_node[(i149 + 33 * (goal_node[1] - 1)) - 1] < map_min) {
              /* 最小値を更新 */
              i149 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              map_min = row_num_node[(i149 + 33 * (goal_node[1] - 1)) - 1];

              /* 現在ノードの進行方向を北東向きに */
              *goal_dir = g_d_direction.North_East;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Row;
              i149 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              next_node_idx_0 = (unsigned char)i149;
              next_node_idx_1 = goal_node[1];
            }
          } else if (goal_flag == g_d_direction.East) {
            i149 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i149 > 255U) {
              i149 = 255;
            }

            if (col_num_node[(goal_node[0] + ((i149 - 1) << 5)) - 1] < map_min)
            {
              /* 最小値を更新 */
              i149 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              map_min = col_num_node[(goal_node[0] + ((i149 - 1) << 5)) - 1];

              /* 現在ノードの進行方向を東向きに */
              *goal_dir = g_d_direction.East;

              /* 進行方向先の座標、行列の方向を更新 */
              next_matrix_dir = matrix_dir.Col;
              next_node_idx_0 = goal_node[0];
              i149 = (int)(goal_node[1] + 1U);
              if ((unsigned int)i149 > 255U) {
                i149 = 255;
              }

              next_node_idx_1 = (unsigned char)i149;
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
                i149 = (int)(goal_node[0] + 1U);
                if ((unsigned int)i149 > 255U) {
                  i149 = 255;
                }

                qY = goal_node[1] - 1U;
                if (qY > goal_node[1]) {
                  qY = 0U;
                }

                if (row_num_node[(i149 + 33 * ((int)qY - 1)) - 1] < map_min) {
                  /* 最小値を更新 */
                  i149 = (int)(goal_node[0] + 1U);
                  if ((unsigned int)i149 > 255U) {
                    i149 = 255;
                  }

                  qY = goal_node[1] - 1U;
                  if (qY > goal_node[1]) {
                    qY = 0U;
                  }

                  map_min = row_num_node[(i149 + 33 * ((int)qY - 1)) - 1];

                  /* 現在ノードの進行方向を北西向きに */
                  *goal_dir = g_d_direction.North_West;

                  /* 進行方向先の座標、行列の方向を更新 */
                  next_matrix_dir = matrix_dir.Row;
                  i149 = (int)(goal_node[0] + 1U);
                  if ((unsigned int)i149 > 255U) {
                    i149 = 255;
                  }

                  next_node_idx_0 = (unsigned char)i149;
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
  int i150;
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
      i150 = (int)(goal_node[0] + 1U);
      ex = i150;
      if ((unsigned int)i150 > 255U) {
        ex = 255;
      }

      if (goal_judge(maze_goal, goal_node[1], (unsigned char)ex) != 0.0) {
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_section[0] = (unsigned char)i150;
        goal_section[1] = goal_node[1];

        /* (y,x) */
        goal_node2[0] = (unsigned char)i150;
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
      i150 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i150 > 255U) {
        i150 = 255;
      }

      ex = (int)(goal_node[0] + 1U);
      if ((unsigned int)ex > 255U) {
        ex = 255;
      }

      if (goal_judge(maze_goal, (unsigned char)i150, (unsigned char)ex) != 0.0)
      {
        i150 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_section[0] = (unsigned char)i150;
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_section[1] = (unsigned char)i150;

        /* (y,x) */
        i150 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_node2[0] = (unsigned char)i150;
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_node2[1] = (unsigned char)i150;
      } else {
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        if (goal_judge(maze_goal, (unsigned char)i150, goal_node[0]) != 0.0) {
          goal_section[0] = goal_node[0];
          i150 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_section[1] = (unsigned char)i150;

          /* (y,x) */
          goal_node2[0] = goal_node[0];
          i150 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_node2[1] = (unsigned char)i150;
          *goal_matrix_dir2 = matrix_dir.Col;
        } else {
          i150 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          if (goal_judge(maze_goal, goal_node[1], (unsigned char)i150) != 0.0) {
            i150 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            goal_section[0] = (unsigned char)i150;
            goal_section[1] = goal_node[1];

            /* (y,x) */
            i150 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            goal_node2[0] = (unsigned char)i150;
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
      i150 = (int)(goal_node[1] + 1U);
      ex = i150;
      if ((unsigned int)i150 > 255U) {
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
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_section[1] = (unsigned char)i150;

        /* (y,x) */
        goal_node2[0] = (unsigned char)ex;
        goal_node2[1] = (unsigned char)i150;
        *goal_matrix_dir2 = matrix_dir.Col;
      } else {
        ex = (int)(goal_node[0] + 1U);
        if ((unsigned int)ex > 255U) {
          ex = 255;
        }

        if (goal_judge(maze_goal, goal_node[1], (unsigned char)ex) != 0.0) {
          i150 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_section[0] = (unsigned char)i150;
          goal_section[1] = goal_node[1];

          /* (y,x) */
          i150 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_node2[0] = (unsigned char)i150;
          goal_node2[1] = goal_node[1];
        } else {
          ex = i150;
          if ((unsigned int)i150 > 255U) {
            ex = 255;
          }

          if (goal_judge(maze_goal, (unsigned char)ex, goal_node[0]) != 0.0) {
            goal_section[0] = goal_node[0];
            ex = i150;
            if ((unsigned int)i150 > 255U) {
              ex = 255;
            }

            goal_section[1] = (unsigned char)ex;

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            goal_node2[1] = (unsigned char)i150;
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
      i150 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i150 > 255U) {
        i150 = 255;
      }

      if (goal_judge(maze_goal, (unsigned char)i150, goal_node[0]) != 0.0) {
        goal_section[0] = goal_node[0];
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_section[1] = (unsigned char)i150;

        /* (y,x) */
        goal_node2[0] = goal_node[0];
        goal_node2[1] = (unsigned char)i150;
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
      i150 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i150 > 255U) {
        i150 = 255;
      }

      qY = goal_node[0] - 2U;
      if (qY > goal_node[0]) {
        qY = 0U;
      }

      if (goal_judge(maze_goal, (unsigned char)i150, (unsigned char)qY) != 0.0)
      {
        qY = goal_node[0] - 2U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_section[1] = (unsigned char)i150;

        /* (y,x) */
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_node2[0] = (unsigned char)qY;
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_node2[1] = (unsigned char)i150;
      } else {
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        if (goal_judge(maze_goal, (unsigned char)i150, (unsigned char)qY) != 0.0)
        {
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_section[0] = (unsigned char)qY;
          i150 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_section[1] = (unsigned char)i150;

          /* (y,x) */
          qY = goal_node[0] - 1U;
          if (qY > goal_node[0]) {
            qY = 0U;
          }

          goal_node2[0] = (unsigned char)qY;
          i150 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_node2[1] = (unsigned char)i150;
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
      i150 = (int)(goal_node[1] + 1U);
      if ((unsigned int)i150 > 255U) {
        i150 = 255;
      }

      qY = goal_node[0] - 1U;
      if (qY > goal_node[0]) {
        qY = 0U;
      }

      if (goal_judge(maze_goal, (unsigned char)i150, (unsigned char)qY) != 0.0)
      {
        qY = goal_node[0] - 1U;
        if (qY > goal_node[0]) {
          qY = 0U;
        }

        goal_section[0] = (unsigned char)qY;
        i150 = (int)(goal_node[1] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        goal_section[1] = (unsigned char)i150;

        /* (y,x) */
        goal_node2[0] = (unsigned char)qY;
        goal_node2[1] = (unsigned char)i150;
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
          i150 = (int)(goal_node[1] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          if (goal_judge(maze_goal, (unsigned char)i150, goal_node[0]) != 0.0) {
            goal_section[0] = goal_node[0];
            i150 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            goal_section[1] = (unsigned char)i150;

            /* (y,x) */
            goal_node2[0] = goal_node[0];
            i150 = (int)(goal_node[1] + 1U);
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            goal_node2[1] = (unsigned char)i150;
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

        i150 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)i150) != 0.0)
        {
          i150 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_section[0] = (unsigned char)i150;
          qY = goal_node[1] - 1U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_section[1] = (unsigned char)qY;

          /* (y,x) */
          goal_node2[0] = (unsigned char)i150;
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
            i150 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            if (goal_judge(maze_goal, goal_node[1], (unsigned char)i150) != 0.0)
            {
              i150 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i150 > 255U) {
                i150 = 255;
              }

              goal_section[0] = (unsigned char)i150;
              goal_section[1] = goal_node[1];

              /* (y,x) */
              i150 = (int)(goal_node[0] + 1U);
              if ((unsigned int)i150 > 255U) {
                i150 = 255;
              }

              goal_node2[0] = (unsigned char)i150;
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

        i150 = (int)(goal_node[0] + 1U);
        if ((unsigned int)i150 > 255U) {
          i150 = 255;
        }

        if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)i150) != 0.0)
        {
          i150 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_section[0] = (unsigned char)i150;
          qY = goal_node[1] - 2U;
          if (qY > goal_node[1]) {
            qY = 0U;
          }

          goal_section[1] = (unsigned char)qY;

          /* (y,x) */
          i150 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          goal_node2[0] = (unsigned char)i150;
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

          i150 = (int)(goal_node[0] + 1U);
          if ((unsigned int)i150 > 255U) {
            i150 = 255;
          }

          if (goal_judge(maze_goal, (unsigned char)qY, (unsigned char)i150) !=
              0.0) {
            i150 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            goal_section[0] = (unsigned char)i150;
            qY = goal_node[1] - 1U;
            if (qY > goal_node[1]) {
              qY = 0U;
            }

            goal_section[1] = (unsigned char)qY;

            /* (y,x) */
            i150 = (int)(goal_node[0] + 1U);
            if ((unsigned int)i150 > 255U) {
              i150 = 255;
            }

            goal_node2[0] = (unsigned char)i150;
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
            for (i150 = 0; i150 < 9; i150++) {
              uv0[i150] = (unsigned char)qY;
              uv0[9 + i150] = goal_node[0];
            }

            for (i150 = 0; i150 < 18; i150++) {
              temp1[i150] = (maze_goal[i150] == uv0[i150]);
            }

            /* x,yともに一致するか確認、一致なら1を返す */
            for (i150 = 0; i150 < 9; i150++) {
              varargin_1[i150] = (signed char)(temp1[i150] * temp1[9 + i150]);
            }

            ex = varargin_1[0];
            for (k = 0; k < 8; k++) {
              i150 = varargin_1[k + 1];
              if (ex < i150) {
                ex = i150;
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
 * Arguments    : const coder_internal_ref *goal_size
 *                const coder_internal_ref_5 *wall
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                const unsigned short contour_map[1024]
 *                const unsigned char maze_goal[18]
 *                unsigned short max_length
 *                unsigned char unexp_square[1024]
 *                unsigned char *unexp_square_idx
 * Return Type  : void
 */
static void fust_run(const coder_internal_ref *goal_size, const
                     coder_internal_ref_5 *wall, const unsigned char maze_wall
                     [1024], const unsigned char maze_wall_search[1024], const
                     unsigned short contour_map[1024], const unsigned char
                     maze_goal[18], unsigned short max_length, unsigned char
                     unexp_square[1024], unsigned char *unexp_square_idx)
{
  unsigned char goal_flag;
  unsigned short little;
  unsigned char temp_x;
  unsigned char temp_y;
  unsigned char temp_dir;
  unsigned char next_dir;
  int tempk;
  bool exitg1;
  int i53;
  int i54;
  int i55;
  int b_unexp_square_idx;
  int i56;
  int i57;
  unsigned short u3;
  int i58;
  int i59;
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
    /* 現在位置が未探索マスか判定 */
    i53 = temp_y + ((temp_x - 1) << 5);
    i54 = i53 - 1;
    if (maze_wall_search[i54] != 15) {
      /* 未探索マスであれば、記録する。インデックスを増加させる。 */
      i55 = (int)(*unexp_square_idx + 1U);
      b_unexp_square_idx = i55;
      if ((unsigned int)i55 > 255U) {
        b_unexp_square_idx = 255;
      }

      b_unexp_square_idx--;
      unexp_square[b_unexp_square_idx] = temp_y;
      unexp_square[512 + b_unexp_square_idx] = temp_x;
      if ((unsigned int)i55 > 255U) {
        i55 = 255;
      }

      *unexp_square_idx = (unsigned char)i55;
    }

    /* 現在位置がゴールか判定 */
    i55 = goal_size->contents;
    for (b_unexp_square_idx = 0; b_unexp_square_idx < i55; b_unexp_square_idx++)
    {
      if ((temp_x == maze_goal[b_unexp_square_idx]) && (temp_y ==
           maze_goal[b_unexp_square_idx + 9])) {
        goal_flag = 1U;
      }
    }

    if (goal_flag == 1) {
      /* ゴールかつ走行時、停止処理を実施 */
      exitg1 = true;
    } else {
      /*         %%進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      /* 北側の壁のありなし */
      if (g_direction.North <= 7) {
        i56 = (unsigned char)(1 << g_direction.North);
      } else {
        i56 = 0;
      }

      if (((maze_wall[i54] & i56) == wall->contents.nowall) && (contour_map[i53]
           < little)) {
        /* 北側の等高線mapが閾値より低ければ、 */
        /* 閾値を北側の等高map値に変更 */
        little = contour_map[temp_y + ((temp_x - 1) << 5)];

        /* 北側を進行方向に変更y */
        next_dir = g_direction.North;
      }

      /* 東側 */
      if (g_direction.East <= 7) {
        i57 = (unsigned char)(1 << g_direction.East);
      } else {
        i57 = 0;
      }

      if ((maze_wall[(temp_y + ((temp_x - 1) << 5)) - 1] & i57) ==
          wall->contents.nowall) {
        u3 = contour_map[(temp_y + (temp_x << 5)) - 1];
        if (u3 < little) {
          little = u3;
          next_dir = g_direction.East;
        }
      }

      /* 南側 */
      if (g_direction.South <= 7) {
        i58 = (unsigned char)(1 << g_direction.South);
      } else {
        i58 = 0;
      }

      if ((maze_wall[(temp_y + ((temp_x - 1) << 5)) - 1] & i58) ==
          wall->contents.nowall) {
        u3 = contour_map[i53 - 2];
        if (u3 < little) {
          little = u3;
          next_dir = g_direction.South;
        }
      }

      /* 西側 */
      if (g_direction.West <= 7) {
        i59 = (unsigned char)(1 << g_direction.West);
      } else {
        i59 = 0;
      }

      if ((maze_wall[(temp_y + ((temp_x - 1) << 5)) - 1] & i59) ==
          wall->contents.nowall) {
        u3 = contour_map[(temp_y + ((temp_x - 2) << 5)) - 1];
        if (u3 < little) {
          little = u3;
          next_dir = g_direction.West;
        }
      }

      /* 走行時、探索壁情報に応じて、壁フラグをセット */
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
        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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

        /* disp("front") */
        break;

       case 1:
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_clk_90deg 時計周りに90度ターンする関数 */
        i53 = (int)(4U + temp_dir);
        if ((unsigned int)i53 > 255U) {
          i53 = 255;
        }

        i53++;
        if ((unsigned int)i53 > 255U) {
          i53 = 255;
        }

        temp_dir = (unsigned char)(i53 % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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

        /* disp("right") */
        break;

       case 2:
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*     %% turn_180deg 180度ターンする関数 */
        i53 = (int)(4U + temp_dir);
        if ((unsigned int)i53 > 255U) {
          i53 = 255;
        }

        temp_dir = (unsigned char)((i53 - 2) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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

        /* disp("back") */
        break;

       case 3:
        /* 入力　現在方向 */
        /* 出力　現在方向 */
        /*     %% turn_conclk_90deg 反時計周りに90度回る関数 */
        i53 = (int)(4U + temp_dir);
        if ((unsigned int)i53 > 255U) {
          i53 = 255;
        }

        temp_dir = (unsigned char)((i53 - 1) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*     %% move_step 一マス前進する関数 */
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

  /*          pause(0.01) */
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
  int i222;
  unsigned char temp;
  int i223;
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
      i222 = current_move_dir + k;
      if (i222 > 255) {
        i222 = 255;
      }

      temp = (unsigned char)(i222 % 8);

      /* 現在のノードが行方向の時 */
      if (current_matrix_dir == matrix_dir.Row) {
        if (temp == g_d_direction.North) {
          i222 = (int)(current_node[0] + 1U);
          i223 = i222;
          if ((unsigned int)i222 > 255U) {
            i223 = 255;
          }

          if (row_num_node[(i223 + 33 * (current_node[1] - 1)) - 1] < map_min) {
            /* 最小値を更新 */
            i223 = i222;
            if ((unsigned int)i222 > 255U) {
              i223 = 255;
            }

            map_min = row_num_node[(i223 + 33 * (current_node[1] - 1)) - 1];

            /* 現在ノードの進行方向を北向きに */
            *next_dir = g_d_direction.North;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Row;
            if ((unsigned int)i222 > 255U) {
              i222 = 255;
            }

            next_node[0] = (unsigned char)i222;
            next_node[1] = current_node[1];
          }
        } else if (temp == g_d_direction.North_East) {
          i222 = (int)(current_node[1] + 1U);
          i223 = i222;
          if ((unsigned int)i222 > 255U) {
            i223 = 255;
          }

          if (col_num_node[(current_node[0] + ((i223 - 1) << 5)) - 1] < map_min)
          {
            /* 最小値を更新 */
            i223 = i222;
            if ((unsigned int)i222 > 255U) {
              i223 = 255;
            }

            map_min = col_num_node[(current_node[0] + ((i223 - 1) << 5)) - 1];

            /* 現在ノードの進行方向を東北向きに */
            *next_dir = g_d_direction.North_East;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Col;
            next_node[0] = current_node[0];
            if ((unsigned int)i222 > 255U) {
              i222 = 255;
            }

            next_node[1] = (unsigned char)i222;
          }
        } else if (temp != g_d_direction.East) {
          if (temp == g_d_direction.South_East) {
            qY = current_node[0] - 1U;
            if (qY > current_node[0]) {
              qY = 0U;
            }

            i222 = (int)(current_node[1] + 1U);
            if ((unsigned int)i222 > 255U) {
              i222 = 255;
            }

            if (col_num_node[((int)qY + ((i222 - 1) << 5)) - 1] < map_min) {
              /* 最小値を更新 */
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              i222 = (int)(current_node[1] + 1U);
              if ((unsigned int)i222 > 255U) {
                i222 = 255;
              }

              map_min = col_num_node[((int)qY + ((i222 - 1) << 5)) - 1];

              /* 現在ノードの進行方向を南東向きに */
              *next_dir = g_d_direction.South_East;

              /* 進行方向先の座標、行列の方向を更新 */
              *next_node_property = matrix_dir.Col;
              qY = current_node[0] - 1U;
              if (qY > current_node[0]) {
                qY = 0U;
              }

              next_node[0] = (unsigned char)qY;
              i222 = (int)(current_node[1] + 1U);
              if ((unsigned int)i222 > 255U) {
                i222 = 255;
              }

              next_node[1] = (unsigned char)i222;
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
          i222 = (int)(current_node[0] + 1U);
          if ((unsigned int)i222 > 255U) {
            i222 = 255;
          }

          if (row_num_node[(i222 + 33 * (current_node[1] - 1)) - 1] < map_min) {
            /* 最小値を更新 */
            i222 = (int)(current_node[0] + 1U);
            if ((unsigned int)i222 > 255U) {
              i222 = 255;
            }

            map_min = row_num_node[(i222 + 33 * (current_node[1] - 1)) - 1];

            /* 現在ノードの進行方向を北東向きに */
            *next_dir = g_d_direction.North_East;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Row;
            i222 = (int)(current_node[0] + 1U);
            if ((unsigned int)i222 > 255U) {
              i222 = 255;
            }

            next_node[0] = (unsigned char)i222;
            next_node[1] = current_node[1];
          }
        } else if (temp == g_d_direction.East) {
          i222 = (int)(current_node[1] + 1U);
          if ((unsigned int)i222 > 255U) {
            i222 = 255;
          }

          if (col_num_node[(current_node[0] + ((i222 - 1) << 5)) - 1] < map_min)
          {
            /* 最小値を更新 */
            i222 = (int)(current_node[1] + 1U);
            if ((unsigned int)i222 > 255U) {
              i222 = 255;
            }

            map_min = col_num_node[(current_node[0] + ((i222 - 1) << 5)) - 1];

            /* 現在ノードの進行方向を東向きに */
            *next_dir = g_d_direction.East;

            /* 進行方向先の座標、行列の方向を更新 */
            *next_node_property = matrix_dir.Col;
            next_node[0] = current_node[0];
            i222 = (int)(current_node[1] + 1U);
            if ((unsigned int)i222 > 255U) {
              i222 = 255;
            }

            next_node[1] = (unsigned char)i222;
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
                i222 = (int)(current_node[0] + 1U);
                if ((unsigned int)i222 > 255U) {
                  i222 = 255;
                }

                qY = current_node[1] - 1U;
                if (qY > current_node[1]) {
                  qY = 0U;
                }

                if (row_num_node[(i222 + 33 * ((int)qY - 1)) - 1] < map_min) {
                  /* 最小値を更新 */
                  i222 = (int)(current_node[0] + 1U);
                  if ((unsigned int)i222 > 255U) {
                    i222 = 255;
                  }

                  qY = current_node[1] - 1U;
                  if (qY > current_node[1]) {
                    qY = 0U;
                  }

                  map_min = row_num_node[(i222 + 33 * ((int)qY - 1)) - 1];

                  /* 現在ノードの進行方向を北西向きに */
                  *next_dir = g_d_direction.North_West;

                  /* 進行方向先の座標、行列の方向を更新 */
                  *next_node_property = matrix_dir.Row;
                  i222 = (int)(current_node[0] + 1U);
                  if ((unsigned int)i222 > 255U) {
                    i222 = 255;
                  }

                  next_node[0] = (unsigned char)i222;
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
 *  出力 次の進行方角
 * Arguments    : unsigned char current_x
 *                unsigned char current_y
 *                const unsigned char maze_wall[1024]
 *                const unsigned short contour_map[1024]
 * Return Type  : unsigned char
 */
static unsigned char get_nextdir2(unsigned char current_x, unsigned char
  current_y, const unsigned char maze_wall[1024], const unsigned short
  contour_map[1024])
{
  unsigned char next_dir;
  unsigned short little;
  int i11;
  int i12;
  int i13;
  int i14;
  unsigned short u0;
  int i15;
  int i16;

  /*     %% get_nextdir2 等高線mapから次に向かう方向を選択 */
  /* 出力の初期化 */
  next_dir = 0U;
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
    next_dir = g_direction.North;
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
      next_dir = g_direction.East;
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
      next_dir = g_direction.South;
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
    next_dir = g_direction.West;
  }

  return next_dir;
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
  int i151;
  unsigned char uv1[18];
  bool temp1[18];
  int ex;
  signed char varargin_1[9];
  int k;

  /* ゴール判定関数 */
  /* ゴール座標と比較 */
  for (i151 = 0; i151 < 9; i151++) {
    uv1[i151] = x;
    uv1[9 + i151] = y;
  }

  for (i151 = 0; i151 < 18; i151++) {
    temp1[i151] = (maze_goal[i151] == uv1[i151]);
  }

  /* x,yともに一致するか確認、一致なら1を返す */
  for (i151 = 0; i151 < 9; i151++) {
    varargin_1[i151] = (signed char)(temp1[i151] * temp1[9 + i151]);
  }

  ex = varargin_1[0];
  for (k = 0; k < 8; k++) {
    i151 = varargin_1[k + 1];
    if (ex < i151) {
      ex = i151;
    }
  }

  return ex;
}

/*
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : const coder_internal_ref_5 *wall
 *                const unsigned char maze_goal[18]
 *                unsigned char l_goal_size
 *                const unsigned char maze_wall[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void make_map_find(const coder_internal_ref_5 *wall, const unsigned char
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
 *                const coder_internal_ref_5 *wall
 *                const coder_internal_ref_4 *search
 *                const unsigned char maze_goal[18]
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned char unknown_wall_flg
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void make_map_fustrun(const coder_internal_ref *goal_size, const
  coder_internal_ref_5 *wall, const coder_internal_ref_4 *search, const unsigned
  char maze_goal[18], const unsigned char maze_wall[1024], const unsigned char
  maze_wall_search[1024], unsigned char unknown_wall_flg, unsigned short
  contour_map[1024])
{
  unsigned char contor_renew_square[2048];
  unsigned char contor_renew_square_temp[2048];
  unsigned char contor_renew_square_idx;
  unsigned char contor_renew_square_idx_temp;
  int i33;
  unsigned char move_dir_map[1024];
  int tempn;
  unsigned short tempi;
  bool exitg1;
  int contour_map_tmp;
  int i34;
  unsigned char change_flag;
  int i35;
  int i36;
  int i37;
  int i38;
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
  unsigned int u2;
  unsigned int qY;
  int i49;
  int i50;
  int i51;
  int i52;

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
  for (i33 = 0; i33 < 1024; i33++) {
    contour_map[i33] = MAX_uint16_T;
    move_dir_map[i33] = 0U;
  }

  /* ゴール座標に */
  /*  歩数マップ：0を入力 */
  /*  進行方向 : 1+2+4+8(東西南北すべて)=15 */
  /*  を入力 */
  i33 = goal_size->contents;
  for (tempn = 0; tempn < i33; tempn++) {
    contor_renew_square_idx = maze_goal[tempn + 9];
    contour_map_tmp = (contor_renew_square_idx + ((maze_goal[tempn] - 1) << 5))
      - 1;
    contour_map[contour_map_tmp] = 0U;
    if (g_direction.North <= 7) {
      i34 = (unsigned char)(1 << g_direction.North);
    } else {
      i34 = 0;
    }

    if (g_direction.East <= 7) {
      i35 = (unsigned char)(1 << g_direction.East);
    } else {
      i35 = 0;
    }

    i36 = (int)((unsigned int)i34 + i35);
    if ((unsigned int)i36 > 255U) {
      i36 = 255;
    }

    if (g_direction.South <= 7) {
      i37 = (unsigned char)(1 << g_direction.South);
    } else {
      i37 = 0;
    }

    i36 = (int)((unsigned int)i36 + i37);
    if ((unsigned int)i36 > 255U) {
      i36 = 255;
    }

    if (g_direction.West <= 7) {
      i40 = (unsigned char)(1 << g_direction.West);
    } else {
      i40 = 0;
    }

    i36 = (int)((unsigned int)i36 + i40);
    if ((unsigned int)i36 > 255U) {
      i36 = 255;
    }

    move_dir_map[contour_map_tmp] = (unsigned char)i36;

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
    i33 = contor_renew_square_idx;
    for (tempn = 0; tempn < i33; tempn++) {
      /* 北側 */
      /* 壁が無い & (探索済み || ~未知壁フラグ)であるとき */
      if (g_direction.North <= 7) {
        i38 = (unsigned char)(1 << g_direction.North);
      } else {
        i38 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i38) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i39 = (unsigned char)(1 << g_direction.North);
        } else {
          i39 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i39) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が北向きである時 */
          contor_renew_square_idx = contor_renew_square[tempn + 1024];
          i36 = (contor_renew_square_idx - 1) << 5;
          contour_map_tmp = (contor_renew_square[tempn] + i36) - 1;
          if (g_direction.North <= 7) {
            i43 = (unsigned char)(1 << g_direction.North);
          } else {
            i43 = 0;
          }

          if ((move_dir_map[contour_map_tmp] & i43) != 0) {
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

            if (contour_map[(contour_map_tmp + i36) - 1] > (int)qY) {
              /* 歩数MAP更新 */
              contour_map_tmp = (int)(contor_renew_square[tempn] + 1U);
              if ((unsigned int)contour_map_tmp > 255U) {
                contour_map_tmp = 255;
              }

              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contour_map_tmp + i36) - 1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              contour_map_tmp = (int)(contor_renew_square[tempn] + 1U);
              if ((unsigned int)contour_map_tmp > 255U) {
                contour_map_tmp = 255;
              }

              if (g_direction.North <= 7) {
                move_dir_map[(contour_map_tmp + i36) - 1] = (unsigned char)(1 <<
                  g_direction.North);
              } else {
                move_dir_map[(contour_map_tmp + i36) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              i36 = (int)(contor_renew_square[tempn] + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                (unsigned char)i36;
              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                contor_renew_square_idx;

              /* 更新マス用インデックスを増加 */
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
            }

            /* かつ進行方向が北向きでないとき */
          } else {
            /* かつ北のマスの歩数MAP値が、更新予定値より大きい場合 */
            i46 = (int)(contor_renew_square[tempn] + 1U);
            i47 = i46;
            if ((unsigned int)i46 > 255U) {
              i47 = 255;
            }

            u2 = contour_map[contour_map_tmp] + 5U;
            qY = u2;
            if (u2 > 65535U) {
              qY = 65535U;
            }

            if (contour_map[(i47 + i36) - 1] > (int)qY) {
              /* 歩数MAP更新(重みづけあり) */
              contour_map_tmp = i46;
              if ((unsigned int)i46 > 255U) {
                contour_map_tmp = 255;
              }

              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contour_map_tmp + i36) - 1] = (unsigned short)u2;

              /* 移動方向MAP更新 */
              contour_map_tmp = i46;
              if ((unsigned int)i46 > 255U) {
                contour_map_tmp = 255;
              }

              if (g_direction.North <= 7) {
                move_dir_map[(contour_map_tmp + i36) - 1] = (unsigned char)(1 <<
                  g_direction.North);
              } else {
                move_dir_map[(contour_map_tmp + i36) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              if ((unsigned int)i46 > 255U) {
                i46 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                (unsigned char)i46;
              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                contor_renew_square_idx;

              /* 更新マス用インデックスを増加 */
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
            }
          }
        }
      }

      /* 東側 */
      /* 壁が無い & (探索済み|| ~未知壁フラグ)であるとき */
      if (g_direction.East <= 7) {
        i41 = (unsigned char)(1 << g_direction.East);
      } else {
        i41 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i41) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i42 = (unsigned char)(1 << g_direction.East);
        } else {
          i42 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i42) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が東向きである時 */
          if (g_direction.East <= 7) {
            i45 = (unsigned char)(1 << g_direction.East);
          } else {
            i45 = 0;
          }

          if ((move_dir_map[(contor_renew_square[tempn] +
                             ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
               & i45) != 0) {
            /* かつ東のマスが更新予定値よりも大きな値の場合 */
            i36 = (int)(contor_renew_square[tempn + 1024] + 1U);
            if ((unsigned int)i36 > 255U) {
              i36 = 255;
            }

            u2 = contour_map[(contor_renew_square[tempn] +
                              ((contor_renew_square[tempn + 1024] - 1) << 5)) -
              1] + 1U;
            if (u2 > 65535U) {
              u2 = 65535U;
            }

            if (contour_map[(contor_renew_square[tempn] + ((i36 - 1) << 5)) - 1]
                > (int)u2) {
              /*                                  %更新確認用のMAP更新 */
              /*                                  contour_refine_map(row(tempn),col(tempn)+1) = contour_refine_map(row(tempn),col(tempn))+uint16(1); */
              /* 歩数MAP更新 */
              i36 = (int)(contor_renew_square[tempn + 1024] + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              u2 = contour_map[(contor_renew_square[tempn] +
                                ((contor_renew_square[tempn + 1024] - 1) << 5))
                - 1] + 1U;
              if (u2 > 65535U) {
                u2 = 65535U;
              }

              contour_map[(contor_renew_square[tempn] + ((i36 - 1) << 5)) - 1] =
                (unsigned short)u2;

              /* 移動方向MAP更新 */
              i36 = (int)(contor_renew_square[tempn + 1024] + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              if (g_direction.East <= 7) {
                move_dir_map[(contor_renew_square[tempn] + ((i36 - 1) << 5)) - 1]
                  = (unsigned char)(1 << g_direction.East);
              } else {
                move_dir_map[(contor_renew_square[tempn] + ((i36 - 1) << 5)) - 1]
                  = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新マスを更新 */
              contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
                contor_renew_square[tempn];
              i36 = (int)(contor_renew_square[tempn + 1024] + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                (unsigned char)i36;

              /* 更新マス用インデックスを増加 */
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
            }

            /* かつ進行方向が東向きでないとき */
          } else {
            /* かつ東のマスの歩数MAP値が、更新予定値より大きい場合 */
            i36 = (int)(contor_renew_square[tempn + 1024] + 1U);
            contour_map_tmp = i36;
            if ((unsigned int)i36 > 255U) {
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
              contour_map_tmp = i36;
              if ((unsigned int)i36 > 255U) {
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
              contour_map_tmp = i36;
              if ((unsigned int)i36 > 255U) {
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
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
                (unsigned char)i36;

              /* 更新マス用インデックスを増加 */
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
            }
          }
        }
      }

      /* 南側 */
      /* 壁が無い &  (探索済み|| ~未知壁フラグ)であるとき */
      if (g_direction.South <= 7) {
        i44 = (unsigned char)(1 << g_direction.South);
      } else {
        i44 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i44) != 0) == wall->contents.nowall) {
        if (g_direction.South <= 7) {
          i48 = (unsigned char)(1 << g_direction.South);
        } else {
          i48 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i48) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が南向きである時 */
          if (g_direction.South <= 7) {
            i50 = (unsigned char)(1 << g_direction.South);
          } else {
            i50 = 0;
          }

          if ((move_dir_map[(contor_renew_square[tempn] +
                             ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
               & i50) != 0) {
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
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
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
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
            }
          }
        }
      }

      /* 西側 */
      /* 壁が無い &  (探索済み|| ~未知壁フラグ)であるとき */
      if (g_direction.West <= 7) {
        i49 = (unsigned char)(1 << g_direction.West);
      } else {
        i49 = 0;
      }

      if (((maze_wall[(contor_renew_square[tempn] + ((contor_renew_square[tempn
               + 1024] - 1) << 5)) - 1] & i49) != 0) == wall->contents.nowall) {
        if (g_direction.West <= 7) {
          i51 = (unsigned char)(1 << g_direction.West);
        } else {
          i51 = 0;
        }

        if ((((maze_wall_search[(contor_renew_square[tempn] +
                ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1] & i51) != 0)
             == search->contents.known) || (unknown_wall_flg == 0)) {
          /* かつ進行方向が西向きである時 */
          if (g_direction.West <= 7) {
            i52 = (unsigned char)(1 << g_direction.West);
          } else {
            i52 = 0;
          }

          if ((move_dir_map[(contor_renew_square[tempn] +
                             ((contor_renew_square[tempn + 1024] - 1) << 5)) - 1]
               & i52) != 0) {
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
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
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
              i36 = (int)(contor_renew_square_idx_temp + 1U);
              if ((unsigned int)i36 > 255U) {
                i36 = 255;
              }

              contor_renew_square_idx_temp = (unsigned char)i36;
            }
          }
        }
      }
    }

    /* ゴール更新マスの更新とインデックスのクリア */
    for (i33 = 0; i33 < 2048; i33++) {
      contor_renew_square[i33] = contor_renew_square_temp[i33];
      contor_renew_square_temp[i33] = 0U;
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
 * Arguments    : coder_internal_ref_2 *max_length
 *                const coder_internal_ref_5 *wall
 *                const coder_internal_ref_4 *search
 *                const unsigned char maze_goal[18]
 *                unsigned char goal_size
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned short row_num_node[1056]
 *                unsigned short col_num_node[1056]
 *                unsigned short *start_num
 * Return Type  : void
 */
static void make_map_fustrun_diagonal(coder_internal_ref_2 *max_length, const
  coder_internal_ref_5 *wall, const coder_internal_ref_4 *search, const unsigned
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
  int i74;
  int n;
  int i75;
  unsigned char u6;
  int i76;
  int i77;
  int i78;
  unsigned int qY;
  int i79;
  int row_num_node_tmp;
  unsigned short i;
  bool exitg1;
  int i80;
  int i81;
  int i82;
  unsigned char change_flag;
  int i83;
  int i84;
  int i85;
  int i86;
  int i87;
  int i88;
  int i89;
  int i90;
  int i91;
  unsigned int b_qY;
  int i92;
  int i93;
  int i94;
  int i95;
  int i96;
  unsigned int c_qY;
  int i97;
  int i98;
  int i99;
  int i100;
  unsigned int u7;
  int i101;
  int i102;
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
    i74 = (maze_goal[9] + (q0 << 5)) - 1;
    if (g_direction.North <= 7) {
      i75 = (unsigned char)(1 << g_direction.North);
    } else {
      i75 = 0;
    }

    if ((maze_wall[i74] & i75) == 0) {
      /* 歩数更新 */
      i76 = (int)(maze_goal[9] + 1U);
      i77 = i76;
      if ((unsigned int)i76 > 255U) {
        i77 = 255;
      }

      row_num_node_tmp = 33 * q0;
      row_num_node[(i77 + row_num_node_tmp) - 1] = 3U;

      /* 方向追加 */
      q0 = i76;
      if ((unsigned int)i76 > 255U) {
        q0 = 255;
      }

      if (g_d_direction.North <= 7) {
        row_dir_node[(q0 + row_num_node_tmp) - 1] = (unsigned char)(1 <<
          g_d_direction.North);
      } else {
        row_dir_node[(q0 + row_num_node_tmp) - 1] = 0U;
      }

      /* 更新ノードを更新 */
      if ((unsigned int)i76 > 255U) {
        i76 = 255;
      }

      contor_renew_node_row[0] = (unsigned char)i76;
      contor_renew_node_row[1024] = maze_goal[0];

      /* 更新マス用インデックスを増加 */
      contor_renew_node_row_idx = 2U;
    }

    /* 東壁 */
    if (g_direction.East <= 7) {
      i78 = (unsigned char)(1 << g_direction.East);
    } else {
      i78 = 0;
    }

    if ((maze_wall[i74] & i78) == 0) {
      /* 歩数更新 */
      q0 = (int)(maze_goal[0] + 1U);
      i76 = q0;
      if ((unsigned int)q0 > 255U) {
        i76 = 255;
      }

      col_num_node[(maze_goal[9] + ((i76 - 1) << 5)) - 1] = 3U;

      /* 方向追加 */
      i76 = q0;
      if ((unsigned int)q0 > 255U) {
        i76 = 255;
      }

      if (g_d_direction.East <= 7) {
        col_dir_node[(maze_goal[9] + ((i76 - 1) << 5)) - 1] = (unsigned char)(1 <<
          g_d_direction.East);
      } else {
        col_dir_node[(maze_goal[9] + ((i76 - 1) << 5)) - 1] = 0U;
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
      i81 = (unsigned char)(1 << g_direction.South);
    } else {
      i81 = 0;
    }

    if ((maze_wall[i74] & i81) == 0) {
      /* 歩数更新 */
      row_num_node_tmp = (maze_goal[9] + 33 * (maze_goal[0] - 1)) - 1;
      row_num_node[row_num_node_tmp] = 3U;

      /* 方向追加 */
      if (g_d_direction.South <= 7) {
        i85 = (unsigned char)(1 << g_d_direction.South);
      } else {
        i85 = 0;
      }

      row_dir_node[row_num_node_tmp] = (unsigned char)
        (row_dir_node[row_num_node_tmp] | i85);

      /* 更新ノードを更新 */
      contor_renew_node_row[contor_renew_node_row_idx - 1] = maze_goal[9];
      contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[0];

      /* 更新マス用インデックスを増加 */
      contor_renew_node_row_idx++;
    }

    /* 西壁 */
    if (g_direction.West <= 7) {
      i83 = (unsigned char)(1 << g_direction.West);
    } else {
      i83 = 0;
    }

    if ((maze_wall[i74] & i83) == 0) {
      /* 歩数更新 */
      col_num_node[i74] = 3U;

      /* 方向追加 */
      if (g_d_direction.West <= 7) {
        i87 = (unsigned char)(1 << g_d_direction.West);
      } else {
        i87 = 0;
      }

      col_dir_node[i74] = (unsigned char)(col_dir_node[i74] | i87);

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
      i74 = maze_goal[n] - 1;
      u6 = maze_goal[n + 9];
      i76 = (u6 + (i74 << 5)) - 1;
      if (g_direction.North <= 7) {
        i79 = (unsigned char)(1 << g_direction.North);
      } else {
        i79 = 0;
      }

      if ((maze_wall[i76] & i79) == 0) {
        /* 歩数更新 */
        i77 = (int)(maze_goal[n + 9] + 1U);
        i80 = i77;
        if ((unsigned int)i77 > 255U) {
          i80 = 255;
        }

        row_num_node_tmp = 33 * i74;
        row_num_node[(i80 + row_num_node_tmp) - 1] = 0U;

        /* 方向更新 */
        i74 = i77;
        if ((unsigned int)i77 > 255U) {
          i74 = 255;
        }

        row_dir_node[(i74 + row_num_node_tmp) - 1] = MAX_uint8_T;

        /* 更新ノードを更新 */
        if ((unsigned int)i77 > 255U) {
          i77 = 255;
        }

        contor_renew_node_row[contor_renew_node_row_idx - 1] = (unsigned char)
          i77;
        contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[n];

        /* 更新マス用インデックスを増加 */
        i74 = (int)(contor_renew_node_row_idx + 1U);
        if ((unsigned int)i74 > 255U) {
          i74 = 255;
        }

        contor_renew_node_row_idx = (unsigned char)i74;
      }

      /* 東壁 */
      if (g_direction.East <= 7) {
        i82 = (unsigned char)(1 << g_direction.East);
      } else {
        i82 = 0;
      }

      if ((maze_wall[i76] & i82) == 0) {
        /* 歩数更新 */
        i74 = (int)(maze_goal[n] + 1U);
        i77 = i74;
        if ((unsigned int)i74 > 255U) {
          i77 = 255;
        }

        col_num_node[(u6 + ((i77 - 1) << 5)) - 1] = 0U;

        /* 方向更新 */
        i77 = i74;
        if ((unsigned int)i74 > 255U) {
          i77 = 255;
        }

        col_dir_node[(u6 + ((i77 - 1) << 5)) - 1] = MAX_uint8_T;

        /* 更新ノードを更新 */
        contor_renew_node_col[contor_renew_node_col_idx - 1] = u6;
        if ((unsigned int)i74 > 255U) {
          i74 = 255;
        }

        contor_renew_node_col[contor_renew_node_col_idx + 1023] = (unsigned char)
          i74;

        /* 更新マス用インデックスを増加 */
        i74 = (int)(contor_renew_node_col_idx + 1U);
        if ((unsigned int)i74 > 255U) {
          i74 = 255;
        }

        contor_renew_node_col_idx = (unsigned char)i74;
      }

      /* 南壁 */
      if (g_direction.South <= 7) {
        i84 = (unsigned char)(1 << g_direction.South);
      } else {
        i84 = 0;
      }

      if ((maze_wall[i76] & i84) == 0) {
        /* 歩数更新 */
        row_num_node_tmp = (u6 + 33 * (maze_goal[n] - 1)) - 1;
        row_num_node[row_num_node_tmp] = 0U;

        /* 方向更新 */
        row_dir_node[row_num_node_tmp] = MAX_uint8_T;

        /* 更新ノードを更新 */
        contor_renew_node_row[contor_renew_node_row_idx - 1] = u6;
        contor_renew_node_row[contor_renew_node_row_idx + 1023] = maze_goal[n];

        /* 更新マス用インデックスを増加 */
        i74 = (int)(contor_renew_node_row_idx + 1U);
        if ((unsigned int)i74 > 255U) {
          i74 = 255;
        }

        contor_renew_node_row_idx = (unsigned char)i74;
      }

      /* 西壁 */
      if (g_direction.West <= 7) {
        i86 = (unsigned char)(1 << g_direction.West);
      } else {
        i86 = 0;
      }

      if ((maze_wall[i76] & i86) == 0) {
        /* 歩数更新 */
        col_num_node[i76] = 0U;

        /* 方向更新 */
        col_dir_node[i76] = MAX_uint8_T;

        /* 更新ノードを更新 */
        contor_renew_node_col[contor_renew_node_col_idx - 1] = u6;
        contor_renew_node_col[contor_renew_node_col_idx + 1023] = maze_goal[n];

        /* 更新マス用インデックスを増加 */
        i74 = (int)(contor_renew_node_col_idx + 1U);
        if ((unsigned int)i74 > 255U) {
          i74 = 255;
        }

        contor_renew_node_col_idx = (unsigned char)i74;
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
      i74 = contor_renew_node_row[n + 1024] - 1;
      i76 = (contor_renew_node_row[n] + (i74 << 5)) - 1;
      if (g_direction.North <= 7) {
        i88 = (unsigned char)(1 << g_direction.North);
      } else {
        i88 = 0;
      }

      if (((maze_wall[i76] & i88) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i89 = (unsigned char)(1 << g_direction.North);
        } else {
          i89 = 0;
        }

        if (((maze_wall_search[i76] & i89) != 0) == search->contents.known) {
          /* かつ進行方向が北向きである時 */
          i74 *= 33;
          i76 = (contor_renew_node_row[n] + i74) - 1;
          if (g_d_direction.North <= 7) {
            i94 = (unsigned char)(1 << g_d_direction.North);
          } else {
            i94 = 0;
          }

          if ((row_dir_node[i76] & i94) != 0) {
            /* かつ北のノードが更新予定値よりも大きな値の場合 */
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

            if (row_num_node[(i76 + i74) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i74 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i74 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(1 << g_d_direction.North);
              } else {
                row_dir_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i74 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i74;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i74;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i74 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i74 = (int)(contor_renew_node_row[n] + 1U);
                if ((unsigned int)i74 > 255U) {
                  i74 = 255;
                }

                i76 = (int)(contor_renew_node_row[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i110 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i110 = 0;
                }

                row_dir_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(row_dir_node[(i76 + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i110);
              }
            }

            /* かつ進行方向が北向きでないとき */
          } else {
            /* かつ北のノードの歩数MAP値が、更新予定値より大きい場合 */
            i77 = (int)(contor_renew_node_row[n] + 1U);
            if ((unsigned int)i77 > 255U) {
              i77 = 255;
            }

            b_qY = row_num_node[i76] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i77 + i74) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i74 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i74 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              if (g_d_direction.North <= 7) {
                row_dir_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(1 << g_d_direction.North);
              } else {
                row_dir_node[(i74 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i74 = (int)(contor_renew_node_row[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i74;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i74;

              /* かつ北のノードが更新予定値と同じ場合 */
            } else {
              i74 = (int)(contor_renew_node_row[n] + 1U);
              i76 = i74;
              if ((unsigned int)i74 > 255U) {
                i76 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = i74;
                if ((unsigned int)i74 > 255U) {
                  i76 = 255;
                  i74 = 255;
                }

                if (g_d_direction.North <= 7) {
                  i108 = (unsigned char)(1 << g_d_direction.North);
                } else {
                  i108 = 0;
                }

                row_dir_node[(i76 + 33 * (contor_renew_node_row[n + 1024] - 1))
                  - 1] = (unsigned char)(row_dir_node[(i74 + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i108);
              }
            }
          }
        }
      }

      /* 北東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.East <= 7) {
        i91 = (unsigned char)(1 << g_direction.East);
      } else {
        i91 = 0;
      }

      if (((maze_wall[(contor_renew_node_row[n] + ((contor_renew_node_row[n +
               1024] - 1) << 5)) - 1] & i91) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i93 = (unsigned char)(1 << g_direction.East);
        } else {
          i93 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_row[n] +
                                ((contor_renew_node_row[n + 1024] - 1) << 5)) -
              1] & i93) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i97 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i97 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i97)
              != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i74 > 255U) {
              i74 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新 */
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i74;

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i74;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i74 > 255U) {
                  i74 = 255;
                }

                i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i118 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i118 = 0;
                }

                col_dir_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_row[n] + ((i76
                  - 1) << 5)) - 1] | i118);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i74 > 255U) {
              i74 = 255;
            }

            i76 = (contor_renew_node_row[n] + 33 * (contor_renew_node_row[n +
                    1024] - 1)) - 1;
            b_qY = row_num_node[i76] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                col_dir_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.North_East);
              } else {
                col_dir_node[(contor_renew_node_row[n] + ((i74 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                (unsigned char)i74;

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i74;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i74 = (int)(contor_renew_node_row[n + 1024] + 1U);
              i77 = i74;
              if ((unsigned int)i74 > 255U) {
                i77 = 255;
              }

              b_qY = row_num_node[i76] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[n] + ((i77 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = i74;
                if ((unsigned int)i74 > 255U) {
                  i76 = 255;
                  i74 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i116 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i116 = 0;
                }

                col_dir_node[(contor_renew_node_row[n] + ((i76 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_row[n] + ((i74
                  - 1) << 5)) - 1] | i116);
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

      i74 = (contor_renew_node_row[n + 1024] - 1) << 5;
      if (g_direction.East <= 7) {
        i99 = (unsigned char)(1 << g_direction.East);
      } else {
        i99 = 0;
      }

      if (((maze_wall[((int)c_qY + i74) - 1] & i99) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[n] - 1U;
        if (c_qY > contor_renew_node_row[n]) {
          c_qY = 0U;
        }

        if (g_direction.East <= 7) {
          i103 = (unsigned char)(1 << g_direction.East);
        } else {
          i103 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i74) - 1] & i103) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i106 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i106 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i106)
              != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              4U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i76 - 1) << 5)) - 1] > (int)u7) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = 0U;
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

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i76 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                i77 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i77 > 255U) {
                  i77 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i137 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i137 = 0;
                }

                col_dir_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i77 - 1) << 5)) - 1] | i137);
              }
            }

            /* かつ進行方向が南東向きでないとき */
          } else {
            /* かつ南東のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              18U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            if (col_num_node[((int)c_qY + ((i76 - 1) << 5)) - 1] > (int)u7) {
              /* 歩数MAP更新(重みづけあり) */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.South_East <= 7) {
                col_dir_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = (unsigned
                  char)(1 << g_d_direction.South_East);
              } else {
                col_dir_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = 0U;
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

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + ((i76 - 1) << 5)) - 1] == (int)b_qY)
              {
                /* 移動方向を追加 */
                c_qY = contor_renew_node_row[n] - 1U;
                if (c_qY > contor_renew_node_row[n]) {
                  c_qY = 0U;
                }

                i76 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                b_qY = contor_renew_node_row[n] - 1U;
                if (b_qY > contor_renew_node_row[n]) {
                  b_qY = 0U;
                }

                i77 = (int)(contor_renew_node_row[n + 1024] + 1U);
                if ((unsigned int)i77 > 255U) {
                  i77 = 255;
                }

                if (g_d_direction.South_East <= 7) {
                  i136 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i136 = 0;
                }

                col_dir_node[((int)c_qY + ((i76 - 1) << 5)) - 1] = (unsigned
                  char)(col_dir_node[((int)b_qY + ((i77 - 1) << 5)) - 1] | i136);
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
        i104 = (unsigned char)(1 << g_direction.South);
      } else {
        i104 = 0;
      }

      if (((maze_wall[((int)c_qY + i74) - 1] & i104) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[n] - 1U;
        if (c_qY > contor_renew_node_row[n]) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i109 = (unsigned char)(1 << g_direction.South);
        } else {
          i109 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i74) - 1] & i109) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南向きである時 */
          if (g_d_direction.South <= 7) {
            i115 = (unsigned char)(1 << g_d_direction.South);
          } else {
            i115 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i115)
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
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

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
                  i135 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i135 = 0;
                }

                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = (unsigned char)(row_dir_node[((int)b_qY + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i135);
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
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

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
                  i134 = (unsigned char)(1 << g_d_direction.South);
                } else {
                  i134 = 0;
                }

                row_dir_node[((int)c_qY + 33 * (contor_renew_node_row[n + 1024]
                  - 1)) - 1] = (unsigned char)(row_dir_node[((int)b_qY + 33 *
                  (contor_renew_node_row[n + 1024] - 1)) - 1] | i134);
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
        i113 = (unsigned char)(1 << g_direction.West);
      } else {
        i113 = 0;
      }

      if (((maze_wall[((int)c_qY + i74) - 1] & i113) != 0) ==
          wall->contents.nowall) {
        c_qY = contor_renew_node_row[n] - 1U;
        if (c_qY > contor_renew_node_row[n]) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i122 = (unsigned char)(1 << g_direction.West);
        } else {
          i122 = 0;
        }

        if (((maze_wall_search[((int)c_qY + i74) - 1] & i122) != 0) ==
            search->contents.known) {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i128 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i128 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i128)
              != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = contor_renew_node_row[n] - 1U;
            if (c_qY > contor_renew_node_row[n]) {
              c_qY = 0U;
            }

            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[((int)c_qY + i74) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[((int)c_qY + i74) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i74) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i74) - 1] = 0U;
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
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[((int)c_qY + i74) - 1] == (int)b_qY) {
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
                  i143 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i143 = 0;
                }

                col_dir_node[((int)c_qY + i74) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i74) - 1] | i143);
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

            if (col_num_node[((int)c_qY + i74) - 1] > (int)b_qY) {
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

              col_num_node[((int)c_qY + i74) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              c_qY = contor_renew_node_row[n] - 1U;
              if (c_qY > contor_renew_node_row[n]) {
                c_qY = 0U;
              }

              if (g_d_direction.South_West <= 7) {
                col_dir_node[((int)c_qY + i74) - 1] = (unsigned char)(1 <<
                  g_d_direction.South_West);
              } else {
                col_dir_node[((int)c_qY + i74) - 1] = 0U;
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
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

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

              if (col_num_node[((int)c_qY + i74) - 1] == (int)b_qY) {
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
                  i142 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i142 = 0;
                }

                col_dir_node[((int)c_qY + i74) - 1] = (unsigned char)
                  (col_dir_node[((int)b_qY + i74) - 1] | i142);
              }
            }
          }
        }
      }

      /* 北西側 */
      /* 壁が無い & 探索済みであるとき */
      i74 = (contor_renew_node_row[n] + i74) - 1;
      if (g_direction.West <= 7) {
        i124 = (unsigned char)(1 << g_direction.West);
      } else {
        i124 = 0;
      }

      if (((maze_wall[i74] & i124) != 0) == wall->contents.nowall) {
        if (g_direction.West <= 7) {
          i127 = (unsigned char)(1 << g_direction.West);
        } else {
          i127 = 0;
        }

        if (((maze_wall_search[i74] & i127) != 0) == search->contents.known) {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i131 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i131 = 0;
          }

          if ((row_dir_node[(contor_renew_node_row[n] + 33 *
                             (contor_renew_node_row[n + 1024] - 1)) - 1] & i131)
              != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                 (contor_renew_node_row[n + 1024] - 1)) - 1] +
              4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_row[n] +
                              ((contor_renew_node_row[n + 1024] - 1) << 5)) - 1]
                > (int)b_qY) {
              /* 歩数MAP更新 */
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i74] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i74] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i74] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i74;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_row[n] +
                                ((contor_renew_node_row[n + 1024] - 1) << 5)) -
                  1] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i141 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i141 = 0;
                }

                col_dir_node[i74] = (unsigned char)(col_dir_node[i74] | i141);
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

            if (col_num_node[i74] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[i74] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.North_West <= 7) {
                col_dir_node[i74] = (unsigned char)(1 <<
                  g_d_direction.North_West);
              } else {
                col_dir_node[i74] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_row[n];
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp + 1023] =
                contor_renew_node_row[n + 1024];

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i74;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              b_qY = row_num_node[(contor_renew_node_row[n] + 33 *
                                   (contor_renew_node_row[n + 1024] - 1)) - 1] +
                18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[i74] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.North_West <= 7) {
                  i140 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i140 = 0;
                }

                col_dir_node[i74] = (unsigned char)(col_dir_node[i74] | i140);
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
      i74 = (contor_renew_node_col[n] + ((contor_renew_node_col[n + 1024] - 1) <<
              5)) - 1;
      if (g_direction.North <= 7) {
        i90 = (unsigned char)(1 << g_direction.North);
      } else {
        i90 = 0;
      }

      if (((maze_wall[i74] & i90) != 0) == wall->contents.nowall) {
        if (g_direction.North <= 7) {
          i92 = (unsigned char)(1 << g_direction.North);
        } else {
          i92 = 0;
        }

        if (((maze_wall_search[i74] & i92) != 0) == search->contents.known) {
          /* かつ進行方向が北東向きである時 */
          if (g_d_direction.North_East <= 7) {
            i96 = (unsigned char)(1 << g_d_direction.North_East);
          } else {
            i96 = 0;
          }

          if ((col_dir_node[i74] & i96) != 0) {
            /* かつ北東のノードが更新予定値よりも大きな値の場合 */
            i74 = (int)(contor_renew_node_col[n] + 1U);
            if ((unsigned int)i74 > 255U) {
              i74 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 4U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            if (row_num_node[(i74 + 33 * (contor_renew_node_col[n + 1024] - 1))
                - 1] > (int)u7) {
              /* 歩数MAP更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i74 + 33 * (contor_renew_node_col[n + 1024] - 1)) -
                1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i74 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] = (unsigned char)(1 << g_d_direction.North_East);
              } else {
                row_dir_node[(i74 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i74;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[n + 1024];

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i74;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i74 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i74 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i74 > 255U) {
                  i74 = 255;
                }

                i76 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i114 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i114 = 0;
                }

                row_dir_node[(i74 + 33 * (contor_renew_node_col[n + 1024] - 1))
                  - 1] = (unsigned char)(row_dir_node[(i76 + 33 *
                  (contor_renew_node_col[n + 1024] - 1)) - 1] | i114);
              }
            }

            /* かつ進行方向が北東向きでないとき */
          } else {
            /* かつ北東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i74 = (int)(contor_renew_node_col[n] + 1U);
            i76 = i74;
            if ((unsigned int)i74 > 255U) {
              i76 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 18U;
            u7 = b_qY;
            if (b_qY > 65535U) {
              u7 = 65535U;
            }

            i77 = 33 * (contor_renew_node_col[n + 1024] - 1);
            if (row_num_node[(i76 + i77) - 1] > (int)u7) {
              /* 歩数MAP更新(重みづけあり) */
              i76 = i74;
              if ((unsigned int)i74 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i76 + i77) - 1] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i76 = i74;
              if ((unsigned int)i74 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.North_East <= 7) {
                row_dir_node[(i76 + i77) - 1] = (unsigned char)(1 <<
                  g_d_direction.North_East);
              } else {
                row_dir_node[(i76 + i77) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i74;
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                contor_renew_node_col[n + 1024];

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i74;

              /* かつ北東のノードが更新予定値と同じ場合 */
            } else {
              i76 = i74;
              if ((unsigned int)i74 > 255U) {
                i76 = 255;
              }

              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i76 + i77) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = i74;
                if ((unsigned int)i74 > 255U) {
                  i76 = 255;
                  i74 = 255;
                }

                if (g_d_direction.North_East <= 7) {
                  i112 = (unsigned char)(1 << g_d_direction.North_East);
                } else {
                  i112 = 0;
                }

                row_dir_node[(i76 + i77) - 1] = (unsigned char)(row_dir_node
                  [(i74 + i77) - 1] | i112);
              }
            }
          }
        }
      }

      /* 東側 */
      /* 壁が無い & 探索済みであるとき */
      u6 = contor_renew_node_col[n + 1024];
      i74 = (contor_renew_node_col[n] + ((u6 - 1) << 5)) - 1;
      if (g_direction.East <= 7) {
        i95 = (unsigned char)(1 << g_direction.East);
      } else {
        i95 = 0;
      }

      if (((maze_wall[i74] & i95) != 0) == wall->contents.nowall) {
        if (g_direction.East <= 7) {
          i98 = (unsigned char)(1 << g_direction.East);
        } else {
          i98 = 0;
        }

        if (((maze_wall_search[i74] & i98) != 0) == search->contents.known) {
          /* かつ進行方向が東向きである時 */
          if (g_d_direction.East <= 7) {
            i101 = (unsigned char)(1 << g_d_direction.East);
          } else {
            i101 = 0;
          }

          if ((col_dir_node[i74] & i101) != 0) {
            /* かつ東のノードが更新予定値よりも大きな値の場合 */
            i76 = (int)(contor_renew_node_col[n + 1024] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 6U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[n] + ((i76 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新 */
              i76 = (int)(contor_renew_node_col[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[n] + ((i76 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i76 = (int)(contor_renew_node_col[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[n] + ((i76 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[n] + ((i76 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[n];
              i76 = (int)(contor_renew_node_col[n + 1024] + 1U);
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

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i76 = (int)(contor_renew_node_col[n + 1024] + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 6U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[n] + ((i76 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i76 = (int)(contor_renew_node_col[n + 1024] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                i77 = (int)(contor_renew_node_col[n + 1024] + 1U);
                if ((unsigned int)i77 > 255U) {
                  i77 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i126 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i126 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + ((i76 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_col[n] + ((i77
                  - 1) << 5)) - 1] | i126);
              }
            }

            /* かつ進行方向が東向きでないとき */
          } else {
            /* かつ東のノードの歩数MAP値が、更新予定値より大きい場合 */
            i76 = (int)(contor_renew_node_col[n + 1024] + 1U);
            i77 = i76;
            if ((unsigned int)i76 > 255U) {
              i77 = 255;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (col_num_node[(contor_renew_node_col[n] + ((i77 - 1) << 5)) - 1] >
                (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i77 = i76;
              if ((unsigned int)i76 > 255U) {
                i77 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              col_num_node[(contor_renew_node_col[n] + ((i77 - 1) << 5)) - 1] =
                (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              i77 = i76;
              if ((unsigned int)i76 > 255U) {
                i77 = 255;
              }

              if (g_d_direction.East <= 7) {
                col_dir_node[(contor_renew_node_col[n] + ((i77 - 1) << 5)) - 1] =
                  (unsigned char)(1 << g_d_direction.East);
              } else {
                col_dir_node[(contor_renew_node_col[n] + ((i77 - 1) << 5)) - 1] =
                  0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_col_temp[contor_renew_node_col_idx_temp - 1] =
                contor_renew_node_col[n];
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

              /* かつ東のノードが更新予定値と同じ場合 */
            } else {
              i77 = i76;
              if ((unsigned int)i76 > 255U) {
                i77 = 255;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (col_num_node[(contor_renew_node_col[n] + ((i77 - 1) << 5)) - 1]
                  == (int)b_qY) {
                /* 移動方向を追加 */
                i77 = i76;
                if ((unsigned int)i76 > 255U) {
                  i77 = 255;
                  i76 = 255;
                }

                if (g_d_direction.East <= 7) {
                  i123 = (unsigned char)(1 << g_d_direction.East);
                } else {
                  i123 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + ((i77 - 1) << 5)) - 1] =
                  (unsigned char)(col_dir_node[(contor_renew_node_col[n] + ((i76
                  - 1) << 5)) - 1] | i123);
              }
            }
          }
        }
      }

      /* 南東側 */
      /* 壁が無い & 探索済みであるとき */
      if (g_direction.South <= 7) {
        i100 = (unsigned char)(1 << g_direction.South);
      } else {
        i100 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + ((contor_renew_node_col[n +
               1024] - 1) << 5)) - 1] & i100) != 0) == wall->contents.nowall) {
        if (g_direction.South <= 7) {
          i102 = (unsigned char)(1 << g_direction.South);
        } else {
          i102 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] +
                                ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] & i102) != 0) == search->contents.known) {
          /* かつ進行方向が南東向きである時 */
          if (g_d_direction.South_East <= 7) {
            i105 = (unsigned char)(1 << g_d_direction.South_East);
          } else {
            i105 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[n] +
                             ((contor_renew_node_col[n + 1024] - 1) << 5)) - 1]
               & i105) != 0) {
            /* かつ南東のノードが更新予定値よりも大きな値の場合 */
            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(contor_renew_node_col[n] + 33 *
                              (contor_renew_node_col[n + 1024] - 1)) - 1] > (int)
                b_qY) {
              /* 歩数MAP更新 */
              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 4U;
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
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(contor_renew_node_col[n] + 33 *
                                (contor_renew_node_col[n + 1024] - 1)) - 1] ==
                  (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i120 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i120 = 0;
                }

                row_dir_node[(contor_renew_node_col[n] + 33 *
                              (contor_renew_node_col[n + 1024] - 1)) - 1] =
                  (unsigned char)(row_dir_node[(contor_renew_node_col[n] + 33 *
                  (contor_renew_node_col[n + 1024] - 1)) - 1] | i120);
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

            i76 = (contor_renew_node_col[n] + 33 * (contor_renew_node_col[n +
                    1024] - 1)) - 1;
            if (row_num_node[i76] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[i76] = (unsigned short)b_qY;

              /* 移動方向MAP更新 */
              if (g_d_direction.South_East <= 7) {
                row_dir_node[i76] = (unsigned char)(1 <<
                  g_d_direction.South_East);
              } else {
                row_dir_node[i76] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                contor_renew_node_col[n];
              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                u6;

              /* 更新マス用インデックスを増加 */
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ南東のノードが更新予定値と同じ場合 */
            } else {
              b_qY = col_num_node[i74] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[i76] == (int)b_qY) {
                /* 移動方向を追加 */
                if (g_d_direction.South_East <= 7) {
                  i121 = (unsigned char)(1 << g_d_direction.South_East);
                } else {
                  i121 = 0;
                }

                row_dir_node[i76] = (unsigned char)(row_dir_node[i76] | i121);
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
        i107 = (unsigned char)(1 << g_direction.South);
      } else {
        i107 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) - 1] &
            i107) != 0) == wall->contents.nowall) {
        c_qY = u6 - 1U;
        if (c_qY > u6) {
          c_qY = 0U;
        }

        if (g_direction.South <= 7) {
          i111 = (unsigned char)(1 << g_direction.South);
        } else {
          i111 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
              - 1] & i111) != 0) == search->contents.known) {
          /* かつ進行方向が南西向きである時 */
          if (g_d_direction.South_West <= 7) {
            i119 = (unsigned char)(1 << g_d_direction.South_West);
          } else {
            i119 = 0;
          }

          if ((col_dir_node[i74] & i119) != 0) {
            /* かつ南西のノードが更新予定値よりも大きな値の場合 */
            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 4U;
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
                - 1] + 4U;
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
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 4U;
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
                  i139 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i139 = 0;
                }

                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = (unsigned char)(row_dir_node[(contor_renew_node_col[n] +
                  33 * ((int)b_qY - 1)) - 1] | i139);
              }
            }

            /* かつ進行方向が南西向きでないとき */
          } else {
            /* かつ南西のノードの歩数MAP値が、更新予定値より大きい場合 */
            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i74] + 18U;
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
              i76 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i76;

              /* かつ南西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i74] + 18U;
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
                  i138 = (unsigned char)(1 << g_d_direction.South_West);
                } else {
                  i138 = 0;
                }

                row_dir_node[(contor_renew_node_col[n] + 33 * ((int)c_qY - 1)) -
                  1] = (unsigned char)(row_dir_node[(contor_renew_node_col[n] +
                  33 * ((int)b_qY - 1)) - 1] | i138);
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
        i117 = (unsigned char)(1 << g_direction.West);
      } else {
        i117 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) - 1] &
            i117) != 0) == wall->contents.nowall) {
        c_qY = u6 - 1U;
        if (c_qY > u6) {
          c_qY = 0U;
        }

        if (g_direction.West <= 7) {
          i125 = (unsigned char)(1 << g_direction.West);
        } else {
          i125 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
              - 1] & i125) != 0) == search->contents.known) {
          /* かつ進行方向が西向きである時 */
          if (g_d_direction.West <= 7) {
            i130 = (unsigned char)(1 << g_d_direction.West);
          } else {
            i130 = 0;
          }

          if ((col_dir_node[i74] & i130) != 0) {
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
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

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
                  i145 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i145 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = (unsigned char)(col_dir_node[(contor_renew_node_col[n]
                  + (((int)b_qY - 1) << 5)) - 1] | i145);
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
              i76 = (int)(contor_renew_node_col_idx_temp + 1U);
              if ((unsigned int)i76 > 255U) {
                i76 = 255;
              }

              contor_renew_node_col_idx_temp = (unsigned char)i76;

              /* かつ西のノードが更新予定値と同じ場合 */
            } else {
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[i74] + 18U;
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
                  i144 = (unsigned char)(1 << g_d_direction.West);
                } else {
                  i144 = 0;
                }

                col_dir_node[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
                  - 1] = (unsigned char)(col_dir_node[(contor_renew_node_col[n]
                  + (((int)b_qY - 1) << 5)) - 1] | i144);
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
        i129 = (unsigned char)(1 << g_direction.North);
      } else {
        i129 = 0;
      }

      if (((maze_wall[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5)) - 1] &
            i129) != 0) == wall->contents.nowall) {
        c_qY = u6 - 1U;
        if (c_qY > u6) {
          c_qY = 0U;
        }

        if (g_direction.North <= 7) {
          i132 = (unsigned char)(1 << g_direction.North);
        } else {
          i132 = 0;
        }

        if (((maze_wall_search[(contor_renew_node_col[n] + (((int)c_qY - 1) << 5))
              - 1] & i132) != 0) == search->contents.known) {
          /* かつ進行方向が北西向きである時 */
          if (g_d_direction.North_West <= 7) {
            i133 = (unsigned char)(1 << g_d_direction.North_West);
          } else {
            i133 = 0;
          }

          if ((col_dir_node[(contor_renew_node_col[n] +
                             ((contor_renew_node_col[n + 1024] - 1) << 5)) - 1]
               & i133) != 0) {
            /* かつ北西のノードが更新予定値よりも大きな値の場合 */
            i74 = (int)(contor_renew_node_col[n] + 1U);
            if ((unsigned int)i74 > 255U) {
              i74 = 255;
            }

            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[(contor_renew_node_col[n] +
                                 ((contor_renew_node_col[n + 1024] - 1) << 5)) -
              1] + 4U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i74 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              row_num_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i74;
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i74;

              /* かつ北西のノードが更新予定値と同じ場合 */
            } else {
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              b_qY = col_num_node[(contor_renew_node_col[n] +
                                   ((contor_renew_node_col[n + 1024] - 1) << 5))
                - 1] + 4U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i74 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i74 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i74 > 255U) {
                  i74 = 255;
                }

                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                i76 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i147 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i147 = 0;
                }

                row_dir_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i76 + 33 * ((int)b_qY - 1)) - 1] | i147);
              }
            }

            /* かつ進行方向が北西向きでないとき */
          } else {
            /* かつ北西のノードの歩数MAP値が、更新予定値より大きい場合 */
            i76 = (int)(contor_renew_node_col[n] + 1U);
            if ((unsigned int)i76 > 255U) {
              i76 = 255;
            }

            c_qY = u6 - 1U;
            if (c_qY > u6) {
              c_qY = 0U;
            }

            b_qY = col_num_node[i74] + 18U;
            if (b_qY > 65535U) {
              b_qY = 65535U;
            }

            if (row_num_node[(i76 + 33 * ((int)c_qY - 1)) - 1] > (int)b_qY) {
              /* 歩数MAP更新(重みづけあり) */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
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

              row_num_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = (unsigned short)
                b_qY;

              /* 移動方向MAP更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              if (g_d_direction.North_West <= 7) {
                row_dir_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (1 << g_d_direction.North_West);
              } else {
                row_dir_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = 0U;
              }

              /* 更新フラグを立てる */
              change_flag = 1U;

              /* 更新ノードを更新 */
              i74 = (int)(contor_renew_node_col[n] + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp - 1] =
                (unsigned char)i74;
              c_qY = u6 - 1U;
              if (c_qY > u6) {
                c_qY = 0U;
              }

              contor_renew_node_row_temp[contor_renew_node_row_idx_temp + 1023] =
                (unsigned char)c_qY;

              /* 更新マス用インデックスを増加 */
              i74 = (int)(contor_renew_node_row_idx_temp + 1U);
              if ((unsigned int)i74 > 255U) {
                i74 = 255;
              }

              contor_renew_node_row_idx_temp = (unsigned char)i74;

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

              b_qY = col_num_node[i74] + 18U;
              if (b_qY > 65535U) {
                b_qY = 65535U;
              }

              if (row_num_node[(i76 + 33 * ((int)c_qY - 1)) - 1] == (int)b_qY) {
                /* 移動方向を追加 */
                i74 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i74 > 255U) {
                  i74 = 255;
                }

                c_qY = u6 - 1U;
                if (c_qY > u6) {
                  c_qY = 0U;
                }

                i76 = (int)(contor_renew_node_col[n] + 1U);
                if ((unsigned int)i76 > 255U) {
                  i76 = 255;
                }

                b_qY = u6 - 1U;
                if (b_qY > u6) {
                  b_qY = 0U;
                }

                if (g_d_direction.North_West <= 7) {
                  i146 = (unsigned char)(1 << g_d_direction.North_West);
                } else {
                  i146 = 0;
                }

                row_dir_node[(i74 + 33 * ((int)c_qY - 1)) - 1] = (unsigned char)
                  (row_dir_node[(i76 + 33 * ((int)b_qY - 1)) - 1] | i146);
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
 * Arguments    : const coder_internal_ref_5 *wall
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned short contour_map[1024]
 *                unsigned char new_goal[2]
 * Return Type  : void
 */
static void make_new_goal_all(const coder_internal_ref_5 *wall, const unsigned
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
  int i26;
  bool guard1 = false;
  bool guard2 = false;
  bool guard3 = false;
  int i27;
  int i28;
  int i29;
  int i30;
  int i31;
  int i32;
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
        i26 = (unsigned char)(1 << g_direction.North);
      } else {
        i26 = 0;
      }

      guard1 = false;
      guard2 = false;
      guard3 = false;
      if ((q0 & i26) == wall->contents.nowall) {
        /* 北側のMAPが更新されているか判断、されていなければ書き込み */
        i27 = (int)(contor_renew_square[tempn] + 1U);
        if ((unsigned int)i27 > 255U) {
          i27 = 255;
        }

        i28 = (contor_renew_square[tempn + 1024] - 1) << 5;
        if (contour_map[(i27 + i28) - 1] == 65535) {
          i27 = (int)(contor_renew_square[tempn] + 1U);
          i30 = i27;
          if ((unsigned int)i27 > 255U) {
            i30 = 255;
          }

          u1 = tempi + 2U;
          if (u1 > 65535U) {
            u1 = 65535U;
          }

          contour_map[(i30 + i28) - 1] = (unsigned short)(65535 - (int)u1);
          change_flag = 1U;

          /* 更新マスを更新 */
          i30 = i27;
          if ((unsigned int)i27 > 255U) {
            i30 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)i30;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square[tempn + 1024];

          /* 更新マス用インデックスを増加 */
          i30 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i30 > 255U) {
            i30 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i30;

          /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
          i30 = i27;
          if ((unsigned int)i27 > 255U) {
            i30 = 255;
          }

          if (maze_wall_search[(i30 + i28) - 1] != 15) {
            new_goal[0] = contor_renew_square[tempn + 1024];
            if ((unsigned int)i27 > 255U) {
              i27 = 255;
            }

            new_goal[1] = (unsigned char)i27;
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
          i29 = (unsigned char)(1 << g_direction.East);
        } else {
          i29 = 0;
        }

        if ((q0 & i29) == wall->contents.nowall) {
          /* 東側のMAPが更新されているか判断、されていなければ書き込み */
          q0 = (int)(contor_renew_square[tempn + 1024] + 1U);
          i27 = q0;
          if ((unsigned int)q0 > 255U) {
            i27 = 255;
          }

          if (contour_map[(contor_renew_square[tempn] + ((i27 - 1) << 5)) - 1] ==
              65535) {
            i27 = q0;
            if ((unsigned int)q0 > 255U) {
              i27 = 255;
            }

            u1 = tempi + 2U;
            if (u1 > 65535U) {
              u1 = 65535U;
            }

            contour_map[(contor_renew_square[tempn] + ((i27 - 1) << 5)) - 1] =
              (unsigned short)(65535 - (int)u1);
            change_flag = 1U;

            /* 更新マスを更新 */
            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              contor_renew_square[tempn];
            i27 = q0;
            if ((unsigned int)q0 > 255U) {
              i27 = 255;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              (unsigned char)i27;

            /* 更新マス用インデックスを増加 */
            i27 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i27 > 255U) {
              i27 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i27;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            i27 = q0;
            if ((unsigned int)q0 > 255U) {
              i27 = 255;
            }

            if (maze_wall_search[(contor_renew_square[tempn] + ((i27 - 1) << 5))
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
          i31 = (unsigned char)(1 << g_direction.South);
        } else {
          i31 = 0;
        }

        if ((q0 & i31) == wall->contents.nowall) {
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
            i27 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i27 > 255U) {
              i27 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i27;

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
          i32 = (unsigned char)(1 << g_direction.West);
        } else {
          i32 = 0;
        }

        if ((q0 & i32) == wall->contents.nowall) {
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
 * Arguments    : const coder_internal_ref_5 *wall
 *                const unsigned char maze_wall[1024]
 *                unsigned char current_x
 *                unsigned char current_y
 *                const unsigned char unexp_square[1024]
 *                unsigned char unexp_square_idx
 *                unsigned short contour_map[1024]
 *                unsigned char new_goal[2]
 * Return Type  : void
 */
static void make_new_goal_sh(const coder_internal_ref_5 *wall, const unsigned
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
  int i60;
  bool guard1 = false;
  bool guard2 = false;
  bool guard3 = false;
  int i61;
  int i62;
  int i63;
  int i64;
  int i65;
  int i66;
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
        i60 = (unsigned char)(1 << g_direction.North);
      } else {
        i60 = 0;
      }

      guard1 = false;
      guard2 = false;
      guard3 = false;
      if ((q0 & i60) == wall->contents.nowall) {
        /* 北側のMAPが更新されているか判断、されていなければ書き込み */
        i61 = (int)(contor_renew_square[tempn] + 1U);
        if ((unsigned int)i61 > 255U) {
          i61 = 255;
        }

        i62 = (contor_renew_square[tempn + 1024] - 1) << 5;
        if (contour_map[(i61 + i62) - 1] == 65535) {
          i61 = (int)(contor_renew_square[tempn] + 1U);
          i64 = i61;
          if ((unsigned int)i61 > 255U) {
            i64 = 255;
          }

          u4 = tempi + 2U;
          if (u4 > 65535U) {
            u4 = 65535U;
          }

          contour_map[(i64 + i62) - 1] = (unsigned short)(65535 - (int)u4);
          change_flag = 1U;

          /* 更新マスを更新 */
          i62 = i61;
          if ((unsigned int)i61 > 255U) {
            i62 = 255;
          }

          contor_renew_square_temp[contor_renew_square_idx_temp - 1] = (unsigned
            char)i62;
          contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
            contor_renew_square[tempn + 1024];

          /* 更新マス用インデックスを増加 */
          i62 = (int)(contor_renew_square_idx_temp + 1U);
          if ((unsigned int)i62 > 255U) {
            i62 = 255;
          }

          contor_renew_square_idx_temp = (unsigned char)i62;

          /* 更新した地点が最短経路未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
          i62 = i61;
          if ((unsigned int)i61 > 255U) {
            i62 = 255;
          }

          if (sh_route_unexp_sq_jud(unexp_square, unexp_square_idx, (unsigned
                char)i62, contor_renew_square[tempn + 1024]) == 1) {
            new_goal[0] = contor_renew_square[tempn + 1024];
            if ((unsigned int)i61 > 255U) {
              i61 = 255;
            }

            new_goal[1] = (unsigned char)i61;
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
          i63 = (unsigned char)(1 << g_direction.East);
        } else {
          i63 = 0;
        }

        if ((q0 & i63) == wall->contents.nowall) {
          /* 東側のMAPが更新されているか判断、されていなければ書き込み */
          q0 = (int)(contor_renew_square[tempn + 1024] + 1U);
          i61 = q0;
          if ((unsigned int)q0 > 255U) {
            i61 = 255;
          }

          if (contour_map[(contor_renew_square[tempn] + ((i61 - 1) << 5)) - 1] ==
              65535) {
            i61 = q0;
            if ((unsigned int)q0 > 255U) {
              i61 = 255;
            }

            u4 = tempi + 2U;
            if (u4 > 65535U) {
              u4 = 65535U;
            }

            contour_map[(contor_renew_square[tempn] + ((i61 - 1) << 5)) - 1] =
              (unsigned short)(65535 - (int)u4);
            change_flag = 1U;

            /* 更新マスを更新 */
            contor_renew_square_temp[contor_renew_square_idx_temp - 1] =
              contor_renew_square[tempn];
            i61 = q0;
            if ((unsigned int)q0 > 255U) {
              i61 = 255;
            }

            contor_renew_square_temp[contor_renew_square_idx_temp + 1023] =
              (unsigned char)i61;

            /* 更新マス用インデックスを増加 */
            i61 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i61 > 255U) {
              i61 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i61;

            /* 更新した地点が未探索領域であれば、そこを新規ゴール点とし、コンター展開を終了する。 */
            i61 = q0;
            if ((unsigned int)q0 > 255U) {
              i61 = 255;
            }

            if (sh_route_unexp_sq_jud(unexp_square, unexp_square_idx,
                 contor_renew_square[tempn], (unsigned char)i61) == 1) {
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
          i65 = (unsigned char)(1 << g_direction.South);
        } else {
          i65 = 0;
        }

        if ((q0 & i65) == wall->contents.nowall) {
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
            i61 = (int)(contor_renew_square_idx_temp + 1U);
            if ((unsigned int)i61 > 255U) {
              i61 = 255;
            }

            contor_renew_square_idx_temp = (unsigned char)i61;

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
          i66 = (unsigned char)(1 << g_direction.West);
        } else {
          i66 = 0;
        }

        if ((q0 & i66) == wall->contents.nowall) {
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
 * Arguments    : const unsigned short row_num_node[1056]
 *                const unsigned short col_num_node[1056]
 *                const unsigned char goal_section[2]
 *                const unsigned char goal_node2[2]
 *                unsigned char goal_node_property
 * Return Type  : void
 */
static void make_route_diagonal(const unsigned short row_num_node[1056], const
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
  unsigned int qY;
  unsigned char b_next_move_dir;

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

        if (p && (next_node_property == goal_node_property)) {
          /* ゴールの場合、ゴール進入時のパターンを決定する。 */
          get_next_dir_diagonal(row_num_node, col_num_node, next_move_dir,
                                b_next_node, next_node_property, goal_node2,
                                goal_node_property, goal_section,
                                &b_next_move_dir, next_node,
                                &b_next_node_property);

          /* 直進の場合（直進侵入） */
          if (next_move_dir == b_next_move_dir) {
            /* ゴール分カウンタを増加 */
            i = (int)((unsigned char)i + 1U);
            if ((unsigned int)i > 255U) {
              i = 255;
            }

            /* 現在ノードから直進カウンタ分移動する関数。 */
            move_straight(current_node, &current_node_property,
                          &current_move_dir, &current_move_mode, (unsigned char)
                          i);
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
            /* 現在ノードから直進カウンタ分移動する関数。 */
            move_straight(current_node, &current_node_property,
                          &current_move_dir, &current_move_mode, (unsigned char)
                          i);
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
            turn_pattern_num = get_turn_pattern_num(move_dir_buffer,
              ref_move_mode);

            /* ターンの軌跡を描画する */
            /* パターンが決まらなかった場合、エラー */
            /* パターンに応じた移動処理 */
            if (turn_pattern_num == turn_pattern.r_45) {
              turn_r_45(current_node, &current_node_property, &current_move_dir,
                        &current_move_mode);
            } else {
              if (turn_pattern_num == turn_pattern.l_45) {
                turn_l_45(current_node, &current_node_property,
                          &current_move_dir, &current_move_mode);
              }
            }
          }

          /* 参照位置がセクションであるとき */
        } else {
          /* 直進カウンタがあれば、移動する。 */
          /*                      disp("ゴール時直進（セクション）") */
          /* 軌跡をプロット */
          /* 現在ノードから直進カウンタ分移動する関数。 */
          move_straight(current_node, &current_node_property, &current_move_dir,
                        &current_move_mode, (unsigned char)i);
          straight_count = 0U;
        }

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
        turn_r_45(current_node, &current_node_property, &current_move_dir,
                  &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.l_45) {
        turn_l_45(current_node, &current_node_property, &current_move_dir,
                  &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.r_90) {
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
        turn_l_90(current_node, &current_node_property, &current_move_dir,
                  &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.r_135) {
        turn_r_135(current_node, &current_node_property, &current_move_dir,
                   &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.l_135) {
        turn_l_135(current_node, &current_node_property, &current_move_dir,
                   &current_move_mode);
      } else if (turn_pattern_num == turn_pattern.r_180) {
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
          turn_l_180(current_node, &current_node_property, &current_move_dir,
                     &current_move_mode);
        }
      }

      /*              disp("ターン終了時、ノード") */
      /*              disp(current_node) */
      /*              disp(goal_node2) */
      /* 移動後、ゴールを判定 */
      if (isequal(current_node, goal_node2) && (current_node_property ==
           goal_node_property)) {
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

  /*     %% move_step 一マス前進する関数 */
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
 * Arguments    : const coder_internal_ref_5 *wall
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_4 *search
 *                const coder_internal_ref_1 *maze_goal
 *                const coder_internal_ref_3 *adachi_search_mode
 *                unsigned char *current_x
 *                unsigned char *current_y
 *                unsigned char *current_dir
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 *                const unsigned char exploration_goal[18]
 *                unsigned char l_goal_size
 *                unsigned char *start_flg
 *                unsigned char adachi_s_mode
 *                unsigned short contour_map[1024]
 * Return Type  : void
 */
static void search_adachi(const coder_internal_ref_5 *wall, coder_internal_ref
  *wall_flg, const coder_internal_ref_4 *search, const coder_internal_ref_1
  *maze_goal, const coder_internal_ref_3 *adachi_search_mode, unsigned char
  *current_x, unsigned char *current_y, unsigned char *current_dir, unsigned
  char maze_row_size, unsigned char maze_col_size, unsigned char maze_wall[1024],
  unsigned char maze_wall_search[1024], const unsigned char exploration_goal[18],
  unsigned char l_goal_size, unsigned char *start_flg, unsigned char
  adachi_s_mode, unsigned short contour_map[1024])
{
  unsigned char goal_flg;
  unsigned char contour_flg;
  int i224;
  int exitg1;
  unsigned char next_dir;
  int i;
  bool exitg2;
  unsigned int qY;
  *start_flg = 0U;

  /*     %% search_adachi 足立法での探索 */
  /* local変数宣言 */
  goal_flg = 0U;

  /* ゴール判定フラグ */
  /* 壁情報更新確認用変数 */
  contour_flg = 0U;

  /*      search_start_x = current_x %探索開始時x */
  /*      search_start_y = current_y %探索開始時y */
  /* 初回のコンターマップ作製 */
  make_map_find(wall, exploration_goal, l_goal_size, maze_wall, *current_x,
                *current_y, contour_map);
  i224 = l_goal_size;
  do {
    exitg1 = 0;

    /* 壁情報取得 */
    /* ゴール直後は壁情報を更新しない */
    next_dir = maze_wall[(*current_y + ((*current_x - 1) << 5)) - 1];
    wall_set(wall, wall_flg, search, maze_goal, maze_row_size, maze_col_size,
             *current_x, *current_y, *current_dir, maze_wall, maze_wall_search);

    /* 壁情報が更新されれば、コンター更新のフラグを立てる。 */
    if (next_dir != maze_wall[(*current_y + ((*current_x - 1) << 5)) - 1]) {
      contour_flg = 1U;
    }

    /*  等高線MAP生成 */
    /* 壁情報に変更があった場合のみ */
    if (contour_flg != 0) {
      make_map_find(wall, exploration_goal, l_goal_size, maze_wall, *current_x, *
                    current_y, contour_map);
    }

    /* 現在位置がゴールか判定 */
    for (i = 0; i < i224; i++) {
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
      next_dir = get_nextdir2(*current_x, *current_y, maze_wall, contour_map);

      /*  現在方向と進行方向に応じた処理 */
      i = (int)(4U + next_dir);
      if ((unsigned int)i > 255U) {
        i = 255;
      }

      qY = (unsigned int)i - *current_dir;
      if (qY > (unsigned int)i) {
        qY = 0U;
      }

      next_dir = b_rem((unsigned char)qY);
      if (l_direction.front == next_dir) {
        i = 0;
      } else if (l_direction.right == next_dir) {
        i = 1;
      } else if (l_direction.back == next_dir) {
        i = 2;
      } else if (l_direction.left == next_dir) {
        i = 3;
      } else {
        i = -1;
      }

      switch (i) {
       case 0:
        move_step(current_x, current_y, *current_dir);

        /* disp("front") */
        m_move_front(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 1:
        turn_clk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* disp("right") */
        m_move_right(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 2:
        turn_180deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* disp("back") */
        m_move_back(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;

       case 3:
        turn_conclk_90deg(current_dir);
        move_step(current_x, current_y, *current_dir);

        /* disp("left") */
        m_move_left(0, wall_flg->contents, move_dir_property.straight);

        /* スタート直後フラグをクリア */
        *start_flg = 0U;

        /* 壁フラグをクリア */
        wall_flg->contents = 0U;
        break;
      }

      /* for code generation */
    }
  } while (exitg1 == 0);

  /* ゴール時停止フラグが立っているとき */
  /* 停止動作を実施 */
  m_goal_movement(0, wall_flg->contents, move_dir_property.straight);

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
 * Arguments    : const coder_internal_ref_5 *wall
 *                coder_internal_ref *wall_flg
 *                const coder_internal_ref_4 *search
 *                const coder_internal_ref_1 *maze_goal
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char current_x
 *                unsigned char current_y
 *                unsigned char current_dir
 *                unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 * Return Type  : void
 */
static void wall_set(const coder_internal_ref_5 *wall, coder_internal_ref
                     *wall_flg, const coder_internal_ref_4 *search, const
                     coder_internal_ref_1 *maze_goal, unsigned char
                     maze_row_size, unsigned char maze_col_size, unsigned char
                     current_x, unsigned char current_y, unsigned char
                     current_dir, unsigned char maze_wall[1024], unsigned char
                     maze_wall_search[1024])
{
  unsigned char wall_write[4];
  unsigned char serch_write[4];
  short wall_sensor_front;
  int i225;
  int ex;
  int i226;
  int i227;
  int k;
  int i228;
  int i229;
  int i230;
  int i231;
  int i232;
  int i233;
  int i234;
  int i235;
  int i236;
  int i237;
  int i238;
  unsigned int qY;
  int i239;
  int i240;
  int i241;
  int i242;
  signed char varargin_1[9];
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
  int i276;
  unsigned int b_qY;
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
    i225 = (int)(b_rem(current_dir) + 1U);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    wall_write[i225 - 1] = wall->contents.wall;

    /* 壁フラグセット */
    wall_flg->contents |= 1;
  }

  /* 探索情報取更新 */
  i225 = (int)(b_rem(current_dir) + 1U);
  if ((unsigned int)i225 > 255U) {
    i225 = 255;
  }

  serch_write[i225 - 1] = search->contents.known;

  /* 右壁判定 */
  /* for Cgen */
  /* センサ値取得 */
  wall_sensor_front = m_get_right_sensor();

  /* センサ値をもとに、壁の有無を判定 */
  if (wall_sensor_front > wall_sensor_right_th) {
    /* 壁情報取得 */
    i225 = (int)(current_dir + 1U);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    i225 = (int)(b_rem((unsigned char)i225) + 1U);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    wall_write[i225 - 1] = wall->contents.wall;

    /* 壁フラグセット */
    wall_flg->contents = (unsigned char)(wall_flg->contents | 2);
  }

  /* 探索情報取更新 */
  i225 = (int)(current_dir + 1U);
  if ((unsigned int)i225 > 255U) {
    i225 = 255;
  }

  i225 = (int)(b_rem((unsigned char)i225) + 1U);
  if ((unsigned int)i225 > 255U) {
    i225 = 255;
  }

  serch_write[i225 - 1] = search->contents.known;

  /* 後方は情報を得ることができないので処理しない。 */
  /* 左壁判定 */
  /* for Cgen */
  /* センサ値取得 */
  wall_sensor_front = m_get_left_sensor();

  /* センサ値をもとに、壁の有無を判定 */
  if (wall_sensor_front > wall_sensor_left_th) {
    /* 壁情報取得 */
    i225 = (int)(current_dir + 3U);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    i225 = (int)(b_rem((unsigned char)i225) + 1U);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    wall_write[i225 - 1] = wall->contents.wall;

    /* 壁フラグセット */
    wall_flg->contents = (unsigned char)(wall_flg->contents | 8);
  }

  /* 探索情報取更新 */
  i225 = (int)(current_dir + 3U);
  if ((unsigned int)i225 > 255U) {
    i225 = 255;
  }

  i225 = (int)(b_rem((unsigned char)i225) + 1U);
  if ((unsigned int)i225 > 255U) {
    i225 = 255;
  }

  serch_write[i225 - 1] = search->contents.known;

  /* ここまで */
  /* 壁情報,探索情報を入力 */
  /* 北側 */
  i225 = (int)(g_direction.North + 1U);
  ex = i225;
  if ((unsigned int)i225 > 255U) {
    ex = 255;
  }

  if (g_direction.North <= 7) {
    i226 = (unsigned char)(1 << g_direction.North);
  } else {
    i226 = 0;
  }

  ex = (int)((unsigned int)i226 * wall_write[ex - 1]);
  if ((unsigned int)ex > 255U) {
    ex = 255;
  }

  i227 = (current_x - 1) << 5;
  k = current_y + i227;
  i228 = k - 1;
  maze_wall[i228] = (unsigned char)(maze_wall[i228] | ex);
  ex = i225;
  if ((unsigned int)i225 > 255U) {
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

  maze_wall_search[i228] = (unsigned char)(maze_wall_search[i228] | ex);

  /* 東側 */
  ex = (int)(g_direction.East + 1U);
  i230 = ex;
  if ((unsigned int)ex > 255U) {
    i230 = 255;
  }

  if (g_direction.East <= 7) {
    i231 = (unsigned char)(1 << g_direction.East);
  } else {
    i231 = 0;
  }

  i230 = (int)((unsigned int)i231 * wall_write[i230 - 1]);
  if ((unsigned int)i230 > 255U) {
    i230 = 255;
  }

  maze_wall[i228] = (unsigned char)(maze_wall[i228] | i230);
  i230 = ex;
  if ((unsigned int)ex > 255U) {
    i230 = 255;
  }

  if (g_direction.East <= 7) {
    i232 = (unsigned char)(1 << g_direction.East);
  } else {
    i232 = 0;
  }

  i230 = (int)((unsigned int)i232 * serch_write[i230 - 1]);
  if ((unsigned int)i230 > 255U) {
    i230 = 255;
  }

  maze_wall_search[i228] = (unsigned char)(maze_wall_search[i228] | i230);

  /* 南側 */
  i230 = (int)(g_direction.South + 1U);
  i233 = i230;
  if ((unsigned int)i230 > 255U) {
    i233 = 255;
  }

  if (g_direction.South <= 7) {
    i234 = (unsigned char)(1 << g_direction.South);
  } else {
    i234 = 0;
  }

  i233 = (int)((unsigned int)i234 * wall_write[i233 - 1]);
  if ((unsigned int)i233 > 255U) {
    i233 = 255;
  }

  maze_wall[i228] = (unsigned char)(maze_wall[i228] | i233);
  i233 = i230;
  if ((unsigned int)i230 > 255U) {
    i233 = 255;
  }

  if (g_direction.South <= 7) {
    i235 = (unsigned char)(1 << g_direction.South);
  } else {
    i235 = 0;
  }

  i233 = (int)((unsigned int)i235 * serch_write[i233 - 1]);
  if ((unsigned int)i233 > 255U) {
    i233 = 255;
  }

  maze_wall_search[i228] = (unsigned char)(maze_wall_search[i228] | i233);

  /* 西側 */
  i233 = (int)(g_direction.West + 1U);
  i236 = i233;
  if ((unsigned int)i233 > 255U) {
    i236 = 255;
  }

  if (g_direction.West <= 7) {
    i237 = (unsigned char)(1 << g_direction.West);
  } else {
    i237 = 0;
  }

  i236 = (int)((unsigned int)i237 * wall_write[i236 - 1]);
  if ((unsigned int)i236 > 255U) {
    i236 = 255;
  }

  maze_wall[i228] = (unsigned char)(maze_wall[i228] | i236);
  i236 = i233;
  if ((unsigned int)i233 > 255U) {
    i236 = 255;
  }

  if (g_direction.West <= 7) {
    i238 = (unsigned char)(1 << g_direction.West);
  } else {
    i238 = 0;
  }

  i236 = (int)((unsigned int)i238 * serch_write[i236 - 1]);
  if ((unsigned int)i236 > 255U) {
    i236 = 255;
  }

  maze_wall_search[i228] = (unsigned char)(maze_wall_search[i228] | i236);

  /* 隣り合うマスの情報にも入力 */
  /* 北側のマスの南側の壁情報 */
  qY = maze_row_size - 1U;
  if (qY > maze_row_size) {
    qY = 0U;
  }

  if (current_y < (int)qY) {
    i236 = i225;
    if ((unsigned int)i225 > 255U) {
      i236 = 255;
    }

    if (g_direction.South <= 7) {
      i239 = (unsigned char)(1 << g_direction.South);
    } else {
      i239 = 0;
    }

    i236 = (int)((unsigned int)i239 * wall_write[i236 - 1]);
    if ((unsigned int)i236 > 255U) {
      i236 = 255;
    }

    maze_wall[k] = (unsigned char)(maze_wall[k] | i236);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    if (g_direction.South <= 7) {
      i242 = (unsigned char)(1 << g_direction.South);
    } else {
      i242 = 0;
    }

    i225 = (int)((unsigned int)i242 * serch_write[i225 - 1]);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    maze_wall_search[k] = (unsigned char)(maze_wall_search[k] | i225);
  }

  /* 東側のマスの西側の壁情報 */
  qY = maze_col_size - 1U;
  if (qY > maze_col_size) {
    qY = 0U;
  }

  if (current_x < (int)qY) {
    i225 = ex;
    if ((unsigned int)ex > 255U) {
      i225 = 255;
    }

    if (g_direction.West <= 7) {
      i240 = (unsigned char)(1 << g_direction.West);
    } else {
      i240 = 0;
    }

    i225 = (int)((unsigned int)i240 * wall_write[i225 - 1]);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    i236 = (current_y + (current_x << 5)) - 1;
    maze_wall[i236] = (unsigned char)(maze_wall[i236] | i225);
    if ((unsigned int)ex > 255U) {
      ex = 255;
    }

    if (g_direction.West <= 7) {
      i244 = (unsigned char)(1 << g_direction.West);
    } else {
      i244 = 0;
    }

    i225 = (int)((unsigned int)i244 * serch_write[ex - 1]);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    maze_wall_search[i236] = (unsigned char)(maze_wall_search[i236] | i225);
  }

  /* 南側のマスの北側の壁情報 */
  if (current_y > 1) {
    i225 = i230;
    if ((unsigned int)i230 > 255U) {
      i225 = 255;
    }

    if (g_direction.North <= 7) {
      i241 = (unsigned char)(1 << g_direction.North);
    } else {
      i241 = 0;
    }

    i225 = (int)((unsigned int)i241 * wall_write[i225 - 1]);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    ex = k - 2;
    maze_wall[ex] = (unsigned char)(maze_wall[ex] | i225);
    if ((unsigned int)i230 > 255U) {
      i230 = 255;
    }

    if (g_direction.North <= 7) {
      i246 = (unsigned char)(1 << g_direction.North);
    } else {
      i246 = 0;
    }

    i225 = (int)((unsigned int)i246 * serch_write[i230 - 1]);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    maze_wall_search[ex] = (unsigned char)(maze_wall_search[ex] | i225);
  }

  /* 西側のマスの東側の壁情報 */
  if (current_x > 1) {
    i225 = i233;
    if ((unsigned int)i233 > 255U) {
      i225 = 255;
    }

    if (g_direction.East <= 7) {
      i243 = (unsigned char)(1 << g_direction.East);
    } else {
      i243 = 0;
    }

    i225 = (int)((unsigned int)i243 * wall_write[i225 - 1]);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    ex = (current_y + ((current_x - 2) << 5)) - 1;
    maze_wall[ex] = (unsigned char)(maze_wall[ex] | i225);
    if ((unsigned int)i233 > 255U) {
      i233 = 255;
    }

    if (g_direction.East <= 7) {
      i248 = (unsigned char)(1 << g_direction.East);
    } else {
      i248 = 0;
    }

    i225 = (int)((unsigned int)i248 * serch_write[i233 - 1]);
    if ((unsigned int)i225 > 255U) {
      i225 = 255;
    }

    maze_wall_search[ex] = (unsigned char)(maze_wall_search[ex] | i225);
  }

  /* 現在地がゴールでない場合 */
  for (i225 = 0; i225 < 9; i225++) {
    varargin_1[i225] = (signed char)((maze_goal->contents[i225] == current_x) *
      (maze_goal->contents[9 + i225] == current_y));
  }

  ex = varargin_1[0];
  for (k = 0; k < 8; k++) {
    i225 = varargin_1[k + 1];
    if (ex < i225) {
      ex = i225;
    }
  }

  if (ex == 0) {
    /* 柱に対し、3方向探索済みかつ、すべて壁がない場合、もう一方向の壁を確定させる。 */
    /* 北,東に壁がない場合 */
    if (g_direction.North <= 7) {
      i245 = (unsigned char)(1 << g_direction.North);
    } else {
      i245 = 0;
    }

    if (g_direction.North <= 7) {
      i247 = (unsigned char)(1 << g_direction.North);
    } else {
      i247 = 0;
    }

    if ((maze_wall[i228] & i245) != i247) {
      if (g_direction.East <= 7) {
        i249 = (unsigned char)(1 << g_direction.East);
      } else {
        i249 = 0;
      }

      if (g_direction.East <= 7) {
        i251 = (unsigned char)(1 << g_direction.East);
      } else {
        i251 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i249) != i251)
      {
        /* 北のマスの東が探索済み　かつ　壁がないとき */
        i225 = (int)(current_y + 1U);
        ex = i225;
        if ((unsigned int)i225 > 255U) {
          ex = 255;
        }

        if (g_direction.East <= 7) {
          i256 = (unsigned char)(1 << g_direction.East);
        } else {
          i256 = 0;
        }

        if ((maze_wall_search[(ex + i227) - 1] & i256) != 0) {
          ex = i225;
          if ((unsigned int)i225 > 255U) {
            ex = 255;
          }

          if (g_direction.East <= 7) {
            i259 = (unsigned char)(1 << g_direction.East);
          } else {
            i259 = 0;
          }

          if (g_direction.East <= 7) {
            i264 = (unsigned char)(1 << g_direction.East);
          } else {
            i264 = 0;
          }

          if ((maze_wall[(ex + i227) - 1] & i259) != i264) {
            /* 東のマスの北の壁が確定、探索済みとする。 */
            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            k = (int)(current_x + 1U);
            if ((unsigned int)k > 255U) {
              k = 255;
            }

            if (g_direction.North <= 7) {
              i274 = (unsigned char)(1 << g_direction.North);
            } else {
              i274 = 0;
            }

            maze_wall[(current_y + ((ex - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + ((k - 1) << 5)) - 1] | i274);
            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            k = (int)(current_x + 1U);
            if ((unsigned int)k > 255U) {
              k = 255;
            }

            if (g_direction.North <= 7) {
              i283 = (unsigned char)(1 << g_direction.North);
            } else {
              i283 = 0;
            }

            maze_wall_search[(current_y + ((ex - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(current_y + ((k - 1) << 5)) - 1] | i283);

            /* 隣り合うマス（東北マス）の南の壁も確定 */
            ex = i225;
            if ((unsigned int)i225 > 255U) {
              ex = 255;
            }

            k = (int)(current_x + 1U);
            if ((unsigned int)k > 255U) {
              k = 255;
            }

            i228 = i225;
            if ((unsigned int)i225 > 255U) {
              i228 = 255;
            }

            i230 = (int)(current_x + 1U);
            if ((unsigned int)i230 > 255U) {
              i230 = 255;
            }

            if (g_direction.South <= 7) {
              i299 = (unsigned char)(1 << g_direction.South);
            } else {
              i299 = 0;
            }

            maze_wall[(ex + ((k - 1) << 5)) - 1] = (unsigned char)(maze_wall
              [(i228 + ((i230 - 1) << 5)) - 1] | i299);
            ex = i225;
            if ((unsigned int)i225 > 255U) {
              ex = 255;
            }

            k = (int)(current_x + 1U);
            if ((unsigned int)k > 255U) {
              k = 255;
            }

            i228 = i225;
            if ((unsigned int)i225 > 255U) {
              i228 = 255;
            }

            i230 = (int)(current_x + 1U);
            if ((unsigned int)i230 > 255U) {
              i230 = 255;
            }

            if (g_direction.South <= 7) {
              i305 = (unsigned char)(1 << g_direction.South);
            } else {
              i305 = 0;
            }

            maze_wall_search[(ex + ((k - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(i228 + ((i230 - 1) << 5)) - 1] | i305);
          }
        }

        /* 東のマスの北が探索済み　かつ　壁がないとき */
        ex = (int)(current_x + 1U);
        k = ex;
        if ((unsigned int)ex > 255U) {
          k = 255;
        }

        if (g_direction.North <= 7) {
          i263 = (unsigned char)(1 << g_direction.North);
        } else {
          i263 = 0;
        }

        if ((maze_wall_search[(current_y + ((k - 1) << 5)) - 1] & i263) != 0) {
          k = ex;
          if ((unsigned int)ex > 255U) {
            k = 255;
          }

          if (g_direction.North <= 7) {
            i270 = (unsigned char)(1 << g_direction.North);
          } else {
            i270 = 0;
          }

          if (g_direction.North <= 7) {
            i273 = (unsigned char)(1 << g_direction.North);
          } else {
            i273 = 0;
          }

          if ((maze_wall[(current_y + ((k - 1) << 5)) - 1] & i270) != i273) {
            /* 北のマスの東の壁が確定、探索済みとする。 */
            k = i225;
            if ((unsigned int)i225 > 255U) {
              k = 255;
            }

            i228 = i225;
            if ((unsigned int)i225 > 255U) {
              i228 = 255;
            }

            if (g_direction.East <= 7) {
              i282 = (unsigned char)(1 << g_direction.East);
            } else {
              i282 = 0;
            }

            maze_wall[(k + i227) - 1] = (unsigned char)(maze_wall[(i228 + i227)
              - 1] | i282);
            k = i225;
            if ((unsigned int)i225 > 255U) {
              k = 255;
            }

            i228 = i225;
            if ((unsigned int)i225 > 255U) {
              i228 = 255;
            }

            if (g_direction.East <= 7) {
              i292 = (unsigned char)(1 << g_direction.East);
            } else {
              i292 = 0;
            }

            maze_wall_search[(k + i227) - 1] = (unsigned char)(maze_wall_search
              [(i228 + i227) - 1] | i292);

            /* 隣り合うマス（東北マス）の西の壁も確定 */
            k = i225;
            if ((unsigned int)i225 > 255U) {
              k = 255;
            }

            i228 = ex;
            if ((unsigned int)ex > 255U) {
              i228 = 255;
            }

            i230 = i225;
            if ((unsigned int)i225 > 255U) {
              i230 = 255;
            }

            i233 = ex;
            if ((unsigned int)ex > 255U) {
              i233 = 255;
            }

            if (g_direction.West <= 7) {
              i304 = (unsigned char)(1 << g_direction.West);
            } else {
              i304 = 0;
            }

            maze_wall[(k + ((i228 - 1) << 5)) - 1] = (unsigned char)(maze_wall
              [(i230 + ((i233 - 1) << 5)) - 1] | i304);
            k = i225;
            if ((unsigned int)i225 > 255U) {
              k = 255;
            }

            i228 = ex;
            if ((unsigned int)ex > 255U) {
              i228 = 255;
            }

            if ((unsigned int)i225 > 255U) {
              i225 = 255;
            }

            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i310 = (unsigned char)(1 << g_direction.West);
            } else {
              i310 = 0;
            }

            maze_wall_search[(k + ((i228 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(i225 + ((ex - 1) << 5)) - 1] | i310);
          }
        }
      }
    }

    /* 北,西に壁がない場合 */
    if (g_direction.North <= 7) {
      i250 = (unsigned char)(1 << g_direction.North);
    } else {
      i250 = 0;
    }

    if (g_direction.North <= 7) {
      i252 = (unsigned char)(1 << g_direction.North);
    } else {
      i252 = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i250) != i252) {
      if (g_direction.West <= 7) {
        i253 = (unsigned char)(1 << g_direction.West);
      } else {
        i253 = 0;
      }

      if (g_direction.West <= 7) {
        i255 = (unsigned char)(1 << g_direction.West);
      } else {
        i255 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i253) != i255)
      {
        /* 北のマスの西が探索済み　かつ　壁がないとき */
        i225 = (int)(current_y + 1U);
        if ((unsigned int)i225 > 255U) {
          i225 = 255;
        }

        if (g_direction.West <= 7) {
          i261 = (unsigned char)(1 << g_direction.West);
        } else {
          i261 = 0;
        }

        if ((maze_wall_search[(i225 + i227) - 1] & i261) != 0) {
          i225 = (int)(current_y + 1U);
          if ((unsigned int)i225 > 255U) {
            i225 = 255;
          }

          if (g_direction.West <= 7) {
            i267 = (unsigned char)(1 << g_direction.West);
          } else {
            i267 = 0;
          }

          if (g_direction.West <= 7) {
            i272 = (unsigned char)(1 << g_direction.West);
          } else {
            i272 = 0;
          }

          if ((maze_wall[(i225 + i227) - 1] & i267) != i272) {
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
              i281 = (unsigned char)(1 << g_direction.North);
            } else {
              i281 = 0;
            }

            maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + (((int)b_qY - 1) << 5)) - 1] | i281);
            qY = current_x - 1U;
            if (qY > current_x) {
              qY = 0U;
            }

            b_qY = current_x - 1U;
            if (b_qY > current_x) {
              b_qY = 0U;
            }

            if (g_direction.North <= 7) {
              i291 = (unsigned char)(1 << g_direction.North);
            } else {
              i291 = 0;
            }

            maze_wall_search[(current_y + (((int)qY - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[(current_y + (((int)b_qY - 1) << 5)) - 1] |
                    i291);

            /* 隣り合うマス（北西マス）の南の壁も確定 */
            i225 = (int)(current_y + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall[(i225 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(ex + (((int)b_qY - 1) << 5)) - 1] | i301);
            i225 = (int)(current_y + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall_search[(i225 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(ex + (((int)b_qY - 1) << 5)) - 1] | i309);
          }
        }

        /* 西のマスの北が探索済み　かつ　壁がないとき */
        qY = current_x - 1U;
        if (qY > current_x) {
          qY = 0U;
        }

        if (g_direction.North <= 7) {
          i269 = (unsigned char)(1 << g_direction.North);
        } else {
          i269 = 0;
        }

        if ((maze_wall_search[(current_y + (((int)qY - 1) << 5)) - 1] & i269) !=
            0) {
          qY = current_x - 1U;
          if (qY > current_x) {
            qY = 0U;
          }

          if (g_direction.North <= 7) {
            i276 = (unsigned char)(1 << g_direction.North);
          } else {
            i276 = 0;
          }

          if (g_direction.North <= 7) {
            i280 = (unsigned char)(1 << g_direction.North);
          } else {
            i280 = 0;
          }

          if ((maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] & i276) != i280)
          {
            /* 北のマスの西の壁が確定、探索済みとする。 */
            i225 = (int)(current_y + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i290 = (unsigned char)(1 << g_direction.West);
            } else {
              i290 = 0;
            }

            maze_wall[(i225 + i227) - 1] = (unsigned char)(maze_wall[(ex + i227)
              - 1] | i290);
            i225 = (int)(current_y + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
            }

            ex = (int)(current_y + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.West <= 7) {
              i298 = (unsigned char)(1 << g_direction.West);
            } else {
              i298 = 0;
            }

            maze_wall_search[(i225 + i227) - 1] = (unsigned char)
              (maze_wall_search[(ex + i227) - 1] | i298);

            /* 隣り合うマス（北西マス）の東の壁も確定 */
            i225 = (int)(current_y + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall[(i225 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(ex + (((int)b_qY - 1) << 5)) - 1] | i307);
            i225 = (int)(current_y + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall_search[(i225 + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[(ex + (((int)b_qY - 1) << 5)) - 1] | i314);
          }
        }
      }
    }

    /* 南,東に壁がない場合 */
    if (g_direction.East <= 7) {
      i254 = (unsigned char)(1 << g_direction.East);
    } else {
      i254 = 0;
    }

    if (g_direction.East <= 7) {
      i257 = (unsigned char)(1 << g_direction.East);
    } else {
      i257 = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i254) != i257) {
      if (g_direction.South <= 7) {
        i258 = (unsigned char)(1 << g_direction.South);
      } else {
        i258 = 0;
      }

      if (g_direction.South <= 7) {
        i262 = (unsigned char)(1 << g_direction.South);
      } else {
        i262 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i258) != i262)
      {
        /* 南のマスの東が探索済み　かつ　壁がないとき */
        qY = current_y - 1U;
        if (qY > current_y) {
          qY = 0U;
        }

        if (g_direction.East <= 7) {
          i268 = (unsigned char)(1 << g_direction.East);
        } else {
          i268 = 0;
        }

        if ((maze_wall_search[((int)qY + i227) - 1] & i268) != 0) {
          qY = current_y - 1U;
          if (qY > current_y) {
            qY = 0U;
          }

          if (g_direction.East <= 7) {
            i275 = (unsigned char)(1 << g_direction.East);
          } else {
            i275 = 0;
          }

          if (g_direction.East <= 7) {
            i279 = (unsigned char)(1 << g_direction.East);
          } else {
            i279 = 0;
          }

          if ((maze_wall[((int)qY + i227) - 1] & i275) != i279) {
            /* 東のマスの南の壁が確定、探索済みとする。 */
            i225 = (int)(current_x + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.South <= 7) {
              i289 = (unsigned char)(1 << g_direction.South);
            } else {
              i289 = 0;
            }

            maze_wall[(current_y + ((i225 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + ((ex - 1) << 5)) - 1] | i289);
            i225 = (int)(current_x + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
            }

            ex = (int)(current_x + 1U);
            if ((unsigned int)ex > 255U) {
              ex = 255;
            }

            if (g_direction.South <= 7) {
              i297 = (unsigned char)(1 << g_direction.South);
            } else {
              i297 = 0;
            }

            maze_wall_search[(current_y + ((i225 - 1) << 5)) - 1] = (unsigned
              char)(maze_wall_search[(current_y + ((ex - 1) << 5)) - 1] | i297);

            /* 隣り合うマス（南東マス）の北の壁も確定 */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i225 = (int)(current_x + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall[((int)qY + ((i225 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)b_qY + ((ex - 1) << 5)) - 1] | i306);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i225 = (int)(current_x + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall_search[((int)qY + ((i225 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[((int)b_qY + ((ex - 1) << 5)) - 1] | i313);
          }
        }

        /* 東のマスの南が探索済み　かつ　壁がないとき */
        i225 = (int)(current_x + 1U);
        if ((unsigned int)i225 > 255U) {
          i225 = 255;
        }

        if (g_direction.South <= 7) {
          i278 = (unsigned char)(1 << g_direction.South);
        } else {
          i278 = 0;
        }

        if ((maze_wall_search[(current_y + ((i225 - 1) << 5)) - 1] & i278) != 0)
        {
          i225 = (int)(current_x + 1U);
          if ((unsigned int)i225 > 255U) {
            i225 = 255;
          }

          if (g_direction.South <= 7) {
            i285 = (unsigned char)(1 << g_direction.South);
          } else {
            i285 = 0;
          }

          if (g_direction.South <= 7) {
            i288 = (unsigned char)(1 << g_direction.South);
          } else {
            i288 = 0;
          }

          if ((maze_wall[(current_y + ((i225 - 1) << 5)) - 1] & i285) != i288) {
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
              i296 = (unsigned char)(1 << g_direction.East);
            } else {
              i296 = 0;
            }

            maze_wall[((int)qY + i227) - 1] = (unsigned char)(maze_wall[((int)
              b_qY + i227) - 1] | i296);
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

            maze_wall_search[((int)qY + i227) - 1] = (unsigned char)
              (maze_wall_search[((int)b_qY + i227) - 1] | i303);

            /* 隣り合うマス（南東マス）の西の壁が確定。探索済みとする */
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i225 = (int)(current_x + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall[((int)qY + ((i225 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[((int)b_qY + ((ex - 1) << 5)) - 1] | i312);
            qY = current_y - 1U;
            if (qY > current_y) {
              qY = 0U;
            }

            i225 = (int)(current_x + 1U);
            if ((unsigned int)i225 > 255U) {
              i225 = 255;
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

            maze_wall_search[((int)qY + ((i225 - 1) << 5)) - 1] = (unsigned char)
              (maze_wall_search[((int)b_qY + ((ex - 1) << 5)) - 1] | i317);
          }
        }
      }
    }

    /* 南,西に壁がない場合 */
    if (g_direction.West <= 7) {
      i260 = (unsigned char)(1 << g_direction.West);
    } else {
      i260 = 0;
    }

    if (g_direction.West <= 7) {
      i265 = (unsigned char)(1 << g_direction.West);
    } else {
      i265 = 0;
    }

    if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i260) != i265) {
      if (g_direction.South <= 7) {
        i266 = (unsigned char)(1 << g_direction.South);
      } else {
        i266 = 0;
      }

      if (g_direction.South <= 7) {
        i271 = (unsigned char)(1 << g_direction.South);
      } else {
        i271 = 0;
      }

      if ((maze_wall[(current_y + ((current_x - 1) << 5)) - 1] & i266) != i271)
      {
        /* 南のマスの西が探索済み　かつ　壁がないとき */
        qY = current_y - 1U;
        if (qY > current_y) {
          qY = 0U;
        }

        if (g_direction.West <= 7) {
          i277 = (unsigned char)(1 << g_direction.West);
        } else {
          i277 = 0;
        }

        if ((maze_wall_search[((int)qY + i227) - 1] & i277) != 0) {
          qY = current_y - 1U;
          if (qY > current_y) {
            qY = 0U;
          }

          if (g_direction.West <= 7) {
            i284 = (unsigned char)(1 << g_direction.West);
          } else {
            i284 = 0;
          }

          if (g_direction.West <= 7) {
            i287 = (unsigned char)(1 << g_direction.West);
          } else {
            i287 = 0;
          }

          if ((maze_wall[((int)qY + i227) - 1] & i284) != i287) {
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
              i295 = (unsigned char)(1 << g_direction.South);
            } else {
              i295 = 0;
            }

            maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] = (unsigned char)
              (maze_wall[(current_y + (((int)b_qY - 1) << 5)) - 1] | i295);
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
          i286 = (unsigned char)(1 << g_direction.South);
        } else {
          i286 = 0;
        }

        if ((maze_wall_search[(current_y + (((int)qY - 1) << 5)) - 1] & i286) !=
            0) {
          qY = current_x - 1U;
          if (qY > current_x) {
            qY = 0U;
          }

          if (g_direction.South <= 7) {
            i293 = (unsigned char)(1 << g_direction.South);
          } else {
            i293 = 0;
          }

          if (g_direction.South <= 7) {
            i294 = (unsigned char)(1 << g_direction.South);
          } else {
            i294 = 0;
          }

          if ((maze_wall[(current_y + (((int)qY - 1) << 5)) - 1] & i293) != i294)
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

            maze_wall[((int)qY + i227) - 1] = (unsigned char)(maze_wall[((int)
              b_qY + i227) - 1] | i300);
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

            maze_wall_search[((int)qY + i227) - 1] = (unsigned char)
              (maze_wall_search[((int)b_qY + i227) - 1] | i308);

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
  coder_internal_ref_1 b_maze_goal;
  coder_internal_ref wall_flg;
  unsigned char new_goal[2];
  coder_internal_ref_5 wall;
  coder_internal_ref_4 search;
  coder_internal_ref_3 adachi_search_mode;
  coder_internal_ref_2 max_length;
  unsigned short start_num;
  unsigned char current_x;
  unsigned char search_flag;
  unsigned char goal_dir;
  unsigned char current_y;
  unsigned char goal_section[2];
  unsigned char goal_node2[2];
  unsigned char start_flg;
  unsigned char current_dir;
  unsigned short unusedExpr[1024];
  int exitg1;
  bool exitg2;
  unsigned short b_unusedExpr[1024];
  unsigned char unexp_square[1024];
  unsigned short c_unusedExpr[1024];
  unsigned short d_unusedExpr[1024];
  b_goal_size.contents = goal_size;
  for (N = 0; N < 18; N++) {
    b_maze_goal.contents[N] = maze_goal[N];
  }

  /* C言語関数インクルード */
  /* ローカル変数定義 */
  /*  ゴール時ストップフラグ(0:移動継続　1:ストップ) */
  /*  スタートフラグ(0:動作中　1:停止からの移動開始) */
  /* ゴール直後フラグ(0:ゴール直後でない, 1:ゴール直後) */
  wall_flg.contents = 0U;

  /* 壁フラグ(1:前、2:右、（4:後ろ)、8:左) */
  /*  グローバル変数宣言 */
  /* メインfigure */
  /* メインaxes */
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
    current_x = 1U;
    current_y = 1U;
    move_step(&current_x, &current_y, g_direction.North);

    /* C言語でのスタート処理 */
    m_start_movement(1, 0, move_dir_property.straight);

    /* 停止直後フラグをクリア */
    /* ゴールをプロット */
    /* 足立法による探索 */
    current_dir = g_direction.North;
    search_flag = 0U;
    search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &adachi_search_mode,
                  &current_x, &current_y, &current_dir, maze_row_size,
                  maze_col_size, maze_wall, maze_wall_search, maze_goal,
                  goal_size, &search_flag, 0U, unusedExpr);

    /* ひとまづゴール(停止) */
    /* 各フラグを定義 */
    start_flg = 1U;

    /* 停止直後 */
    /* 停止処理を実施しない */
    /* ゴール直後フラグをたてる */
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
        b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal,
                        &adachi_search_mode, &current_x, &current_y,
                        &current_dir, maze_row_size, maze_col_size, maze_wall,
                        maze_wall_search, new_goal, &start_flg, 0U, 1U, 1U,
                        b_unusedExpr);

        /* ゴール直後フラグをたてる */
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* 帰路探索 */
    /* 全面探索 */
    /* 未探索マスがなくなるまで。 */
    /* 現地点から一番近い未探索マスを探索 */
    /* 現在地からコンターマップを展開、探索済みでないマスが見つかれば、そこをゴールとする。 */
    switch (run_mode_2) {
     case 1:
      do {
        exitg1 = 0;
        make_new_goal_all(&wall, maze_wall, maze_wall_search, current_x,
                          current_y, contour_map, new_goal);
        if (new_goal[0] == 0) {
          exitg1 = 1;
        } else {
          /* ゴールをプロット */
          /* 既存のコンターを使用し、探索。 */
          b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal,
                          &adachi_search_mode, &current_x, &current_y,
                          &current_dir, maze_row_size, maze_col_size, maze_wall,
                          maze_wall_search, new_goal, &start_flg, 0U, 1U, 1U,
                          c_unusedExpr);

          /* ゴール直後フラグをたてる */
        }
      } while (exitg1 == 0);

      /* 新規ゴールが見つからないとき、停止処理を実施 */
      m_goal_movement(start_flg, wall_flg.contents, move_dir_property.straight);

      /* 探索終了時、停止させているため、フラグをたてる。 */
      start_flg = 1U;

      /* 停止直後 */
      /* 最短経路探索 */
      /* 最短となりうるマスの未探索のみ探索 */
      break;

     case 2:
      do {
        exitg1 = 0;

        /* 未探索壁はないものとして、ゴールからスタートまで、コンターマップを展開（直線の重みあり） */
        make_map_fustrun(&b_goal_size, &wall, &search, b_maze_goal.contents,
                         maze_wall, maze_wall_search, 0U, contour_map);

        /* マップをもとに、最短経路を導出。経路上の未探索マスとその数を出力 */
        fust_run(&b_goal_size, &wall, maze_wall, maze_wall_search, contour_map,
                 b_maze_goal.contents, MAX_uint16_T, unexp_square, &search_flag);

        /* 未探索マスがなければ、ブレイク */
        if (search_flag == 0) {
          exitg1 = 1;
        } else {
          /* 未探索マスがある場合、探索する。 */
          /* 現在地点からコンターを展開し、該当の未探索が更新されれば、そこを新規ゴールとして出力 */
          make_new_goal_sh(&wall, maze_wall, current_x, current_y, unexp_square,
                           search_flag, contour_map, new_goal);

          /* ゴールをプロット */
          /* 新規ゴールに向け、探索 */
          b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal,
                          &adachi_search_mode, &current_x, &current_y,
                          &current_dir, maze_row_size, maze_col_size, maze_wall,
                          maze_wall_search, new_goal, &start_flg, 0U, 1U, 1U,
                          d_unusedExpr);

          /* ゴール直後フラグをたてる */
        }
      } while (exitg1 == 0);

      /* 新規ゴールが見つからないとき、停止処理を実施 */
      m_goal_movement(start_flg, wall_flg.contents, move_dir_property.straight);

      /* 探索終了時、停止させているため、停止フラグを立てる。 */
      start_flg = 1U;

      /* 停止直後 */
      /* その他の場合無視 */
      break;

     default:
      /* 停止させない */
      start_flg = 0U;

      /* 停止直後でない */
      break;
    }

    /* スタートを目的地として足立法で再探索 */
    /* 各フラグを定義 */
    /* 停止処理を実施する */
    /* ゴール直後フラグをたてる */
    /* スタートをゴールに設定 */
    /* ゴールをプロット */
    /* 足立法で帰宅 */
    goal_section[0] = 1U;
    goal_section[1] = 1U;
    b_search_adachi(&wall, &wall_flg, &search, &b_maze_goal, &adachi_search_mode,
                    &current_x, &current_y, &current_dir, maze_row_size,
                    maze_col_size, maze_wall, maze_wall_search, goal_section,
                    &start_flg, 1U, 1U, 0U, contour_map);

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
        current_x = 1U;
        current_y = 1U;
        move_step(&current_x, &current_y, g_d_direction.North);

        /* C言語でのスタート処理 */
        m_start_movement(1, 0, move_dir_property.straight);

        /* 停止直後フラグをクリア */
        /* 最短距離走行 */
        b_fust_run(&b_goal_size, &wall_flg, &wall, maze_wall, contour_map,
                   maze_goal, MAX_uint16_T, current_x, current_y);

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
                              goal_section, goal_node2, &start_flg);

          /* 確定されたゴールマスから、再度マップを生成 */
          new_goal[0] = goal_section[1];
          new_goal[1] = goal_section[0];

          /* x,yに変換 */
          b_make_map_fustrun_diagonal(&max_length, &wall, &search, new_goal,
            maze_wall, maze_wall_search, row_num_node, col_num_node, &start_num);

          /* 生成されたMAPをもとに最短走行 */
          make_route_diagonal(row_num_node, col_num_node, new_goal, goal_node2,
                              start_flg);
        }
      }
    }
  }
}

/*
 * File trailer for maze_solve.c
 *
 * [EOF]
 */
