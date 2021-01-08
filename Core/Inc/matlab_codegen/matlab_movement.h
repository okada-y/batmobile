#ifndef MATLAB_MOVEMENT
#define MATLAB_MOVEMENT

//前進
void m_move_front(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//前進_ロング
void m_move_front_long(unsigned char straight_count,unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//右折
void m_move_right(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//左折
void m_move_left(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//バック
void m_move_back(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//スタート時動作
void m_start_movement(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//ゴール時の動作（停止処理）
void m_goal_movement(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);


//エラー時の動作
void m_error_movement(unsigned char error_flg);

//右V90度ターン
void m_turn_V90_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//左V90度ターン
void m_turn_V90_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//右90度ターン
void m_turn_90_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//左90度ターン
void m_turn_90_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//右180度ターン
void m_turn_180_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//左180度ターン
void m_turn_180_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//右135度ターン
void m_turn_135_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//左135度ターン
void m_turn_135_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//右45度ターン
void m_turn_45_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
//左45度ターン
void m_turn_45_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

#endif