#ifndef MATLAB_MOVEMENT
#define MATLAB_MOVEMENT

//前進
void m_move_front(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

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

#endif