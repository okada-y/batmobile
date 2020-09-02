#ifndef MATLAB_MOVEMENT
#define MATLAB_MOVEMENT

//�O�i
void m_move_front(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//�E��
void m_move_right(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//����
void m_move_left(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//�o�b�N
void m_move_back(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//�X�^�[�g������
void m_start_movement(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//�S�[�����̓���i��~�����j
void m_goal_movement(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

//前進_ロング
void m_move_front_long(unsigned char straight_count,unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
#endif
