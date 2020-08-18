#ifndef TARGET_H_
#define TARGET_H_

//ターンライブラリ
typedef enum {
	search	= 0,	//探索
	straight  = 1,	//既知区間加速
	turn_45 =2,
	turn_90 =3,
	turn_V90 =4,
	turn_135 =5,
	turn_180 =6,
} turn_lib;


////移動方向モード
//typedef enum {
//	forward_mode 	= 0,	//前進
//	backward_mode 	= 1,	//後進
//} direction_mode;

//回転方向モード
typedef enum {
	counter_clockwise 	= 0,	//反時計回り
	clockwise 	= 1,			//時計回り
} rotation_mode;

////加速モード
//typedef enum {
//	deceleration	= 0, //減速
//	acceleration	= 1, //加速
//} accel_mode;

////下限速度モード
//typedef enum {
//	zero 	= 0, 	//停止
//	slow 	= 1, 	//スロー
//} speed_under_lim_mode;

void target_1ms ( void );
//void set_direction_mode ( direction_mode );
void set_rotation_mode ( rotation_mode );
//void set_accel_mode ( accel_mode );
//void set_speed_under_lim_flg ( speed_under_lim_mode );
float get_target_length ( void );
float get_ideal_length ( void );
float get_target_angle ( void );
float get_ideal_angle ( void );
void set_ideal_length ( float );
void set_ideal_angle ( float );
float get_target_move_speed ( void );
float get_target_rotation_speed ( void );
float get_target_move_accel ( void );
float get_target_rotation_accel ( void );

void set_target_length ( float );
void set_target_angle ( float );
void calc_target_move_speed(void);
void calc_target_rotation_speed ( void );
void calc_ideal_length(void);
void calc_ideal_angle(void);
void set_target_move_speed_max ( float t_movespeed );
void set_target_move_speed_fin ( float t_movespeed_fin );
void set_target_move_accel ( float t_moveaccel );
void set_target_rotate_speed_max ( float t_rotatespeed );
void set_target_rotate_accel ( float t_rotateaccel );
void set_target_turn_param ( turn_lib turnlibnum,float t_length, float t_m_speed_fin );






#endif /* TARGET_H_*/
