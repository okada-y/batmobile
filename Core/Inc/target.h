#ifndef TARGET_H_
#define TARGET_H_

//動作パターン番号
typedef enum {
	search,	    //探索直進
	straight,	//既知区間加速
	diagonal,	//斜め直進
	turn_s90,//探索90
	turn_45, //45
	turn_90,//大廻90
	turn_V90,//V90
	turn_135,//135
	turn_180,//180
} e_movement_pattern_No;

//オフセット距離番号
typedef enum {
	before_offset,	    //前距離
	after_offset,	//後距離
} e_offset_No;

//ターン走行パラメータ
typedef struct {
	float offset_befor;
	float offset_after;
	float move_v;
	float turn_w;
	float turn_wa;
} t_turn_param_set;

//並進走行パラメータ
typedef struct {
	float move_v;
	float move_a;
} t_move_param_set;

//走行パラメータセット
typedef struct {

	//並進パラメータセット
	t_move_param_set search;
	t_move_param_set straight;
	t_move_param_set diagonal;

	//ターンパラメータセット
	t_turn_param_set turn_s90;
	t_turn_param_set turn_45;
	t_turn_param_set turn_90;
	t_turn_param_set turn_V90;
	t_turn_param_set turn_135;
	t_turn_param_set turn_180;

} t_param_set;



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

//下限速度モード
typedef enum {
	zero 	= 0, 	//停止
	slow 	= 1, 	//スロー
} speed_under_lim_mode;



void target_1ms ( void );
//void set_direction_mode ( direction_mode );
void set_rotation_mode ( rotation_mode );
//void set_accel_mode ( accel_mode );
void set_speed_under_lim_flg ( speed_under_lim_mode );
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
void set_target_move_param (e_movement_pattern_No pattern_No,float t_length, float t_m_speed_fin );
void set_target_turn_param ( e_movement_pattern_No pattern_No);
float get_offset_length(e_movement_pattern_No pattern_num, e_offset_No dir);
void init_target_turn_param(void);




#endif /* TARGET_H_*/
