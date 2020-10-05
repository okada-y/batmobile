/*
 * target.c
 *
 *  Created on: Aug 7, 2019
 *      Author: 岡田 泰裕
 */

#include "index.h"
#include "param.h"
#include "ir_sensor.h"
#include "target.h"
#include "mouse_state.h"


static float target_move_speed = 0;			//目標移動速度[m/s]
static float target_move_speed_fin = 0;		//目標終端速度[m/s]
static float target_rotation_speed = 0;		//目標角速度[rad/s]
static float target_move_accel = 0;			//目標移動加速度[m/s2]
static float target_rotation_accel = 0;		//目標角加速度[rad/s2]
static float target_length = 0;				//目標移動距離[m]
static float target_angle = 0;				//目標回転角度[rad]
static float ideal_length = 0;				//理想移動距離[m]
static float ideal_angle = 0;				//理想回転角度[rad]

static float target_move_speed_max = 0;//目標移動速度[m/s]
static float target_rotation_speed_max = 0;//目標移動速度[rad/s]

static rotation_mode rotation_dir_flg = counter_clockwise;	//回転方向フラグ　0:反時計周り 1:時計周り
static speed_under_lim_mode speed_under_lim_flg = zero;		//下限速度フラグ

static t_param_set param_set;

//機能	: traget.cの1msタスクのまとめ
//引数	: なし
//返り値	: なし
void target_1ms ( void )
{
	calc_target_move_speed();			//目標移動速度更新
	calc_ideal_length();							//理想移動距離更新
	calc_target_rotation_speed();		//目標角速度更新
	calc_ideal_angle();							//理想角度更新
}


//機能	: 最大目標移動速度をセットする
//引数	: 目標最大移動速度
//返り値	: なし
void set_target_move_speed_max ( float t_movespeed )
{
	target_move_speed_max = t_movespeed;
}

//機能	: 最終目標移動速度をセットする
//引数	: 最終目標移動速度
//返り値	: なし
void set_target_move_speed_fin ( float t_movespeed_fin )
{
	target_move_speed_fin = t_movespeed_fin;
}

//機能	: 目標加速度をセットする
//引数	: 目標移動加速度
//返り値	: なし
void set_target_move_accel ( float t_moveaccel )
{
	target_move_accel = t_moveaccel;
}

//機能	: 最大目標回転速度をセットする
//引数	: 移動速度目標
//返り値	: なし
void set_target_rotate_speed_max ( float t_rotatespeed )
{
	target_rotation_speed_max= t_rotatespeed;
}

//機能	: 目標加速度をセットする
//引数	: 目標移動加速度
//返り値	: なし
void set_target_rotate_accel ( float t_rotateaccel )
{
	target_rotation_accel = t_rotateaccel;
}


//機能	: ランモード３に応じた走行パラメータの初期化
//引数	: なし（ランモード３）はグローバル
//返り値	: なし
void init_target_turn_param(void)
{
	switch (run_mode_3){
	case 0:
		//並進パラメータ
		param_set.search.move_v = search_move_speed_max;
		param_set.search.move_a = search_move_accel;
		param_set.straight.move_v = straight_move_speed_max;
		param_set.straight.move_a = straight_move_accel;
		param_set.diagonal.move_v = straight_move_speed_max;
		param_set.diagonal.move_a = straight_move_accel;

		//ターンパラメータ
		//探索90
		param_set.turn_s90.move_v = slalom_s90_v_1;
		param_set.turn_s90.turn_w = slalom_s90_w_1;
		param_set.turn_s90.turn_wa = slalom_s90_wa_1;
		param_set.turn_s90.offset_befor = slalom_s90_bo_1;
		param_set.turn_s90.offset_after = slalom_s90_ao_1;

		//斜め45
		param_set.turn_45.move_v = slalom_45_v_1;
		param_set.turn_45.turn_w = slalom_45_w_1;
		param_set.turn_45.turn_wa = slalom_45_wa_1;
		param_set.turn_45.offset_befor = slalom_45_bo_1;
		param_set.turn_45.offset_after = slalom_45_ao_1;

		//大廻90
		param_set.turn_90.move_v = slalom_90_v_1;
		param_set.turn_90.turn_w = slalom_90_w_1;
		param_set.turn_90.turn_wa = slalom_90_wa_1;
		param_set.turn_90.offset_befor = slalom_90_bo_1;
		param_set.turn_90.offset_after = slalom_90_ao_1;

		//V90
		param_set.turn_V90.move_v = slalom_v90_v_1;
		param_set.turn_V90.turn_w = slalom_v90_w_1;
		param_set.turn_V90.turn_wa = slalom_v90_wa_1;
		param_set.turn_V90.offset_befor = slalom_v90_bo_1;
		param_set.turn_V90.offset_after = slalom_v90_ao_1;

		//135
		param_set.turn_135.move_v = slalom_135_v_1;
		param_set.turn_135.turn_w = slalom_135_w_1;
		param_set.turn_135.turn_wa = slalom_135_wa_1;
		param_set.turn_135.offset_befor = slalom_135_bo_1;
		param_set.turn_135.offset_after = slalom_135_ao_1;

		//180
		param_set.turn_180.move_v = slalom_180_v_1;
		param_set.turn_180.turn_w = slalom_180_w_1;
		param_set.turn_180.turn_wa = slalom_180_wa_1;
		param_set.turn_180.offset_befor = slalom_180_bo_1;
		param_set.turn_180.offset_after = slalom_180_ao_1;
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

//機能	: 各ターン用の動作パラメータ(角速度、角加速度)をセットする(角度を除く)
//引数	: パターン番号
//返り値	: なし
void set_target_turn_param ( e_movement_pattern_No pattern_No )
{

	switch(pattern_No){
	case turn_s90://探索90
		set_target_rotate_speed_max(param_set.turn_s90.turn_w);
		set_target_rotate_accel(param_set.turn_s90.turn_wa);
		//set_target_angle(rotate_dir * PI/2);
		break;
	case turn_45://45
		set_target_rotate_speed_max(param_set.turn_45.turn_w);
		set_target_rotate_accel(param_set.turn_45.turn_wa);
		//set_target_angle(rotate_dir * PI/4);
		break;
	case turn_90://大廻90
		set_target_rotate_speed_max(param_set.turn_90.turn_w);
		set_target_rotate_accel(param_set.turn_90.turn_wa);
		//set_target_angle(rotate_dir * PI/2);
		break;
	case turn_V90://V90
		set_target_rotate_speed_max(param_set.turn_V90.turn_w);
		set_target_rotate_accel(param_set.turn_V90.turn_wa);
		//set_target_angle(rotate_dir * PI/2);
		break;
	case turn_135://135
		set_target_rotate_speed_max(param_set.turn_135.turn_w);
		set_target_rotate_accel(param_set.turn_135.turn_wa);
		//set_target_angle(rotate_dir * PI*3/4);
		break;
	case turn_180://180
		set_target_rotate_speed_max(param_set.turn_180.turn_w);
		set_target_rotate_accel(param_set.turn_180.turn_wa);
		//set_target_angle(rotate_dir * PI);
		break;
	default :
		set_target_rotate_speed_max(0);
		set_target_rotate_accel(0);
		set_target_angle(0);
		break;
	}
}

//機能	: 各ターン用の動作パラメータ(速度、加速度)＋移動距離、終端速度をセットする
//引数	: ターン番号,目標移動距離,終端速度
//返り値	: なし
void set_target_move_param (e_movement_pattern_No pattern_No,float t_length, float t_m_speed_fin )
{
	switch(pattern_No){
	case search://探索
		set_target_move_speed_max(param_set.search.move_v);
		set_target_move_accel(param_set.search.move_a);
		set_target_length(t_length);
		set_target_move_speed_fin(t_m_speed_fin);
		break;

		//その他ターン作成時、追記する。
	case straight://既知区間加速
		set_target_move_speed_max(param_set.straight.move_v);
		set_target_move_accel(param_set.straight.move_a);
		set_target_length(t_length);
		set_target_move_speed_fin(t_m_speed_fin);
		break;

	case diagonal:	//斜め直進
		set_target_move_speed_max(param_set.diagonal.move_v);
		set_target_move_accel(param_set.diagonal.move_a);
		set_target_length(t_length);
		set_target_move_speed_fin(t_m_speed_fin);
		break;

		//各種ターンは定速（＝ターン時に速度、加速度を更新しない。)


	default:
		set_target_move_speed_max(0);
		set_target_move_accel(0);
		set_target_length(0);
		set_target_move_speed_fin(0);
		break;
	}
}

////機能	: 各種ターンのオフセット距離を取得する
////引数	: 動作パターンNo, オフセット距離No
////返り値	: オフセット距離
float get_offset_length(e_movement_pattern_No pattern_num, e_offset_No dir)
{
	float temp;

	switch (pattern_num){
	case turn_s90://探索90
		if (dir == before_offset){
			temp = param_set.turn_s90.offset_befor;
		}else{
			temp = param_set.turn_s90.offset_after;
		}
		break;
	case turn_45://斜め45
		if (dir == before_offset){
			temp = param_set.turn_45.offset_befor;
		}else{
			temp = param_set.turn_45.offset_after;
		}
		break;

	case turn_90://大廻90
		if (dir == before_offset){
			temp = param_set.turn_90.offset_befor;
		}else{
			temp = param_set.turn_90.offset_after;
		}
		break;

	case turn_V90://V90
		if (dir == before_offset){
			temp = param_set.turn_V90.offset_befor;
		}else{
			temp = param_set.turn_V90.offset_after;
		}
		break;

	case turn_135://斜め135
		if (dir == before_offset){
			temp = param_set.turn_135.offset_befor;
		}else{
			temp = param_set.turn_135.offset_after;
		}
		break;

	case turn_180://大廻180
		if (dir == before_offset){
			temp = param_set.turn_180.offset_befor;
		}else{
			temp = param_set.turn_180.offset_after;
		}
		break;

	default:
		temp = 0;
		break;
	}



	return temp;
}


////機能	: 移動方向モードをセットする
////引数	: 移動方向モード(forward,back)
////返り値	: なし
//void set_direction_mode ( direction_mode dmode )
//{
//	move_dir_flg = dmode;
//}

//機能	: 回転方向モードをセットする
//引数	: 回転方向モード(counter_clocwise,clockwise)
//返り値	: なし
void set_rotation_mode ( rotation_mode rmode )
{
	rotation_dir_flg = rmode;
}

////機能	: 加速モードをセットする
////引数	: 加速モード(acceleration,deceleration)
////返り値	: なし
//void set_accel_mode ( accel_mode amode )
//{
//	accel_dir_flg = amode;
//}

//機能	: 下限速度モードをセットする
//引数	: 下限速度モード(zero,slow)
//返り値	: なし
void set_speed_under_lim_flg ( speed_under_lim_mode smode  )
{
	speed_under_lim_flg = smode;
}

//機能	: 目標移動速度[m/s]取得
//引数	: なし
//返り値	: 目標移動速度[m/s]
float get_target_move_speed ( void )
{
	return target_move_speed;
}

//機能	: 目標角速度[m/s]取得
//引数	: なし
//返り値	: 目標角速度[m/s]
float get_target_rotation_speed ( void )
{
	return target_rotation_speed;
}

//機能	: 目標移動加速度[m/s]取得
//引数	: なし
//返り値	: 目標移動加速度[m/s]
float get_target_move_accel ( void )
{
	return target_move_accel;
}

//機能	: 目標角加速度[m/s]取得
//引数	: なし
//返り値	: 目標角加速度[m/s]
float get_target_rotation_accel ( void )
{
	return target_rotation_accel;
}

//機能	: 目標移動距離取得
//引数	: なし
//返り値	: 目標移動距離
float get_target_length ( void )
{
	return target_length;
}

//機能	: 理想移動距離セット
//引数	: 理想移動距離
//返り値	: なし
void set_ideal_length ( float templ )
{
	ideal_length = templ;
}

//機能	: 理想角度セット
//引数	: 理想角度
//返り値	: なし
void set_ideal_angle ( float templ )
{
	ideal_angle = templ;
}

//機能	: 理想移動距離取得
//引数	: なし
//返り値	: 理想移動距離
float get_ideal_length ( void )
{
	return ideal_length;
}

//機能	: 目標角度取得
//引数	: なし
//返り値	: 目標角度
float get_target_angle ( void )
{
	return target_angle;
}

//機能	: 理想角度取得
//引数	: なし 
//返り値	: 理想角度
float get_ideal_angle ( void )
{
	return ideal_angle;
}



//機能	: 目標移動距離更新
//引数	: 加算する移動距離[m]
//返り値	: なし
void set_target_length ( float set_length )
{
	target_length = set_length;
}

//機能	: 目標角度更新
//引数	: 加算する角度[rad]
//返り値	: なし
void set_target_angle ( float set_angle )
{
	target_angle = set_angle;
}


//機能	: 移動距離、終端速度から加速度をスイッチ、速度を算出。
//引数	: なし
//返り値	: なし
//備考	: 現在移動距離と目標移動距離に応じて、加速度をスイッチ
//		: 1msタスク
void calc_target_move_speed(void)
{

	float tm_deccel_length = 0;

//	tm_target_delta_v = target_move_speed - target_move_speed_fin;

		/* 加減速に必要な距離算出*/
		tm_deccel_length = ( (target_move_speed * target_move_speed)  - (target_move_speed_fin*target_move_speed_fin))/ (2 * target_move_accel);

		//目標距離が加減速距離外のとき
		if(ideal_length  < (target_length - tm_deccel_length))
		{
			//加速処理
			target_move_speed += target_move_accel * 0.001; //1ms分の加速加算
			//リミット処理(設定最大速度で頭打ち)
			if (target_move_speed > target_move_speed_max)
			{
				target_move_speed = target_move_speed_max;
			}
		}

		//目標距離が加減速距離内であるとき
		else
		{
			//現在の目標速度を終端速度と比較し、加減速を決定
			if(target_move_speed < target_move_speed_fin){

				//加速処理
				target_move_speed += target_move_accel * 0.001; //1ms分の加速加算
				//リミット処理(設定最大速度で頭打ち)
				if (target_move_speed > target_move_speed_max)
				{
					target_move_speed = target_move_speed_max;
				}

			}
			else{
				//減速処理
				target_move_speed -= target_move_accel * 0.001; //1ms分の減速加算
				//下限処理
				switch(speed_under_lim_flg)
				{
					case zero :
						if (target_move_speed < target_move_speed_fin){
							target_move_speed = target_move_speed_fin;
							}
						break;
					case slow :
						if (target_move_speed < move_speed_slow){
							target_move_speed = move_speed_slow;
							}
						break;
				}
			}
		}
}

//機能	: 理想移動距離の更新
//引数	: なし
//返り値	: なし
//備考	:  1msタスク
void calc_ideal_length(void)
{
	//理想移動距離算出
	ideal_length += target_move_speed * 0.001;
}


//機能	: 角加速度の更新
//引数	: なし
//返り値	: なし
//備考	: 現在角度と目標角度に応じて、加速度をスイッチ
//		: 1msタスク
void calc_target_rotation_speed(void)
{

	float tm_deccel_angle = 0;

	switch(rotation_dir_flg)
	{
	case counter_clockwise:
		/*減速に必要な角度算出*/
		tm_deccel_angle = 0.5 * target_rotation_speed * target_rotation_speed / target_rotation_accel;
		
		//減速開始前
		if(ideal_angle < (target_angle - tm_deccel_angle))
		{			
			//目標角速度更新
			target_rotation_speed += target_rotation_accel * 0.001;

			//上限処理
			if (target_rotation_speed > target_rotation_speed_max)
			{
				target_rotation_speed = target_rotation_speed_max;
			}
		}
		else//減速開始後
		{
			//目標角速度更新
			target_rotation_speed -= target_rotation_accel * 0.001;

			//下限処理
			if (target_rotation_speed < 0)
			{
				target_rotation_speed = 0;
			}
		}
		break;

	case clockwise:
		/*加速に必要な角度算出*/
		tm_deccel_angle = 0.5 * target_rotation_speed * target_rotation_speed / target_rotation_accel;
		
		//加速開始前
		if(ideal_angle > (target_angle + tm_deccel_angle))
		{
			//目標角速度更新
			target_rotation_speed -= target_rotation_accel * 0.001;

			//下限処理
			if (target_rotation_speed < -1*target_rotation_speed_max)
			{
				target_rotation_speed = -1*target_rotation_speed_max;
			}

		}
		else//加速開始後
		{
			//目標角速度更新
			target_rotation_speed += target_rotation_accel * 0.001;

			//上限処理
			if (target_rotation_speed > 0)
			{
				target_rotation_speed = 0;
			}

		}
		break;
	}

}


//機能	: 理想角度を算出
//引数	: なし:
//返り値	: なし
//備考	:1msタスク
void calc_ideal_angle ( void )
{
	ideal_angle += target_rotation_speed * 0.001;
}


