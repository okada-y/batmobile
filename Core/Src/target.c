/*
 * target.c
 *
 *  Created on: Aug 7, 2019
 *      Author: 岡田 泰裕
 */

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



//機能	: 各ターン用の動作パラメータ(速度、加速度、角速度、角加速度)＋移動距離、終端速度をセットする
//引数	: ターン番号,目標移動距離,終端速度
//返り値	: なし
void set_target_turn_param ( turn_lib turnlibnum,float t_length, float t_m_speed_fin )
{
	switch(turnlibnum){
	case search:
		set_target_move_speed_max(search_move_speed_max);
		set_target_move_accel(search_move_accel);
		set_target_rotate_speed_max(search_rotate_speed_max);
		set_target_rotate_accel(search_rotate_accel);
		set_target_length(t_length);
		set_target_move_speed_fin(t_m_speed_fin);
		break;

		//その他ターン作成時、追記する。
	case straight:
		set_target_move_speed_max(straight_move_speed_max);
		set_target_move_accel(straight_move_accel);
		set_target_rotate_speed_max(straight_rotate_speed_max);
		set_target_rotate_accel(straight_rotate_accel);
		set_target_length(t_length);
		set_target_move_speed_fin(t_m_speed_fin);
		break;


	default:
		set_target_move_speed_max(0);
		set_target_move_accel(0);
		set_target_rotate_speed_max(0);
		set_target_rotate_accel(0);
		set_target_length(0);
		set_target_move_speed_fin(0);
		break;
	}
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
	float tm_target_delta_v =0;

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


