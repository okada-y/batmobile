/*
 * mouse_state.c
 *
 *  Created on: Feb 1, 2020
 *      Author: 岡田 泰裕
 */

#include <stdio.h>
#include "param.h"
#include "encoder.h"
#include "imu.h"
#include "mouse_state.h"

static float tire_r_speed = 0;
static float tire_l_speed = 0;
static float tire_r_speed_max = 0;
static float tire_r_speed_min = 0;
static float tire_l_speed_max = 0;
static float tire_l_speed_min = 0; 
static float move_speed = 0;
static float move_speed_ave = 0;
static float move_accel_ave = 0;
static float move_length = 0;
static float rotation_speed = 0;
static float rotation_angle = 0;
static float est_x = 0;
static float est_v = 0;
static uint16_t right_count = 0;
static uint16_t left_count = 0;

//機能	: mouse_stateの1msタスクまとめ
//引数	: なし
//返り値	: なし
void mouse_state_1ms ( void )
{
//	calc_move_speed();		//速度計算 //速度計算のみ別枠にて処理
	IMU_Receive();				//IMUからヨー方向角速度、ｘ方向加速度を取得
	calc_rotation_speed();	//角速度計算
	filter_move_speed();	//速度をフィルタ処理
// speed_estimate_kalman();//速度をカルマンフィルタにて推定
	calc_move_length();		//移動距離を計算
	calc_rotation_angle();	//回転角度を計算
}

//機能	: 右タイヤのエンコーダのカウント差を取得する
//引数	: なし
//返り値	: 右タイヤのエンコーダのカウント差
int16_t Get_diff_right_count( void )
{
	uint16_t right_count_correct = 0;
	static uint16_t right_count_old = 0;
    int16_t delta_right_count = 0;
    static uint8_t init_flg = 0;

    //初回は初期位置を取得し、記録する。
    if (init_flg == 0){
    	right_count_old = encoder_Getangle_r();
    	right_count_old = encoder_correct_angle_r(right_count_old);
    	init_flg= 1;
    }

    right_count =  encoder_Getangle_r();
    right_count_correct = encoder_correct_angle_r(right_count);

    delta_right_count = (int16_t)( right_count_correct - right_count_old);

    right_count_old = right_count_correct;//カウンタ更新
    return delta_right_count;
}

//機能	: 左タイヤのエンコーダのカウント差を取得する
//引数	: なし
//返り値	: 左タイヤのエンコーダのカウント差
int16_t Get_diff_left_count( void )
{
	uint16_t left_count_correct = 0;
	static uint16_t left_count_old = 0;
    int16_t delta_left_count = 0;
    static uint8_t init_flg = 0;

    //初回は初期位置を取得し、記録する。
    if (init_flg == 0){
    	left_count_old = encoder_Getangle_l();
    	left_count_old = encoder_correct_angle_l(left_count_old);
    	init_flg= 1;
    }

    left_count =  encoder_Getangle_l();
    left_count_correct = encoder_correct_angle_l(left_count);

    delta_left_count = (int16_t)( left_count_correct - left_count_old);

    left_count_old = left_count_correct;//カウンタ更新
    return delta_left_count;
}

//機能	: エンコーダのカウント差から、マウスの速度を算出する
//引数	: なし
//返り値	: タイヤの速度
//備考  : 1msタスク
void calc_move_speed ( void )
{

    float delta_angle_tire_r = 0;
    float delta_angle_tire_l = 0;
   
	/* 各タイヤの角速度計算[rad/s] */
	delta_angle_tire_r = 1 * 1000 * 2 * PI * (float)Get_diff_right_count() / (float)ENC_RESOLUTION;
	delta_angle_tire_l =  -1 * 1000 * 2 * PI * (float)Get_diff_left_count() / (float)ENC_RESOLUTION;

	/*各タイヤの速度計算[m/s]*/
	tire_r_speed = Tire_diameter * 0.5 * delta_angle_tire_r;
	tire_l_speed = Tire_diameter * 0.5 * delta_angle_tire_l;

    /*各タイヤの最大、最小速度更新*/
    //右
    if(tire_r_speed > tire_r_speed_max)
	{   
		tire_r_speed_max = tire_r_speed;
	}
	else if(tire_r_speed < tire_r_speed_min)
	{
		tire_r_speed_min = tire_r_speed;
	}
    //左
	if(tire_l_speed > tire_l_speed_max)
	{
		tire_l_speed_max = tire_l_speed;
	}
	else if(tire_l_speed < tire_l_speed_min)
	{
		tire_l_speed_min = tire_l_speed;
	}

    /*マウスの速度計算[m/s]*/
    move_speed = (tire_r_speed + tire_l_speed)/2;

}

//機能	: 右タイヤのカウント値を取得する
//引数	: なし
//返り値	: 右タイヤのカウント値
uint16_t get_tire_r_angle ( void )
{
    return right_count;
}

//機能	: 右タイヤのカウント値を取得する
//引数	: なし
//返り値	: 右タイヤのカウント値
uint16_t get_tire_l_angle ( void )
{
    return left_count;
}


//機能	: 右タイヤの速度[m/s]を取得する
//引数	: なし
//返り値	: 右タイヤの移動速度[m/s]
float get_tire_r_speed ( void )
{
    return tire_r_speed;
}

//機能	: 右タイヤの最大速度[m/s]を取得する
//引数	: なし
//返り値	: 右タイヤの最大速度[m/s]
float get_tire_r_speed_max ( void )
{
    return tire_r_speed_max;
}

//機能	: 右タイヤの最低速度[m/s]を取得する
//引数	: なし
//返り値	: 右タイヤの最低速度[m/s]
float get_tire_r_speed_min ( void )
{
    return tire_r_speed_min;
}

//機能	: 左タイヤの速度[m/s]を取得する
//引数	: なし
//返り値	: 左タイヤの速度[m/s]
float get_tire_l_speed ( void )
{
    return tire_l_speed;
}

//機能	: 左タイヤの最大速度[m/s]を取得する
//引数	: なし
//返り値	: 左タイヤの最大速度[m/s]
float get_tire_l_speed_max ( void )
{
    return tire_l_speed_max;
}

//機能	: 左タイヤの最低速度[m/s]を取得する
//引数	: なし
//返り値	: 左タイヤの最低速度[m/s]
float get_tire_l_speed_min ( void )
{
    return tire_l_speed_min;
}

//機能	: 各タイヤの最大、最低速度を初期化する
//引数	: なし
//返り値	: なし
void init_tire_speed ( void )
{
    tire_r_speed_min = 0;
    tire_r_speed_max = 0;
    tire_l_speed_min = 0;
    tire_l_speed_max = 0;

}

//機能	: マウスの移動速度[m/s]を取得する
//引数	: なし
//返り値	: マウスの移動速度
float get_move_speed ( void )
{
    return move_speed;
}

//機能	: マウスの速度に移動平均フィルタをかける。
//引数	: なし
//返り値	: なし
//備考  : ジャイロの加速度を用いて、遅れを補完する。
//      : 1msタスク 
void filter_move_speed( void )
{
    typedef struct {
	float	speed;			
	float   accel;			
    } ave_temp;

    static ave_temp ave_store[ave_num];  // データ格納用の構造体

	static uint16_t ave_counter = 0;
	uint16_t i = 0;
	uint16_t j = 0;

	float speed_ave = 0;	//並進方向速度平均
	float accel_ave = 0;	//並進方向加速度平均

	float speed_sum = 0; 	//並進方向速度合計
	float accel_sum = 0;	//並進方向加速度合計

	i = ave_counter % ave_num;
	ave_store[i].speed = get_move_speed();
	ave_store[i].accel = IMU_GetAccel_X();

    /*移動平均処理*/
	for(j=0; j < ave_num; j++)
	{
		speed_sum += ave_store[j].speed;
		accel_sum += ave_store[j].accel;
	}

	speed_ave = speed_sum / ave_num;
	accel_ave = accel_sum / ave_num;

	/*加速度センサのぶれを補正、閾値範囲内を０とする*/
	if(ABS(accel_ave) < 0.5)
	{
		accel_ave = 0;
	}

	/*遅れ補正*/
    //平均速度＋0.5(加速度平均[m/s2]・平均時間[s])
	speed_ave += 0.5 * ( accel_ave * (ave_num * 0.001)); 

	move_speed_ave = speed_ave;
	move_accel_ave = accel_ave;

	ave_counter += 1 ;
}

//機能	: 移動平均処理後の移動速度[m/s]を取得する
//引数	: なし
//返り値	: 移動平均処理後の移動速度[m/s]
float get_move_speed_ave ( void )
{
    return move_speed_ave;
}

//機能	: 移動平均処理後の移動加速度[m/ss]を取得する
//引数	: なし
//返り値	: 移動平均処理後の移動速度[m/ss]
float get_move_accel_ave ( void )
{
    return move_accel_ave;
}
//機能	: マウスの移動距離[m]を計算する
//引数	: なし
//返り値	: なし
//備考  :1msタスク
void calc_move_length ( void )
{
    move_length += get_move_speed_ave() * 0.001; // [m/s] * [s]
}

//機能	: マウスの移動距離[m]を取得する
//引数	: なし
//返り値	: なし
float get_move_length ( void )
{
    return move_length;
}

//機能	: マウスの移動距離[m]を設定する
//引数	: なし
//返り値	: なし
void set_move_length ( float length )
{
    move_length = length;
}

//機能	: マウスの角速度[rad/s]を算出する
//引数	: なし
//返り値	: なし
//備考 :ローパスフィルタ
void calc_rotation_speed ( void )
{
	static float post_rotation_speed;		//前回角速度
	float tau = 1.0/(2.0*PI*cut_off_w);		//ローパスフィルタの時定数
	float den = tau + Sampling_cycle;

    rotation_speed = (Sampling_cycle/den) * IMU_GetGyro_Z()  
						 + (tau / den) * post_rotation_speed;

	post_rotation_speed = rotation_speed; //前回値更新
}

//機能	: マウスの角速度を取得する
//引数	: なし
//返り値	: マウスの角速度[rad/s]
float get_rotation_speed ( void )
{
    return rotation_speed;
}

//機能	: マウスの角度[rad/s]を算出する
//引数	: なし
//返り値	: なし
//備考 :1msタスク
void calc_rotation_angle ( void )
{
    rotation_angle += rotation_speed * 0.001; // [rad/s] * [s]
}

//機能	: マウスの角度を取得する
//引数	: なし
//返り値	: マウスの角度[rad]
float get_rotation_angle ( void )
{
    return rotation_angle;
}

//機能	: マウスの角度を取得する
//引数	: なし
//返り値	: なし
void set_rotation_angle ( float angle )
{
    rotation_angle = angle;
}


//機能	: カルマンフィルタによる速度推定
//引数	: なし
//返り値	: なし
//備考	:1msタスク
void speed_estimate_kalman ( void )
{

	float odo_x = 0;
	float odo_v = 0;

	float P11_before = 0;
	float P12_before = 0;
	float P21_before = 0;
	float P22_before = 0;

	static float P11_after = 0;
	static float P12_after = 0;
	static float P21_after = 0;
	static float P22_after = 0;

	float inv_vec = 0;
	float kalman_gain_den = 0;
	float kalman_gain_x = 0;
	float kalman_gain_v = 0;

	  ////////////////////
	 //	 予測ステップ	//
	////////////////////

	//オドメトリによる事前推定値
	odo_x = est_x + dt * est_v + 0.5 * dt2 * IMU_GetAccel_X();
	odo_v = est_v + dt * IMU_GetAccel_X();
	
	//事前推定値の誤差分散計算
	P11_before = P11_after + dt*P21_after + dt*P12_after + dt2*P22_after + 0.25*dt4*IMU_X_dispersion;
	P12_before = P12_after + dt*P22_after + 0.5*dt3*IMU_X_dispersion;
	P21_before = P21_after + dt*P22_after + 0.5*dt3*IMU_X_dispersion;
	P22_before = P22_after + dt2*IMU_X_dispersion;

	  ///////////////////////////
	 //	 フィルタリングステップ	//
	///////////////////////////

	//イノベーションベクトルの算出
	inv_vec = get_move_speed() - odo_v;

	//kalman filterのgain算出
	kalman_gain_den = P22_before + encorder_dispersion;
	kalman_gain_x = P12_before / kalman_gain_den;
	kalman_gain_v = P22_before / kalman_gain_den;

	//状態推定値更新
	est_x = odo_x + inv_vec * kalman_gain_x;
	est_v = odo_v + inv_vec * kalman_gain_v;
	
	//推定誤差分散更新
	P11_after = P11_before - P12_before * P21_before / kalman_gain_den;
	P12_after = P12_before - P12_before * P22_before / kalman_gain_den;
	P21_after = P21_before - P21_before * P22_before / kalman_gain_den;
	P22_after = P22_before - P22_before * P22_before / kalman_gain_den;

}

//機能	: カルマンフィルタで推定した速度を取得する。
//引数	: なし
//返り値	: なし
float get_est_v ( void )
{
	return est_v;
}
