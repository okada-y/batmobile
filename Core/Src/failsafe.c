/*
 * failsafe.c
 *
 *  Created on: 2020/08/09
 *      Author: 岡田 泰裕
 */

#include "index.h"

uint8_t failsafe_en_flg = 0; 	//フェイルセーフ有効化フラグ　0:無効　1:有効
uint8_t mouse_state_err = 0; //マウスの速度、角速度のエラーフラグ　0:正常 1:エラー

static uint8_t speed_m_err_count = 0; //並進方向のエラーカウンタ
static uint8_t theta_err_count = 0; 	//角度方向のエラーカウンタ
static uint16_t front_wall_err_count = 0; 	//角度方向のエラーカウンタ

//機能	: フェイルセーフフラグをセットする
//引数	: 0（無効)or 1(有効)
//返り値	: なし
//備考  :
void set_failsafe_flg(uint8_t set_flg_num){
	failsafe_en_flg = set_flg_num;
}

//機能	: 動作モードに応じたエラー値を返す
//引数	: 動作モード（ジャイロ、横壁、前壁）
//返り値	: 判定状態 0:正常　1:エラー
//備考  : 1msタスク
uint8_t jud_mouse_state_err(void){

	uint8_t temp_mode_num = 0;
	uint8_t temp_result = 0;

	//制御モードを取得
	temp_mode_num = get_mode_ctrl();

	//制御モードに応じたフェイルセーフを実施
	switch(temp_mode_num){
	case trace://ジャイロ使用時は角度、速度両方
		temp_result = jud_speed_m_err()||jud_theta_err();
		//前壁制御のエラーカウンタをリセット
		front_wall_err_count =0;
		break;
	case side_wall://横壁制御時は速度のみ
		temp_result = jud_speed_m_err();
		//角度のエラーカウンタをリセット
		theta_err_count = 0;
		front_wall_err_count =0;
		break;
	case front_wall://前壁制御時は時間でエラーを返す
		temp_result = jud_front_wall_err();
		//速度、角度のエラーカウンタをリセット
		theta_err_count = 0;
		speed_m_err_count = 0;
		break;
	}
	return temp_result;
}

//機能	: 前壁制御モードが一定時間続くとエラーを返す
//引数	: なし
//返り値	: 判定状態 0:正常　1:エラー
//備考  : 1msタスク
uint8_t jud_front_wall_err(void){

	uint8_t err_flag = 0;	//エラーフラグ

	//前壁制御時間を監視
	//リミッタ処理
		if(front_wall_err_count < 65535){
			front_wall_err_count += 1;
		}
		else{
			front_wall_err_count = 65535;
		}

	//カウンタと設定閾値を比較、超えていればエラーフラグをたてる
	if(front_wall_err_count > front_wall_err_count_th ){
		err_flag = 1;
	}
	else{
		err_flag = 0;
	}

	//エラーフラグを返す
	return err_flag;
}


//機能	: 速度追従を監視し、一定時間偏差があればエラーを返す
//引数	: なし
//返り値	: 判定状態 0:正常　1:エラー
//備考  : 1msタスク
uint8_t jud_speed_m_err(void){

	uint8_t err_flag = 0;	//エラーフラグ
	float m_speed_err = 0;	//速度偏差

	//偏差取得
	m_speed_err = ABS(get_target_move_speed()-get_move_speed_ave());

	//偏差が目標値を超えている場合、カウンタをインクリメント
	if ((m_speed_err > speed_m_err_th)){
		//リミッタ処理
		if(speed_m_err_count < 255){
			speed_m_err_count += 1;
		}
		else{
			speed_m_err_count = 255;
		}

	}
	//偏差が目標値範囲内のとき、カウンタをリセット
	else{
		speed_m_err_count = 0;
	}

	//カウンタと設定閾値を比較、超えていればエラーフラグをたてる
	if(speed_m_err_count > mouse_state_err_count_th ){
		err_flag = 1;
	}
	else{
		err_flag = 0;
	}

	//エラーフラグを返す
	return err_flag;
}

//機能	: 角度追従を監視し、一定時間偏差があればエラーを返す
//引数	: なし
//返り値	: 判定状態 0:正常　1:エラー
//備考  : 1msタスク
uint8_t jud_theta_err(void){

	uint8_t err_flag = 0;	//エラーフラグ
	float theta_err = 0;	//角度偏差

	//偏差取得
	theta_err = ABS(get_ideal_angle()-get_rotation_angle());

	//偏差が目標値を超えている場合、カウンタをインクリメント
	if ((theta_err > theta_err_th)){
		theta_err_count += 1;
	}
	//偏差が目標値範囲内のとき、カウンタをリセット
	else{
		theta_err_count = 0;
	}

	//カウンタと設定閾値を比較、超えていればエラーフラグをたてる
	if(theta_err_count > mouse_state_err_count_th ){
		err_flag = 1;
	}
	else{
		err_flag = 0;
	}

	//エラーフラグを返す
	return err_flag;
}
//機能	: 割り込みにてエラー時の処理
//引数	: なし
//返り値	: 判定状態 0:正常　1:エラー
//備考  : 1msタスク
void Processing_on_error(void)
{
	  //エラー時処理
	  clr_mode_state();
	  Sensor_StopADC();
	  set_duty_r(0);	//motor_r 停止
	  set_duty_l(0);	//motor_l 停止

	  while(1){ // エラー時のループ処理
		  LED_ALL_ON();
		  HAL_Delay(200);
		  LED_ALL_OFF();
		  HAL_Delay(200);
	  }
}

