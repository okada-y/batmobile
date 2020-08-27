/*
 * mode.c
 *
 *  Created on: 2019/08/22
 *      Author: 岡田 泰裕
 */
#include"index.h"

typedef enum{
	before, //モード選択中
	after	//モード実行準備完了
}MODE_DECIDE;

static uint8_t mode_count = 0;	  //mode選択用カウンタ
static uint8_t mode_number = 0;   //mode番号
static MODE_DECIDE ready_mode = before;
static MODE_STATE mode_state = preparation;

uint8_t run_mode_1 = 0;
uint8_t run_mode_2 = 0;
uint8_t run_mode_3 = 0;

//機能 	:メインモード処理
//引数 	:なし
//返り値:なし
void mode_main(void)
{
	//初期処理
	ready_mode = before; 		//モード決定状態を決定前に
	mode_state = preparation;	//モード開始状態を開始前に
	mode_count = 0;				//modeカウンタ初期化
	
	Sensor_StopADC();	//IRセンサ停止
	
	while(ready_mode == before) //モード選択モードのとき
	{	
		mode_select();
		ready_mode = mode_decide_jud();
	}
	
	if (get_mode_number() == 14){
		maze_run_mode_decide();
	}

	//モード開始可能状態に遷移したとき、2回LEDを点滅させる。
	for(int i=0; i<2; i++)
	{ 
		LED_ALL_ON();
		HAL_Delay(700);
		LED_ALL_OFF();
		HAL_Delay(300);
	}

	//IRセンサを稼働させる。
	Sensor_Initialize( );	

	//モード開始を待機
	mode_start();
}


//機能 	:モード実行状態を返す
//引数 	:なし
//返り値	:モード実行状態
uint8_t get_mode_state(void)
{
	return mode_state;
}

//機能 	:モード実行状態をクリアする
//引数 	:なし
//返り値	:なし
void clr_mode_state(void)
{
	mode_state = preparation;
}

//機能 	:モード番号を返す
//引数 	:なし
//返り値:モード番号
uint8_t get_mode_number(void)
{
	return mode_number;
}

//機能 	:モード番号をセットする
//引数 	:新規のモード番号
//返り値	:なし
void set_mode_number(uint8_t mn)
{
	mode_number = mn;
}

//機能	:右タイヤによるインジケータ用番号変更
//引数	:現在の番号
//返り値	変更された番号
uint8_t select_num_r_tire (uint8_t temp_num)
{
	while(get_tire_r_speed() != 0); //いったんタイヤ停止するまで待ち
	init_tire_speed(); //タイヤの最大、最低速度初期化
	HAL_Delay(200);	//タイヤ最大値更新時間

	if (get_tire_r_speed_max() > mode_count_up_th) //右タイヤ速度＞正の閾値の時の処理
	{
		temp_num	+= 1;
	}

	if (get_tire_r_speed_min() < mode_count_down_th) //m右タイヤ速度＜負の閾値の時の処理
	{
		temp_num -= 1;
	}

		return temp_num;
}

//機能	:右タイヤの速度を入力に、モード番号を変更、LEDに出力
//引数	:なし
//返り値	:なし
void mode_select (void)
{
	static uint8_t mode_number_old = 0;

	mode_count =  select_num_r_tire (mode_count);

	mode_count &= 0b00001111;	//8bit->4bit

	//モードカウント値が更新されていれば、インジケータの点灯パターンを変更
	if (mode_count != mode_number_old){
		Indicator_number(mode_count);
		}

	mode_number_old = mode_count;
	set_mode_number(mode_count);//モード番号にセット
}

//機能	:左タイヤ速度を入力とし、モードを決定する。
//引数	:なし
//返り値	:モード決定状態 0:決定前 1:決定後
uint8_t mode_decide_jud(void)
{
	uint8_t temp = 0;
	if(mode_stanby_th < get_tire_l_speed_max() )
	{
		temp = after;
	}
	else
	{
		temp = before;
	}
	return temp;
}

//機能	:IRセンサによるモード動作開始SW
//引数	:なし
//返り値	:なし
void mode_start(void)
{
	//IRセンサ値稼働から少し待つ
	HAL_Delay(500);
	while(1)/*m右前センサをモード開始のスイッチとする。*/
	{
		if( Sensor_GetValue(front_right) >= 500)
		{
			set_buzzer_flg(1);//ブザー出力
			IMU_ResetReference(); //IMUのリファレンス取得
			mode_state = process;//モードを実行中に遷移
			break;
		}
	}
}

//機能:迷路走行時のモードを選択する
//引数	:なし
//返り値	:なし
void maze_run_mode_decide(void)
{


	uint8_t l_run_mode_1 = 0;//探索or最短
	uint8_t l_run_mode_2 = 0;//走行モード詳細（足立？全面？ななめ？直進？）
	uint8_t l_run_mode_3 = 0;//速度設定


	//走行モード１（探索or最短）の選択
	while(!button_state) //モード選択モードのとき
	{
		l_run_mode_1 = select_num_r_tire(l_run_mode_1);
		Indicator_number(l_run_mode_1);
	}
	//ブザーを1回鳴らす
	set_buzzer_flg(2);
	//ボタンを離されるまで停止
	while(button_state){

	}

	//走行モード２（探索or最短の詳細モード）の選択
	while(!button_state) //モード選択モードのとき
	{
		l_run_mode_2 = select_num_r_tire(l_run_mode_2);
		Indicator_number(l_run_mode_2);
	}
	//ブザーを2回鳴らす
	set_buzzer_flg(2);
	HAL_Delay(200);
	set_buzzer_flg(2);
	//ボタンを離されるまで停止
	while(button_state){}

	//走行モード3（速度）の選択
	while(!button_state) //モード選択モードのとき
	{
		l_run_mode_3 = select_num_r_tire(l_run_mode_3);
		Indicator_number(l_run_mode_3);
	}
	//ブザーを3回鳴らす
	set_buzzer_flg(2);
	HAL_Delay(200);
	set_buzzer_flg(2);
	HAL_Delay(200);
	set_buzzer_flg(2);
	//ボタンを離されるまで停止
	while(button_state){}

		//グローバル変数へ展開
	run_mode_1 = l_run_mode_1;
	run_mode_2 = l_run_mode_2;
	run_mode_3 = l_run_mode_3;


}

