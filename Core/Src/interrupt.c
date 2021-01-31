/*
 * interrupt.c
 *
 *  Created on: Aug 4, 2019
 *      Author: 岡田 泰裕
 */


#include "index.h"

#define TIMER_COUNT		(__HAL_TIM_GET_COUNTER(&htim6))
#define TIMER_LOAD		(__HAL_TIM_GET_AUTORELOAD(&htim6))
#define TIMER_PSC		((&htim6)->Instance->PSC)

static uint32_t		interrupt_count_now;
static uint32_t		interrupt_duty;
static uint32_t		interrupt_duty_max = 0;
static float		boot_time = 0.f;


/* ---------------------------------------------------------------
	1ms周期で割り込み処理関数
--------------------------------------------------------------- */
void Interrupt_Main( void )
{
		calc_move_speed();
		button_state = read_button();
		buzzer_1ms();

		if(get_mode_state() == process)
		{
			//フェイセーフが無効、もしくは有効かつエラーフラグが立っていないとき
			if(!failsafe_en_flg || !jud_mouse_state_err()){
			switch(get_mode_number())
			{
				case 0 :
					break;

				case 1 :
					 data_get();
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					break;

				case 2 :
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
					 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
					 adjust_1ms();			//壁制御による印加電圧を計算
					 motor_1ms();			//制御モードに応じた印加電圧を出力
					 data_get();
					break;

				case 3 :
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
					 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
					 adjust_1ms();			//壁制御による印加電圧を計算
					 motor_1ms();			//制御モードに応じた印加電圧を出力
					 data_get();
					break;

				case 4 :
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
					 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
					 adjust_1ms();			//壁制御による印加電圧を計算
					 motor_1ms();			//制御モードに応じた印加電圧を出力
					 data_get();
					break;

				case 5 :
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
					 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
					 adjust_1ms();			//壁制御による印加電圧を計算
					 motor_1ms();			//制御モードに応じた印加電圧を出力
					 data_get();
					break;

				case 6 :
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
					 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
					 adjust_1ms();			//壁制御による印加電圧を計算
					 motor_1ms();			//制御モードに応じた印加電圧を出力
					 data_get();
					break;

				case 7 :
					 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
					 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
					 adjust_1ms();			//壁制御による印加電圧を計算
//					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					break;

				case 8 :
					 data_get();
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					break;

				case 14 :
					 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
					 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
					 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
					 adjust_1ms();			//壁制御による印加電圧を計算
					 motor_1ms();			//制御モードに応じた印加電圧を出力
					 //data_get();
					break;

				case 15 :
	//				 mouse_state_1ms();		//マウスの速度、角速度、移動距離、角度を計算
	//				 target_1ms();			//目標距離、角度から目標速度、角速度、加速度、角加速度を計算
	//				 calc_motor_vol_ctrl();	//速度、角速度制御による印加電圧を計算
	//				 adjust_1ms();			//壁制御による印加電圧を計算
	//				 motor_1ms();			//制御モードに応じた印加電圧を出力
	//				 data_get();
					break;
			}
		}
			else//フェールセーフ時の処理
			{
				Processing_on_error();
			}
	}
}


/* ---------------------------------------------------------------
	メイン割り込みの初期設定関数
--------------------------------------------------------------- */
void Interrupt_Initialize( void )
{
	HAL_TIM_Base_Start_IT( &htim6 );
}

/* ---------------------------------------------------------------
	割り込み前処理関数
--------------------------------------------------------------- */
void Interrupt_PreProcess( void )
{
//	static uint32_t		interrupt_count_old = 0;
//	static uint16_t		interrupt_count_old = 0;

	static uint64_t		boot_time_count = 0;

	interrupt_count_now = TIMER_COUNT;
//	boot_time_count += (uint16_t)(interrupt_count_now - interrupt_count_old);
//	interrupt_count_old = interrupt_count_now;
//	boot_time = (float)boot_time_count / (float)(HAL_RCC_GetPCLK1Freq() *2 / ( TIMER_PSC + 1) );

	boot_time_count += 1;
	boot_time = ((float)boot_time_count /(float)1000);
}

/* ---------------------------------------------------------------
	割り込み後処理関数
--------------------------------------------------------------- */
void Interrupt_PostProcess( void )
{
	interrupt_duty = (TIMER_COUNT - interrupt_count_now) * 1000 / TIMER_LOAD;
	interrupt_duty_max = MAX( interrupt_duty_max, interrupt_duty );

	if (interrupt_duty > 1000)
	{
		while(Set_buzzer_tone(Scale_CH,note_eighth)!=2)
		{
			HAL_Delay(1);
		}
		while(1)
		{
		}
	}

}

/* ---------------------------------------------------------------
	割り込み周期に占める呼び出し位置までの処理時間の割合を取得する関数
--------------------------------------------------------------- */
uint16_t Interrupt_GetDuty( void )
{
	return interrupt_duty;
}

/* ---------------------------------------------------------------
	上記割合の最大値を取得する関数
--------------------------------------------------------------- */
uint16_t Interrupt_GetDuty_Max( void )
{
	return interrupt_duty_max;
}

/* ---------------------------------------------------------------
	マイコン起動時からの経過時間を取得する関数
--------------------------------------------------------------- */
float Interrupt_GetBootTime( void )
{
	return boot_time;
}
