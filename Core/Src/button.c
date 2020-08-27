/*
 * button.c
 *
 *  Created on: May 18, 2020
 *      Author: 岡田 泰裕
 */
#include "button.h"
#include "indicator.h"

#define  cnt_max (300) //チャタリング防止用ウェイト時間

uint8_t button_state;//0:押していない　1:押している

//機能	: ボタン情報を参照し、状態を返す。チャタリング防止用にウェイト付き
//引数	: なし
//返り値	: ボタンの繊維状態（立ち上がり、立下り）
//備考 	: 1msタスク
uint8_t read_button()
{
	static uint16_t cnt; //チャタリング防止用カウンタ
	static uint8_t button_state; //ボタンの状態 0:low 1:High
	static uint8_t button_read_mode; //ボタンの読み込みモード 0:standby 1:wait
	uint8_t tmp_button_state;

	//読み込みモードの時
	if(button_read_mode == 0)
	{
		tmp_button_state = HAL_GPIO_ReadPin(Push_SW_GPIO_Port, Push_SW_Pin);

		if(button_state != tmp_button_state)
		{
			button_state = tmp_button_state;
			button_read_mode = 1;
		}
	}
	//ウェイトモードの時
	else
	{
		cnt = 1+cnt;
		if (cnt==cnt_max)
		{
			cnt = 0;
			button_read_mode= 0;
		}
	}

	return button_state;
}

//機能	: ボタンによるカウンタのインクリメント
//引数	: なし
//返り値	: カウンタ値(4bitまで)
//備考 	: 1msタスク

uint8_t cnt_button(void)
{
	static uint8_t cnt;
	static uint8_t mode; //0:スタンバイ　1:ウェイト


	if (button_state)		//スイッチがHighであれば
	{
		if(mode == 0)//かつスタンバイモードであれば
		{
		//カウンタをインクリメントし、ウェイト状態に遷移
		cnt = cnt+1;
		mode = 1;
		}
	}
	else
	{
		//スイッチがlowのとき、スタンバイモードに遷移
		mode = 0;
	}

	//8bit -> 4bit
	cnt &= 0b00001111;

	return cnt;
}



