#include "buzzer.h"

uint8_t Melody_A_flg;

void TIM2_PWM_START(void)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
}

void TIM2_PWM_CC_set(uint8_t cc)
{
	if (cc>99)
	{
		cc = 99;
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, cc);
}

//機能	:指定された音を指定した時間出力する
//引数	:音、長さ(ms)
//返り値	:完了フラグ
//備考	:1msタスク

uint8_t Set_buzzer_tone(uint16_t Scale, float notes)
{
	static uint16_t cnt;
	static uint16_t Current_Scale;
	static uint8_t mode;	//音の出力完了モード 0:スタンバイ　1:出力中　2:完了

	if(Scale != Scale_Stanby){
		//音が更新されたとき
		if(Current_Scale != Scale)
		{
			cnt = 0;
			Current_Scale = Scale;
		}
		//音が更新されていないとき
		else
		{
			//出力時間が完了しているとき
			//スタンバイモードに移行して、完了を通知
			if(cnt > notes){
			TIM2_PWM_CC_set(0);
			mode = 2;
			Current_Scale = Scale_Stanby;
			return mode;
			}
		}
		//音が更新されているとき、コンパレータ値をセット
		if(cnt == 0)
		{
			//休符の時
			if(Scale == Scale_Reft){
				TIM2_PWM_CC_set(0);
			}
			else
			{
				//分周値を変更して音を変調する
				__HAL_TIM_SET_PRESCALER(&htim2,Scale);
				TIM2_PWM_CC_set(50);
			}
		}
		cnt = cnt + 1;
		mode = 1;
		return mode;
	}
	else
	{
		mode = 0;
		return mode;
	}

}

//機能	:旋律を出力する
//引数	:
//返り値	:完了フラグ
//備考	:1msタスク
uint8_t Melody_A(uint8_t flg)
{
		static uint8_t cnt; //演奏する音符
		static uint8_t temp; //各音符の完了フラグ監視

		//演奏終了&演奏フラグが立った時、
		if(flg == 1 && cnt == 0)
		{
			//演奏開始。
			cnt = 1;
		}
		switch(cnt)
		{
		//cnt=0のときは演奏しない。
		case 0:
			break;
		case 1:
			temp = Set_buzzer_tone(Scale_A,note_eighth);
			break;
		case 2:
			temp = Set_buzzer_tone(Scale_F,note_eighth);
			break;
		case 3:
			temp = Set_buzzer_tone(Scale_DH,note_eighth);
			break;
		case 4:
			temp = Set_buzzer_tone(Scale_CH,note_Quarter);
			break;
		case 5:
			temp = Set_buzzer_tone(Scale_A,note_eighth);
			break;
		case 6:
			temp = Set_buzzer_tone(Scale_F,note_Quarter);
			break;
		case 7:
			temp = Set_buzzer_tone(Scale_D,note_eighth);
			break;
		case 8:
			temp = Set_buzzer_tone(Scale_G,note_half);
			break;
		case 9:
			temp = Set_buzzer_tone(Scale_Reft,note_Quarter);
			break;
		case 10:
			temp = Set_buzzer_tone(Scale_F,note_eighth);
			break;
		case 11:
			temp = Set_buzzer_tone(Scale_G,note_eighth);
			break;
		case 12:
			temp = Set_buzzer_tone(Scale_A,note_eighth);
			break;
		case 13:
			temp = Set_buzzer_tone(Scale_DH,note_Quarter);
			break;
		case 14:
			temp = Set_buzzer_tone(Scale_EH,note_eighth);
			break;
		case 15:
			temp = Set_buzzer_tone(Scale_FH,note_eighth);
			break;
		case 16:
			temp = Set_buzzer_tone(Scale_CH,note_whole);
			break;
		case 17:
			temp = Set_buzzer_tone(Scale_Reft,note_eighth + note_Quarter);
			break;
		case 18:
			temp = Set_buzzer_tone(Scale_A,note_eighth);
			break;
		case 19:
			temp = Set_buzzer_tone(Scale_F,note_eighth);
			break;
		case 20:
			temp = Set_buzzer_tone(Scale_DH,note_eighth);
			break;
		case 21:
			temp = Set_buzzer_tone(Scale_CH,note_Quarter);
			break;
		case 22:
			temp = Set_buzzer_tone(Scale_A,note_eighth);
			break;
		case 23:
			temp = Set_buzzer_tone(Scale_F,note_Quarter);
			break;
		case 24:
			temp = Set_buzzer_tone(Scale_D,note_eighth);
			break;
		case 25:
			temp = Set_buzzer_tone(Scale_G,note_half);
			break;
		case 26:
			//フラグを下ろして演奏終了
			cnt = 0;
			Melody_A_flg = 0;
			temp = 0;
			break;
		}

		if (temp == 2)
		{
			cnt = cnt + 1;
		}

		return cnt;
}
