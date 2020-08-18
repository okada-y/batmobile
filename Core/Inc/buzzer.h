/*
 * buzzer.h
 *
 *  Created on: May 18, 2020
 *      Author: 岡田 泰裕
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include <stdio.h>
#include "tim.h"

#define BPM				(100)	//曲の速さ

#define note_whole		(60.0f/BPM*4.0f*1000.0f)	//全音符の長さ(ms)
#define note_half		(60.0f/BPM*2.0f*1000.0f)	//2分音符の長さ(ms)
#define note_Quarter	(60.0f/BPM*1000.0f)			//4分音符の長さ(ms)
#define note_eighth		(60.0f/BPM/2.0f*1000.0f)	//8分音符の長さ(ms)
#define note_sixteenth	(60.0f/BPM/4.0f*1000.0f)	//16分音符の長さ(ms)

//音階
#define  Scale_C 	(3669)
#define  Scale_CS 	(3463)
#define  Scale_D 	(3269)
#define  Scale_DS 	(3086)
#define  Scale_E 	(2912)
#define  Scale_F 	(2749)
#define  Scale_FS 	(2595)
#define  Scale_G 	(2449)
#define  Scale_GS 	(2312)
#define  Scale_A 	(2182)
#define  Scale_AS 	(2059)
#define  Scale_B   	(1944)
#define  Scale_CH   (1835)
#define  Scale_CHS 	(1732)
#define  Scale_DH 	(1635)
#define  Scale_DHS 	(1543)
#define  Scale_EH 	(1456)
#define  Scale_FH 	(1374)
#define  Scale_FHS 	(1297)
#define  Scale_resonance 	(240)

//休符
#define  Scale_Reft (9999)
//スタンバイモード
#define  Scale_Stanby (9998)

//関数定義
void		TIM2_PWM_CC_set		(uint8_t);
void 		TIM2_PWM_START		(void);
uint8_t		Set_buzzer_tone		(uint16_t, float);
uint8_t		Melody_A			(uint8_t);
void set_buzzer_flg(uint8_t b_flg);
void buzzer_1ms(void);

//グローバル変数定義
extern uint8_t Melody_A_flg;//メロディAを開始するフラグ

#endif /* INC_BUZZER_H_ */
