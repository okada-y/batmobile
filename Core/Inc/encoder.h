/*
 * encoder.h
 *
 *  Created on: Jun 30, 2020
 *      Author: 岡田 泰裕
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdio.h>

uint16_t encoder_Getangle_l( void );
uint16_t encoder_Getangle_r( void );
uint16_t encoder_correct_angle_l( uint16_t );
uint16_t encoder_correct_angle_r( uint16_t );

#define ENC_RESOLUTION 	(65535)		/* encoderの分解能 */
#define table_step_encoder 	(2048)			/*テーブルの分解能 */
#define table_num_encoder (33)            //横壁距離用テーブルの数

#endif /* INC_ENCODER_H_ */
