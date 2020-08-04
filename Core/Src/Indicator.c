/*
 * Indicator.c
 *
 *  Created on: May 18, 2020
 *      Author: 岡田 泰裕
 */

//LEDによるインジケータの関数群

#include "indicator.h"

//機能	:引数の番号をLEDに出力
//引数	:なし
//返り値	:なし
void Indicator_number (uint8_t number)
{
	switch(number)
	{
	/*mode_numberに応じてLED点灯処理*/
		case 0:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
		break;

		case 1:
			HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED1_Pin, GPIO_PIN_SET);
		break;

		case 2:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED2_Pin, GPIO_PIN_SET);
		break;

		case 3:
			HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin, GPIO_PIN_SET);
		break;

		case 4:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin|LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED3_Pin, GPIO_PIN_SET);
		break;

		case 5:
			HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED3_Pin, GPIO_PIN_SET);
		break;

		case 6:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin, GPIO_PIN_SET);
		break;

		case 7:
			HAL_GPIO_WritePin(GPIOA,LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);
		break;

		case 8:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED4_Pin, GPIO_PIN_SET);
		break;

		case 9:
			HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED4_Pin, GPIO_PIN_SET);
		break;

		case 10:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED4_Pin, GPIO_PIN_SET);
		break;

		case 11:
			HAL_GPIO_WritePin(GPIOA,LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin|LED4_Pin, GPIO_PIN_SET);
		break;

		case 12:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED4_Pin, GPIO_PIN_SET);
		break;

		case 13:
			HAL_GPIO_WritePin(GPIOA,LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_SET);
		break;

		case 14:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_SET);
		break;

		case 15:
			HAL_GPIO_WritePin(GPIOA,LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_SET);
		break;
	}
}
