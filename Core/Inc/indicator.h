/*
 * indicator.h
 *
 *  Created on: May 18, 2020
 *      Author: 岡田 泰裕
 */

#ifndef INC_INDICATOR_H_
#define INC_INDICATOR_H_

#include <stdio.h>
#include "gpio.h"

#define LED_ALL_TOGGLE()	HAL_GPIO_TogglePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin)
#define LED_ALL_ON()			HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin,GPIO_PIN_SET)
#define LED_ALL_OFF()			HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin,GPIO_PIN_RESET)

void Indicator_number (uint8_t);

#endif /* INC_INDICATOR_H_ */
