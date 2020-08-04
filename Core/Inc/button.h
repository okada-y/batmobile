/*
 * button.h
 *
 *  Created on: May 18, 2020
 *      Author: 岡田 泰裕
 */
#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdio.h>
#include "gpio.h"



uint8_t read_button(void);//ボタン情報読み込み(1msタスク)
uint8_t cnt_button(void);//ボタンによるカウンタのインクリメント





#endif


