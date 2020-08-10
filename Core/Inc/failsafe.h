/*
 * failsafe.h
 *
 *  Created on: 2020/08/09
 *      Author: 岡田 泰裕
 */

#ifndef INC_FAILSAFE_H_
#define INC_FAILSAFE_H_

extern uint8_t failsafe_en_flg;

uint8_t jud_mouse_state_err(void);
void Processing_on_error(void);
void set_failsafe_flg(uint8_t);
uint8_t jud_front_wall_err(void);
uint8_t jud_speed_m_err(void);
uint8_t jud_theta_err(void);

#endif /* INC_FAILSAFE_H_ */
