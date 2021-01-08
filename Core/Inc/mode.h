/*
 * mode.h
 *
 *  Created on: Jan 30, 2020
 *      Author: Yasuhiro Okada
 */

#ifndef MODE_H_
#define MODE_H_

typedef enum{
	preparation, //モード準備中
	process	//モード実行中
}MODE_STATE;

extern uint8_t run_mode_1;
extern uint8_t run_mode_2;
extern uint8_t run_mode_3;

void mode_main(void);
uint8_t get_mode_state(void);
void clr_mode_state(void);
uint8_t get_mode_number(void);
void set_mode_number(uint8_t);
void mode_select (void);
uint8_t mode_decide_jud(void);
void mode_start(void);
uint8_t select_num_r_tire (uint8_t);
void maze_run_mode_decide(void);
void set_run_mode_3(uint8_t num);

#endif /* MODE_H_ */
