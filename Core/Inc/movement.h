#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "target.h"

typedef enum {
	 already = 0, //走行中
	 start = 1, //停止直後
} run_start;

typedef enum {
	nowall 	= 0, //壁なし
	wall 	= 1, //壁あり
} wall_flg;

uint8_t move_comp_jud ( void );
uint8_t move_comp_jud_stop ( void );
uint8_t rotate_comp_jud ( void );
void start_acceleration (void);
void half_acceleration (void);
void half_deceleration (void);
void constant_speed (float length, e_movement_pattern_No turn_num,float fin_speed);
void constant_speed_offset (float offset_length);
void turn_pattern(e_movement_pattern_No pattern_No,float target_angle );
void turn_clk_90 (void);
void turn_conclk_90 (void);
void turn_conclk_180 (void);
void clr_run_first_flg (void);
void clr_wall_flg (void);
void set_front_wall_flg ( void );
void set_rigth_wall_flg ( void );
void set_left_wall_flg ( void );
void  move_front (unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
void move_front_long(unsigned char straight_count,unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
void move_right  (unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
void move_left  (unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);
void move_back  (unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property);

void slalom_clock_s90 (unsigned char wall_flg);
void slalom_conclock_s90 (unsigned char wall_flg);

void slalom_clock_45(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_conclock_45(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_clock_90(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_conclock_90(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_clock_V90(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_conclock_V90(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_clock_135(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_conclock_135(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_clock_180(unsigned char wall_flg,unsigned char move_dir_property);
void slalom_conclock_180(unsigned char wall_flg,unsigned char move_dir_property);


#endif /* MOVEMENT_H_*/
