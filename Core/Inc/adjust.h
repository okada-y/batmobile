#ifndef ADJUST_H_
#define ADJUST_H_

typedef enum{
    right,
    left,
    both_side,
    none
}side_wall_ctrl;

void clr_adjust(void);
void adjust_1ms (void);
void set_target_side_sensor(void);
void calc_motor_vol_side_wall ( void );
void calc_motor_vol_front_wall ( void );
float get_target_vol_sum_frontwall ( void );
float get_target_vol_diff_frontwall ( void );
void clr_frontwall_operate_history ( void );
void front_wall_calibrate (void);
void calibrate_tim (void);
void set_target_side_sensor(void);
uint8_t get_side_wall_ctrl_mode ( void );
void calc_side_wall_ctrl_mode ( void );
double get_side_wall_err(void);
void adjust_theta_side_wall(void);
void calc_motor_vol_side_wall(void);
float get_target_vol_diff_sidewall ( void );
float get_target_sensor_sr ( void );
float get_target_sensor_sl ( void );
double get_side_sensor_l_th ( void );
double get_side_sensor_r_th ( void );
void wall_break_calibrate(void);
void en_wall_break_calibrate(void);
void set_wall_break_mode(uint8_t wall_info);
void set_wall_break_ref(float tmp_wall_break);



#endif /* ADJUST_H_*/
