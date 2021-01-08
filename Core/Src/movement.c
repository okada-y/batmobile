//動作方法を記述予定(move_back)など



#include <stdio.h>
#include "param.h"
#include "target.h"
#include "movement.h"
#include "adjust.h"
#include "exvol.h"
#include "mouse_state.h"
#include "main.h"
#include "index.h"

static run_start run_first_flg = 0;			// 走行開始フラグ 0:走行開始時　1:それ以外
static wall_flg	front_wall_flg = nowall;	//　前壁の有無フラグ
static wall_flg	right_wall_flg = nowall;	//　右壁の有無フラグ
static wall_flg	left_wall_flg = nowall;		//　左壁の有無フラグ

typedef enum {
	default_offset_run,
	before_offset_run,
	after_offset_run,
} e_offset_run_discrimination_flg;
static e_offset_run_discrimination_flg offset_run_discrimination_flg = default_offset_run;//オフセット距離判別用フラグ

//機能	: オフセット距離走行モード判定
//引数	: なし
//返り値	: オフセット距離走行中判別フラグ
uint8_t	get_offset_run_discrimination_flg(void)
{
	return offset_run_discrimination_flg;
}


//機能	: 移動完了判断
//引数	: なし
//返り値	: 判断結果(0:未完,1:完了)
uint8_t move_comp_jud ( void )
{
    return (uint8_t)((get_target_length() - get_move_length()) < 0);
}

//機能	: 移動完了判断（停止時）
//引数	: なし
//返り値	: 判断結果(0:未完,1:完了)
uint8_t move_comp_jud_stop ( void )
{
    return (uint8_t)((get_target_length() - get_move_length()) 
							< get_target_move_speed() * get_target_move_speed()/(2 * get_target_move_accel()));
}


//機能	: 回転完了判断
//引数	: なし
//返り値	: 判断結果(0:未完,1:完了)
uint8_t rotate_comp_jud ( void )
{
    return (uint8_t)(ABS(get_target_angle() - get_ideal_angle()) < rotate_comp_th);
}

//機能	: スタート時加速
//引数	: なし
//返り値	: なし
void start_acceleration (void)
{
	/*加速度等パラメータ、移動距離、終端速度設定*/
	set_target_move_param(search, 0.05-0.045, search_move_speed_max);

    /*半区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
	 	{
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);
    		//終端速度をキープするよう設定
    		set_target_move_param(search, 1.0, search_move_speed_max);
    		break;
    	}
    }
}

//機能	: 半分区画加速
//引数	: なし
//返り値	: なし
void half_acceleration (void)
{
	/*移動方向、加速モード設定*/
	/*加速度等パラメータ、移動距離、終端速度設定*/
	set_target_move_param(search, 0.045, search_move_speed_max);

    /*半区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
    	{
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);
    		//終端速度をキープするよう設定
    		set_target_move_param(search, 1.0, search_move_speed_max);
    		break;
    	}
    }
}


//機能	: 反区画減速
//引数	: なし
//返り値	: なし
void half_deceleration (void)
{
	/*移動方向、加速モード設定*/
	/*加速度等パラメータ、移動距離、終端速度設定*/
	set_target_move_param(search, 0.045, 0);
	/*停止時動作設定*/
	set_speed_under_lim_flg(slow);


    /*半区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
    	{
    		//停止処理
    		set_speed_under_lim_flg(zero);
    		HAL_Delay(100);
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);
    		//終端速度をキープするよう設定
    		set_target_move_param(search, 0.0, 0);
    		break;
    	}
    }
}

//機能	: 指定距離、指定速度にて直進
//引数	: 距離、パラメータセット番号
//返り値	: なし
void constant_speed (float length, e_movement_pattern_No turn_num,float fin_speed)
{

	/*加速度等パラメータ、移動距離、終端速度設定*/
	set_target_move_param(turn_num, length, fin_speed);


    /*一区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
    	{
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);
    		//終端速度をキープするよう設定
    		set_target_move_param(turn_num, 3.0, fin_speed);
    		break;
    	}
    }
}

//機能	: 一定距離定速
//引数	: オフセット長さ、速度
//返り値	: なし
void constant_speed_offset (float offset_length)
{

	/*加速度等パラメータ、移動距離、終端速度設定*/
	set_target_move_param(search, offset_length, search_move_speed_max);

    /*一区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
    	{
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);
    		//終端速度をキープするよう設定
    		set_target_move_param(search, 1.0, search_move_speed_max);
    		break;
    	}
    }
}

//機能	: 壁切れ検出有の一定距離定速
//引数	: オフセット長さ、速度
//返り値	: なし
void constant_speed_wall_cut (float length, e_movement_pattern_No turn_num,float fin_speed,uint8_t wall_sensor_dir,uint16_t wall_sensor_th, float wall_cut_length)
{

	bool wall_cut_flg = 0;

	/*加速度等パラメータ、移動距離、終端速度設定*/
	set_target_move_param(turn_num, length, fin_speed);


    /*一区画進むまで待機*/
    while (1)
    {
//    	初回壁切れ検出時、移動距離に壁切れ距離をセット。
    	if(wall_cut_flg == 0 && Sensor_GetValue(wall_sensor_dir)<wall_sensor_th)
    	{
    		set_ideal_length(wall_cut_length);
    		set_move_length(wall_cut_length);
    		wall_cut_flg  = 1;
    	}
    	if(move_comp_jud() && wall_cut_flg == 1)//壁切れ検出完了していて、走行完了しているときブレイク
    	{
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);
    		//終端速度をキープするよう設定
    		set_target_move_param(turn_num, 3.0, fin_speed);
    		break;
    	}
    }
}

//前壁距離がある値になるまで、定速
//引数	: 目標前壁距離
//返り値	: なし
void constant_speed_front_wall_adj(float target_front_wall_length)
{
	float temp=0;
	/*加速度等パラメータ、移動距離、終端速度設定*/
	set_target_move_param(search, 1.0, search_move_speed_max);

    while (1)
    {
    	temp = 0.5 * (get_wall_dis_table(Sensor_GetValue(front_left), front_left)
    				+get_wall_dis_table(Sensor_GetValue(front_right), front_right));
    	//前壁距離が目標値よりも小さくなるとき、終了
    	if(temp < target_front_wall_length){
    		//ブザーをならす。
    		set_buzzer_flg(2);
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);
    		//終端速度をキープするよう設定
    		set_target_move_param(search, 1.0, search_move_speed_max);
    		break;
    	}
    }
}

//機能	: ターン時のパラメータ（最高角速度、角加速度）をセットし、指定の角度だけ回る。
//引数	: ターン番号、ターン角度
//返り値	: なし
void move_turn_pattern(e_movement_pattern_No pattern_No,float l_target_angle )
{
	//ターンパラメータ、方向を設定する
	set_target_turn_param ( pattern_No );

	//目標角度に応じてターンモードを設定
	if (l_target_angle >= 0)
	{
		set_rotation_mode(counter_clockwise);
	}
	else
	{
		set_rotation_mode(clockwise);
	}
	//目標回転角度を設定
	set_target_angle(l_target_angle);



    /*目標角度までに達するまで待機*/
    while (1)
    {
    	if(rotate_comp_jud())
    	{
    	    /*理想角度、実角度をクリア*/
    	    set_ideal_angle(0.0);
    	    set_rotation_angle(0.0);
    	    set_target_angle(0.0);//現在角度をキープするため、目標角度を0とする。
    		/*理想移動距離、実移動距離をクリア*/
    		set_ideal_length(0.0);
    		set_move_length(0.0);

    		break;
    	}
    }
}

//機能	: 時計回りに90度回転
//引数	: なし
//返り値	: なし
void turn_clk_90 (void)
{
	/*回転方向設定*/
	set_rotation_mode(clockwise);
    set_target_angle(-PI/2);
    /*理想角度、実角度をクリア*/
    set_ideal_angle(0.0);
    set_rotation_angle(0.0);

    /*90度回転するまで待機*/
    while (1)
    {
    	if(rotate_comp_jud())
    	{
    		break;
    	}
    }

}

//機能	: 反時計回りに90度回転
//引数	: なし
//返り値	: なし
void turn_conclk_90 (void)
{
	/*回転方向設定*/
	set_rotation_mode(counter_clockwise);
	set_target_angle(PI/2);


    /*理想角度、実角度をクリア*/
    set_ideal_angle(0.0);
    set_rotation_angle(0.0);

    /*90度回転するまで待機*/
    while (1)
    {
    	if(rotate_comp_jud())
    	{
    		break;
    	}
    }

}

//機能	: 反時計回りに180度回転
//引数	: なし
//返り値	: なし
void turn_conclk_180 (void)
{
	/*回転方向設定*/
	set_rotation_mode(counter_clockwise);
    set_target_angle(PI);

    /*理想角度、実角度をクリア*/
    set_ideal_angle(0.0);
    set_rotation_angle(0.0);

    /*180度回転するまで待機*/
    while (1)
    {
    	if(rotate_comp_jud())//0.3deg手前まできたらブレイク
    	{
    		break;
    	}
    }

}

//機能	: 走行開始フラグのクリア
//引数	: なし
//返り値	: なし
void clr_run_first_flg (void)
{
	run_first_flg = start;
}

//機能	: 走行開始フラグセット
//引数	: なし
//返り値	: なし
void set_run_first_flg (void)
{
	run_first_flg = already;
}


//機能	: 壁の有無フラグのクリア
//引数	: なし
//返り値	: なし
void clr_wall_flg (void)
{
	front_wall_flg = nowall;
	right_wall_flg = nowall;
	left_wall_flg = nowall;
}

//機能	: 前壁フラグセット
//引数	: なし
//返り値	: なし
void set_front_wall_flg ( void )
{
	front_wall_flg = wall;
}

//機能	: 右壁フラグセット
//引数	: なし
//返り値	: なし
void set_rigth_wall_flg ( void )
{
	right_wall_flg = wall;
}

//機能	: 左壁フラグセット
//引数	: なし
//返り値	: なし
void set_left_wall_flg ( void )
{
	left_wall_flg = wall;
}

////////////////////////////////////////
/* スラローム軌跡生成関数			   */
////////////////////////////////////////

//機能	: 時計回り90度のスラローム軌跡
//引数	: 壁情報
//返り値	: なし
void slalom_clock_s90 (unsigned char wall_flg)
{
	//前壁の有無で前距離処理を変更する
	if ((wall_flg & 1) == 1 )//前壁があるとき
	{
		constant_speed_front_wall_adj(slalom_front_wall_adj);
	}else{//前壁がないとき
	//エンコーダによる一定距離移動
		constant_speed_offset(get_offset_length(turn_s90, before_offset) );
	}
	//90degターン
	//turn_clk_90();
	move_turn_pattern(turn_s90, -PI/2);


	/*理想移動距離、実移動距離をクリア*/
	set_ideal_length(0.0);
	set_move_length(0.0);
	//終端速度をキープするよう設定
	set_target_move_param(search, 1.0, search_move_speed_max);

	//後距離
	constant_speed_offset(get_offset_length(turn_s90, after_offset));
}

//機能	: 反時計回り90度のスラローム軌跡
//引数	: 壁情報
//返り値	: なし
void slalom_conclock_s90 (unsigned char wall_flg)
{
	//前壁の有無で前距離処理を変更する
	if ((wall_flg & 1) == 1 )//前壁があるとき
	{
		constant_speed_front_wall_adj(slalom_front_wall_adj);
	}else{//前壁がないとき
	//前距離
		constant_speed_offset(get_offset_length(turn_s90, before_offset));
	}
	//90degターン
	//turn_conclk_90();
	move_turn_pattern(turn_s90, PI/2);

	/*理想移動距離、実移動距離をクリア*/
	set_ideal_length(0.0);
	set_move_length(0.0);
	//終端速度をキープするよう設定
	set_target_move_param(search, 1.0, search_move_speed_max);

	//後距離
	constant_speed_offset(get_offset_length(turn_s90, after_offset));
}

//機能	:時計回り45度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_clock_45 (unsigned char wall_flg,unsigned char move_dir_property)
{
	if(move_dir_property == p_straight) //直進時
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed(get_offset_length(turn_45, before_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_45, after_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_45, -PI/4);

	if(move_dir_property == p_straight) //直進時
	{
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_45, after_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed(get_offset_length(turn_45, before_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	offset_run_discrimination_flg = default_offset_run;
}

//機能	: 反時計回り45度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_conclock_45 (unsigned char wall_flg,unsigned char move_dir_property)
{
	if(move_dir_property == p_straight) //直進時
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed(get_offset_length(turn_45, before_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_45, after_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_45, PI/4);

	if(move_dir_property == p_straight) //直進時
	{
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_45, after_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed(get_offset_length(turn_45, before_offset), turn_45, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	offset_run_discrimination_flg = default_offset_run;
}

//機能	:時計回り90度のスラローム軌跡(大廻)
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_clock_90(unsigned char wall_flg,unsigned char move_dir_property)
{
	//前距離の走行
	offset_run_discrimination_flg = before_offset_run;
	//壁があるときのみ壁切れの調整を入れる。
	if(Sensor_GetValue(side_right)>400){
		constant_speed_wall_cut(get_offset_length(turn_90, before_offset), turn_90, get_target_move_speed(), side_right,200,get_offset_length(turn_90, before_offset)*0.32);//現在の目標速度を終端速度とする
	}else {
		constant_speed(get_offset_length(turn_90, before_offset), turn_90, get_target_move_speed());//現在の目標速度を終端速度とする
	}

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_90, -PI/2);

	//後距離の走行
	offset_run_discrimination_flg = after_offset_run;
	constant_speed(get_offset_length(turn_90, after_offset), turn_90, get_target_move_speed());//現在の目標速度を終端速度とする

	offset_run_discrimination_flg = default_offset_run;
}

//機能	:反時計回り90度のスラローム軌跡（大廻）
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_conclock_90(unsigned char wall_flg,unsigned char move_dir_property)
{
	//前距離の走行
	offset_run_discrimination_flg = before_offset_run;
	//横壁の有無で壁切れの閾値変更
	//壁があるときのみ壁切れの調整を入れる。
	if(Sensor_GetValue(side_left)>600){
		constant_speed_wall_cut(get_offset_length(turn_90, before_offset), turn_90, get_target_move_speed(), side_left,400,get_offset_length(turn_90, before_offset)*0.31);//現在の目標速度を終端速度とする
	}else {
//		constant_speed_wall_cut(get_offset_length(turn_90, before_offset), turn_90, get_target_move_speed(), side_left,300,get_offset_length(turn_90, before_offset)*0.3);//現在の目標速度を終端速度とする
		constant_speed(get_offset_length(turn_90, before_offset), turn_90, get_target_move_speed());//現在の目標速度を終端速度とする
	}

	//constant_speed(get_offset_length(turn_90, before_offset), turn_90, get_target_move_speed());//現在の目標速度を終端速度とする

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_90, PI/2);

	//前距離の走行
	offset_run_discrimination_flg = after_offset_run;
	constant_speed(get_offset_length(turn_90, after_offset), turn_90, get_target_move_speed());//現在の目標速度を終端速度とする
}

//機能	:時計回り90度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_clock_V90(unsigned char wall_flg,unsigned char move_dir_property)
{
	//右センサで壁切れが検出されるまでは速度維持
	while(Sensor_GetValue(side_right)>250){
		set_ideal_length(0.0);
		set_move_length(0.0);
	}


	//前距離の走行
	offset_run_discrimination_flg = before_offset_run;
	constant_speed(get_offset_length(turn_V90, before_offset)-0.003, turn_V90, get_target_move_speed());//現在の目標速度を終端速度とする

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_V90, -PI/2);

	//後距離の走行
	offset_run_discrimination_flg = after_offset_run;
	constant_speed(get_offset_length(turn_V90, after_offset)+0.0025, turn_V90, get_target_move_speed());//現在の目標速度を終端速度とする
	offset_run_discrimination_flg = default_offset_run;
}

//機能	:反時計回り90度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_conclock_V90(unsigned char wall_flg,unsigned char move_dir_property)
{
	//左センサで壁切れが検出されるまでは速度維持
	while(Sensor_GetValue(side_left)>900)
	{
		set_ideal_length(0.0);
		set_move_length(0.0);
	}

	//前距離の走行
	offset_run_discrimination_flg = before_offset_run;
	constant_speed(get_offset_length(turn_V90, before_offset)-0.005, turn_V90, get_target_move_speed());//現在の目標速度を終端速度とする

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_V90, PI/2);

	//後距離の走行
	offset_run_discrimination_flg = after_offset_run;
	constant_speed(get_offset_length(turn_V90, after_offset)+0.005, turn_V90, get_target_move_speed());//現在の目標速度を終端速度とする
	offset_run_discrimination_flg = default_offset_run;
}

//機能	:時計回り135度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_clock_135(unsigned char wall_flg,unsigned char move_dir_property)
{
	if(move_dir_property == p_straight) //直進時
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed_wall_cut(get_offset_length(turn_135, before_offset)+0.002, turn_135, get_target_move_speed(), side_right,200,get_offset_length(turn_135, before_offset)/5);//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//右センサで壁切れが検出されるまでは速度維持
		while(Sensor_GetValue(side_right)>1000)
		{
			set_ideal_length(0.0);
			set_move_length(0.0);
		}
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_135, after_offset), turn_135, get_target_move_speed());//現在の目標速度を終端速度とする
	}

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_135, -PI*3/4);

	if(move_dir_property == p_straight) //直進時
	{
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_135, after_offset)+0.007, turn_135, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed(get_offset_length(turn_135, before_offset)+0.001, turn_135, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	offset_run_discrimination_flg = default_offset_run;
}

//機能	:反時計回り135度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_conclock_135(unsigned char wall_flg,unsigned char move_dir_property)
{
	if(move_dir_property == p_straight) //直進時
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed_wall_cut(get_offset_length(turn_135, before_offset), turn_135, get_target_move_speed(), side_left,400,get_offset_length(turn_135, before_offset)*0.45);//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//左センサで壁切れが検出されるまでは速度維持
		while(Sensor_GetValue(side_left)>400)
		{
			set_ideal_length(0.0);
			set_move_length(0.0);
		}
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_135, after_offset), turn_135, get_target_move_speed());//現在の目標速度を終端速度とする
	}

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_135, PI*3/4);

	if(move_dir_property == p_straight) //直進時
	{
		//後距離の走行
		offset_run_discrimination_flg = after_offset_run;
		constant_speed(get_offset_length(turn_135, after_offset)+0.002, turn_135, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	else//斜めの場合
	{
		//前距離の走行
		offset_run_discrimination_flg = before_offset_run;
		constant_speed(get_offset_length(turn_135, before_offset)+0.008, turn_135, get_target_move_speed());//現在の目標速度を終端速度とする
	}
	offset_run_discrimination_flg = default_offset_run;
}

//機能	:時計回り180度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_clock_180(unsigned char wall_flg,unsigned char move_dir_property)
{
	//前距離の走行
	offset_run_discrimination_flg = before_offset_run;
	constant_speed_wall_cut(get_offset_length(turn_180, before_offset), turn_180, get_target_move_speed(), side_right,300,get_offset_length(turn_180, before_offset)*0.14);//現在の目標速度を終端速度とする

	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_180, -PI);

	//前距離の走行
	offset_run_discrimination_flg = after_offset_run;
	constant_speed(get_offset_length(turn_180, after_offset), turn_180, get_target_move_speed());//現在の目標速度を終端速度とする
	offset_run_discrimination_flg = default_offset_run;
}

//機能	:反時計回り180度のスラローム軌跡
//引数	: 壁情報,進行方向属性
//返り値	: なし
void slalom_conclock_180(unsigned char wall_flg,unsigned char move_dir_property)
{
	//前距離の走行
	offset_run_discrimination_flg = before_offset_run;
	constant_speed_wall_cut(get_offset_length(turn_180, before_offset), turn_180, get_target_move_speed(), side_left,500,get_offset_length(turn_180, before_offset)*0.33);//現在の目標速度を終端速度とする
	//ターン
	offset_run_discrimination_flg = default_offset_run;
	move_turn_pattern(turn_180, PI);

	//前距離の走行
	offset_run_discrimination_flg = after_offset_run;
	constant_speed(get_offset_length(turn_180, after_offset), turn_180, get_target_move_speed());//現在の目標速度を終端速度とする
	offset_run_discrimination_flg = default_offset_run;
}

////////////////////////////////////////
/* 軌跡生成関数						   */
////////////////////////////////////////

//機能	: 前進
//引数	: なし
//返り値	: なし
void move_front (unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property)
{
	//停止直後の動作
	if(start_flg == start)
	{
		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
	}
	//走行中の動作
	if(start_flg == already)
	{
		//壁切れ補正フラグ処理
//		set_wall_break_mode(wall_flg);
//		set_wall_break_ref(0.09-wall_break_calib_ref_dis);
//		en_wall_break_calibrate();

		set_mode_ctrl(side_wall);
		constant_speed(0.09,search,search_move_speed_max);//定速で一マス前進
		set_mode_ctrl(trace);
	}
}

//機能：直線加速
//引数：直線区画数,終端速度,スタートフラグ,壁フラグ,進行方向属性
//返り値：なし
void move_front_long(unsigned char straight_count,float fin_speed, unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property)
{
		offset_run_discrimination_flg = default_offset_run;
		switch (move_dir_property){
		case p_straight:
			//停止直後の動作
			if(start_flg == start)
			{
				set_mode_ctrl(side_wall);
				constant_speed((straight_count -1)*0.09+0.045, straight, fin_speed);//定速でnマス前進
				set_mode_ctrl(trace);
			}
			//走行中の動作
			if(start_flg == already)
			{
				set_mode_ctrl(side_wall);
				constant_speed(straight_count * 0.09,straight,fin_speed);//定速でnマス前進
				set_mode_ctrl(trace);
			}
		break;

		case p_diagonal:
			{
				set_mode_ctrl(trace);
				constant_speed(straight_count *0.045*SQRT2, diagonal, fin_speed);//定速でnマス前進
			}
		break;
		}

}


//機能	: 右折
//引数	: なし
//返り値	: なし
void move_right  (unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property)
{
	if(start_flg == start){
		if((wall_flg & 1)== 1){
			front_wall_calibrate();
		}
		//turn_clk_90();		//時計回りに90度回転
		move_turn_pattern(turn_s90, -PI/2);
		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
	}
	if(start_flg == already){
		slalom_clock_s90 (wall_flg);
	}
}

//機能	: 左折
//引数	: なし
//返り値	: なし
void move_left  (unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property)
{
	if(start_flg == start){
		if((wall_flg & 1)== 1){
			front_wall_calibrate();
		}
		//turn_conclk_90();	//m反時計回りに90度回転
		move_turn_pattern(turn_s90, PI/2);
		set_mode_ctrl(side_wall);
		half_acceleration();//m半区画加速
		set_mode_ctrl(trace);
	}
	if(start_flg == already){
		slalom_conclock_s90 (wall_flg);
	}
}

//機能	: 後進
//引数	: なし
//返り値	: なし
void move_back(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property)
{
	//走行中であれば、減速させ、中央にとめる。
	if(start_flg == already){
		set_mode_ctrl(side_wall);
		half_deceleration();//m半区画減速で中央に停止
		set_mode_ctrl(trace);
	}

		//壁の有無で位置補正処理を変更

		//前壁があるとき
		if((wall_flg & 1)== 1){
			front_wall_calibrate();
			//右壁があるとき
			if((wall_flg & 2)== 2){
				//turn_clk_90();
				move_turn_pattern(turn_s90, -PI/2);
				front_wall_calibrate();
				//turn_clk_90();
				move_turn_pattern(turn_s90, -PI/2);
			}
			//右壁がなく左壁があるとき
			else if((wall_flg & 8)== 8){
					//turn_conclk_90();
					move_turn_pattern(turn_s90, PI/2);
					front_wall_calibrate();
					//turn_conclk_90();
					move_turn_pattern(turn_s90, PI/2);
			}
			//左右に壁がないとき
			else {
				//turn_conclk_180();	//反時計回りに180度回転
				move_turn_pattern(turn_s90, PI);
			}
		}
		//前壁がないとき
		else{
			//turn_conclk_180();	//反時計回りに180度回転
			move_turn_pattern(turn_s90, PI);
		}

		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
}

//停止処理
void move_stop(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property)
{
	half_deceleration();//半区画減速で中央に停止
	LED_ALL_ON();
	HAL_Delay(1000);
	LED_ALL_OFF();
}
