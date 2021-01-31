#include "index.h"
#include "matlab_movement.h"

/*matlab上での動作関数の定義
 *ハンドにてC側で記述した動作関数を各関数から呼び出すこと
 */


//@param スタート直後フラグ(0:直後でない, 1:スタート直後)
//@param ゴール直後フラグ(0:直後でない, 1:ゴール直後)
//@param 進行方向属性フラグ(0:直進, 1:斜め)

//前進
void m_move_front(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	move_front (start_flg,wall_flg,move_dir_property);
}

//前進_ロング
void m_move_front_long(unsigned char straight_count,unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){
//     c側で記述した動作関数を記述すること
	if(run_mode_1 == 1 && run_mode_2 == 1)//斜め探索時
	{
		move_front_long(straight_count,slalom_45_v_1 ,start_flg,wall_flg,move_dir_property);
	}else{
		move_front_long(straight_count,search_move_speed_max,start_flg,wall_flg,move_dir_property);
	}
}


//右折
void m_move_right(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	move_right (start_flg,wall_flg,move_dir_property);
}

//左折
void m_move_left(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	move_left (start_flg,wall_flg,move_dir_property);
}

//バック
void m_move_back(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	move_back (start_flg,wall_flg,move_dir_property);
}

//スタート時の動作（停止処理）
void m_start_movement(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	move_front(start_flg,wall_flg,move_dir_property);
}

//ゴール時の動作（停止処理）
void m_goal_movement(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	if(run_mode_1 == 1 && run_mode_2 == 1)//斜め走行時
	{
		set_target_move_param(search, 0.0, 0);//即停止
	}else{
		move_stop(start_flg,wall_flg,move_dir_property);
	}
}

//エラー時の動作（停止処理）
void m_error_movement(unsigned char error_flg){
//     c側でエラー処理を記述すること
	set_target_move_param(search, 0.0, 0);//即停止

	//エラーフラグの番号を点滅
	while(1){
		Indicator_number(error_flg);
		HAL_Delay(200);
		Indicator_number(0);
		HAL_Delay(200);
	}
}


//右V90度ターン
void m_turn_V90_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_clock_V90(wall_flg,move_dir_property);
}

//左V90度ターン
void m_turn_V90_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_conclock_V90(wall_flg,move_dir_property);
}

//右90度ターン
void m_turn_90_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_clock_90(wall_flg,move_dir_property);
}

//左90度ターン
void m_turn_90_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_conclock_90(wall_flg,move_dir_property);
}

//右180度ターン
void m_turn_180_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_clock_180(wall_flg,move_dir_property);
}

//左180度ターン
void m_turn_180_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_conclock_180(wall_flg,move_dir_property);
}

//右135度ターン
void m_turn_135_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_clock_135(wall_flg,move_dir_property);
}

//左135度ターン
void m_turn_135_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_conclock_135(wall_flg,move_dir_property);
}

//右45度ターン
void m_turn_45_r(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_clock_45(wall_flg,move_dir_property);
}

//左45度ターン
void m_turn_45_l(unsigned char start_flg,unsigned char wall_flg,unsigned char move_dir_property){  
//     c側で記述した動作関数を記述すること
	slalom_conclock_45(wall_flg,move_dir_property);
}
