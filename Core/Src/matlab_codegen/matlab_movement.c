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
	move_front_long(straight_count,start_flg,wall_flg,move_dir_property);
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
	half_deceleration();//半区画減速で中央に停止
	LED_ALL_ON();
	HAL_Delay(1000);
	LED_ALL_OFF();

}
