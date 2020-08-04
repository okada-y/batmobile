#include <stdio.h>
#include "index.h"
#include "lookuptable.h"

float table_leftwall[table_num_wall] = {
		0.18,
		0.11,
		0.083,
		0.069,
		0.06,
		0.053,
		0.046,
		0.042,
		0.039,
		0.0355,
		0.033,
		0.03,
		0.0285,
		0.026,
		0.0245,
		0.023,
		0.0215,
		0.0205,
		0.0195,
		0.0185,
		0.0175,
		0.0165,
		0.0157,
		0.0149,
		0.0141,
		0.0133,
		0.0125,
		0.0117,
		0.0111,
		0.0105,
		0.0099,
		0.0093,
		0.0087,
		0.0081,
		0.0075,
		0.007,
		0.0065,
		0.006,
		0.0055,
		0.005,
		0.0046,
		0.0042,
		0.0038,
		0.0034,
		0.003,
		0.0026,
		0.0022,
		0.0018,
		0.0014,
		0.001,
		0.0006,
		0.0002,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0
};

float table_rightwall[table_num_wall] = {
		0.18,
		0.12,
		0.074,
		0.06,
		0.05,
		0.043,
		0.037,
		0.032,
		0.0285,
		0.026,
		0.0235,
		0.0215,
		0.02,
		0.0185,
		0.017,
		0.0155,
		0.014,
		0.013,
		0.012,
		0.0114,
		0.0108,
		0.0102,
		0.0096,
		0.009,
		0.0084,
		0.0078,
		0.0072,
		0.0066,
		0.006,
		0.0054,
		0.0048,
		0.0042,
		0.0038,
		0.0034,
		0.003,
		0.0026,
		0.0022,
		0.0018,
		0.0014,
		0.001,
		0.0006,
		0.0002,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0
};

float table_front_right_wall[table_num_wall] = {
		0.3,
		0.2,
		0.13,
		0.095,
		0.08,
		0.07,
		0.062,
		0.054,
		0.05,
		0.046,
		0.043,
		0.04,
		0.0375,
		0.035,
		0.033,
		0.032,
		0.0305,
		0.029,
		0.028,
		0.027,
		0.026,
		0.025,
		0.024,
		0.023,
		0.022,
		0.021,
		0.02,
		0.0195,
		0.019,
		0.0185,
		0.018,
		0.0175,
		0.017,
		0.0165,
		0.016,
		0.0155,
		0.015,
		0.0145,
		0.014,
		0.0135,
		0.0132,
		0.0129,
		0.0126,
		0.0123,
		0.012,
		0.0117,
		0.0114,
		0.0111,
		0.0108,
		0.0105,
		0.0102,
		0.0094,
		0.0086,
		0.0078,
		0.0052,
		0.0026,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0
};

float table_front_left_wall[table_num_wall] = {
		0.3,
		0.13,
		0.1,
		0.077,
		0.068,
		0.06,
		0.054,
		0.05,
		0.046,
		0.043,
		0.04,
		0.0375,
		0.035,
		0.033,
		0.032,
		0.0305,
		0.029,
		0.028,
		0.027,
		0.026,
		0.025,
		0.024,
		0.023,
		0.022,
		0.021,
		0.02,
		0.019,
		0.0185,
		0.018,
		0.0175,
		0.017,
		0.0165,
		0.016,
		0.0155,
		0.015,
		0.0145,
		0.014,
		0.0135,
		0.013,
		0.0125,
		0.012,
		0.0115,
		0.011,
		0.0105,
		0.01,
		0.0095,
		0.009,
		0.0085,
		0.008,
		0.0074,
		0.0068,
		0.0062,
		0.0052,
		0.0035,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0
};

//機能	: 横壁センサ値をルックアップテーブルにより距離に変換する
//引数	: 横壁センサ値
//返り値	: 距離
float get_wall_dis_table ( int16_t sensor_value, uint8_t dir )
{
    int16_t table_int = 0;//テーブルの整数部
    int16_t table_deci = 0;//テーブルの少数部
    float table_tilt = 0;//テーブル間の傾き
    float wall_dis = 0; //距離変換結果

    table_int = sensor_value / table_step_wall;

    switch	(dir){

		case front_left :
			//AD結果がMAPの最小値より低い場合は、MAPの最長距離を返す
		    if( table_int < 0 ){
		        wall_dis = table_front_left_wall[0];
		    }
		    //MAPの最大値より大きい場合は、0を返す
		    else if (table_int >=table_num_wall -1 ){
		        wall_dis = 0;
		    }
		    //MAPの範囲内であれば、MAPに従い値を返す
		    else
		    {
		    	//少数部を計算
		        table_deci = sensor_value % table_step_wall;
		        table_tilt = (table_front_left_wall[table_int+1]-table_front_left_wall[table_int])/table_step_wall;
		        wall_dis = table_front_left_wall[table_int] + (float)table_deci * table_tilt;
		    }
			break;

		case side_left :
			//AD結果がMAPの最小値より低い場合は、MAPの最長距離を返す
		    if( table_int < 0 ){
		        wall_dis = table_leftwall[0];
		    }
		    //MAPの最大値より大きい場合は、0を返す
		    else if (table_int >=table_num_wall -1 ){
		        wall_dis = 0;
		    }
		    //MAPの範囲内であれば、MAPに従い値を返す
		    else
		    {
		    	//少数部を計算
		        table_deci = sensor_value % table_step_wall;
		        table_tilt = (table_leftwall[table_int+1]-table_leftwall[table_int])/table_step_wall;
		        wall_dis = table_leftwall[table_int] + (float)table_deci * table_tilt;
		    }
			break;

		case side_right :
			//AD結果がMAPの最小値より低い場合は、MAPの最長距離を返す
		    if( table_int < 0 ){
		        wall_dis = table_rightwall[0];
		    }
		    //MAPの最大値より大きい場合は、0を返す
		    else if (table_int >=table_num_wall -1 ){
		        wall_dis = 0;
		    }
		    //MAPの範囲内であれば、MAPに従い値を返す
		    else
		    {
		    	//少数部を計算
		        table_deci = sensor_value % table_step_wall;
		        table_tilt = (table_rightwall[table_int+1]-table_rightwall[table_int])/table_step_wall;
		        wall_dis = table_rightwall[table_int] + (float)table_deci * table_tilt;
		    }
			break;

		case front_right :
			//AD結果がMAPの最小値より低い場合は、MAPの最長距離を返す
		    if( table_int < 0 ){
		        wall_dis = table_front_right_wall[0];
		    }
		    //MAPの最大値より大きい場合は、0を返す
		    else if (table_int >=table_num_wall -1 ){
		        wall_dis = 0;
		    }
		    //MAPの範囲内であれば、MAPに従い値を返す
		    else
		    {
		    	//少数部を計算
		        table_deci = sensor_value % table_step_wall;
		        table_tilt = (table_front_right_wall[table_int+1]-table_front_right_wall[table_int])/table_step_wall;
		        wall_dis = table_front_right_wall[table_int] + (float)table_deci * table_tilt;
		    }
			break;
    }

    return wall_dis;
}
