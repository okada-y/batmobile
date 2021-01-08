#include "index.h"
#include "matlab_IR_sensor.h"

/*matlab上でのIRセンサ値取得関数の定義
 *ハンドにてC側で記述したIRセンサ値取得関数を各関数から呼び出すこと
 */

uint16_t wall_sensor_front_th = front_th;
uint16_t wall_sensor_right_th = right_th;
uint16_t wall_sensor_left_th = left_th;

// 前方センサ値取得
int m_get_front_sensor(void)
{
    //c側で記述したセンサ値取得関数を呼び出すこと
	return (Sensor_GetValue(front_left)+Sensor_GetValue(front_right))/2;
//	return (Sensor_GetValue(front_right));
}


// 右センサ値取得
int m_get_right_sensor(void)
{
    //c側で記述したセンサ値取得関数を呼び出すこと
	return Sensor_GetValue(side_right);
}


// 左センサ値取得
int m_get_left_sensor(void)
{
	//壁センサ値読み込み回数をインクリメント
	inc_times_wall_sensor();
    //c側で記述したセンサ値取得関数を呼び出すこと
	return Sensor_GetValue(side_left);
}
