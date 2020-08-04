/*
 * module_test.c
 *
 *  Created on: Aug 13, 2019
 *      Author: 岡田 泰裕
 */

#include "index.h"

static uint16_t log_counter = 0 ; //ログ取得開始からの時間監視用カウンタ[ms]

typedef struct {	//データ格納用構造体の定義
	float		time;				//測定開始からの時間[m/s]
	float		interrupt_duty;	//割り込み割合
	float 	V_battery;		//バッテリー電圧
	float		speed_r;			//	右タイヤ速度[m/s]
	float		speed_l;			//左タイヤ速度[m/s]
	float		angle_r;			//右タイヤ角度
	float		angle_l;			//左タイヤ角度
	float		correct_r;		//右タイヤ補正角度
	float		correct_l;			//左タイヤ補正角度
	float		target_speed_m;	 //目標並進速度
	float		speed_m;		//並進方向の速度[m/s]
	float		ideal_d_m;		//並進方向理想移動距離[m]
	float	 	real_d_m;			//並進方向移動距離[m]
	float 	accel_m;			//並進方向の加速度(m/s)
	float		target_speed_w;	//目標角速度
	float		speed_w;			//実際の角速度(rad/s
	float		ideal_d_w;		//理想角度
	float		real_d_w;		//回転角度(rad)
	float		duty_r;				//右モータ操作量（duty)
	float   	duty_l;				//左モータ操作量	(duty)
	float   	front_sensor_r;		//右前壁センサ値
	float   	front_sensor_l;		//左前壁センサ値
	float   	side_sensor_r;		//右横壁センサ値
	float   	side_sensor_l;		//左横壁センサ値
} log_struct;

static log_struct log_store[log_count_lim / log_count_step];  // データ格納用の構造体
/* ---------------------------------------------------------------
	ログ取得カウンタ初期化関数
--------------------------------------------------------------- */
void log_init (void)
{
	log_counter = 0; //ログカウンタの初期化
//	//データ名欄記述、カンマで区切る
//	SEGGER_RTT_printf(0, "Battery_Voltage[V],Duty_R[%],Dyty_L[%],velocity[m/s],distance[m],angle_velocity[rad/s],angle[rad]\r\n");
}
/* ---------------------------------------------------------------
	ログ取得用関数（1msタスク）
--------------------------------------------------------------- */
void data_get (void)
{

	uint16_t i = 0;
	i = log_counter / log_count_step;

	if(log_counter < log_count_lim)
	{
		if( (log_counter % log_count_step) == 0)
		{
			log_store[i].time = (float)log_counter;
			log_store[i].interrupt_duty = (float)Interrupt_GetDuty();
			log_store[i].V_battery =(float)Battery_GetVoltage();
			log_store[i].duty_r = (float)get_duty_r();
			log_store[i].duty_l = (float)get_duty_l();
			log_store[i].speed_r = (float)get_tire_r_speed();
			log_store[i].speed_l = (float)get_tire_l_speed();
			log_store[i].angle_r = (float)get_tire_r_angle();
			log_store[i].angle_l = (float)get_tire_l_angle();
			log_store[i].correct_r = (float)encoder_correct_angle_r(get_tire_r_angle());
			log_store[i].correct_l = (float)encoder_correct_angle_l(get_tire_l_angle());
			log_store[i].target_speed_m = (float)get_target_move_speed();
			log_store[i].speed_m = (float)get_move_speed_ave();
			log_store[i].ideal_d_m = (float)get_ideal_length();
			log_store[i].real_d_m = (float)get_move_length();
			log_store[i].accel_m = (float)IMU_GetAccel_X();
			log_store[i].target_speed_w = (float)get_target_rotation_speed();
			log_store[i].speed_w = (float)IMU_GetGyro_Z();
			log_store[i].ideal_d_w = (float)get_ideal_angle();
			log_store[i].real_d_w = (float)get_rotation_angle();
			log_store[i].front_sensor_r = (float)get_wall_dis_table(Sensor_GetValue(front_right), front_right);
			log_store[i].front_sensor_l = (float)get_wall_dis_table(Sensor_GetValue(front_left), front_left);
			log_store[i].side_sensor_r = (float)get_wall_dis_table(Sensor_GetValue(side_right), side_right);
			log_store[i].side_sensor_l = (float)get_wall_dis_table(Sensor_GetValue(side_left), side_left);
//			log_store[i].front_sensor_r = (float)(Sensor_GetValue(front_right));
//			log_store[i].front_sensor_l = (float)(Sensor_GetValue(front_left));
//			log_store[i].side_sensor_r = (float)(Sensor_GetValue(side_right));
//			log_store[i].side_sensor_l = (float)(Sensor_GetValue(side_left));
		}
		log_counter += 1; //logカウンタ更新
	}
}
/* ---------------------------------------------------------------
	ログ吐き出し用関数
--------------------------------------------------------------- */
void data_read(void)
{
	uint16_t i = 0;
	uint16_t j = 0;

	j =  log_count_lim / log_count_step - 1 ;

	//printfで一行づつ書き出していく。
	//一行目はパラメータ名
//	SEGGER_RTT_printf(0, "Time[ms],BV[V],Duty_R[%%],Dyty_L[%%],rt_v[m/s],lt_v[m/s],rt_an[],lt_an[],rt_c_an[],lt_c_an[],"
//			"v[m/s],distance[m],accel_m[m/ss],angle_velocity[rad/s],angle[rad],IR_SL,IR_FL,IR_FR,IR_SR\r\n") ;	//パラメータ名を記述

	printf( "Time[ms],BV[V],int_duty,Duty_R[%%],Dyty_L[%%],rt_v[m/s],lt_v[m/s],rt_an[],lt_an[],rt_c_an[],lt_c_an[],"
			"target_v,v[m/s],ideal_d,distance[m],accel_m[m/ss],target_w,w[rad/s],ideal_angle,angle[rad],IR_SL,IR_FL,IR_FR,IR_SR\r\n") ;	//パラメータ名を記述



	//パラメータ名が出力されたのち、ボタンを押してデータを吐き出す
	while(!read_button()){
		HAL_Delay(1);
	}

	for(i = 0; i <= j ; i++)
	{
		//以下出力形式を整えて記述
//		  snprintf(temp_str,300,"%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.7f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\r\n",
		  printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
					log_store[i].time,
					log_store[i].V_battery,
					log_store[i].interrupt_duty,
					log_store[i].duty_r,
					log_store[i].duty_l,
					log_store[i].speed_r,
					log_store[i].speed_l,
					log_store[i].angle_r,
					log_store[i].angle_l,
					log_store[i].correct_r,
					log_store[i].correct_l,
					log_store[i].target_speed_m,
					log_store[i].speed_m,
					log_store[i].ideal_d_m,
					log_store[i].real_d_m,
					log_store[i].accel_m,
					log_store[i].target_speed_w,
					log_store[i].speed_w,
					log_store[i].ideal_d_w,
					log_store[i].real_d_w,
					log_store[i].front_sensor_r ,
					log_store[i].front_sensor_l ,
					log_store[i].side_sensor_r ,
					log_store[i].side_sensor_l );
////		  SEGGER_RTT_WriteString(0,temp_str);
//		  HAL_Delay(50);
//		printf("%f,",log_store[i].time);
//		printf("%f,",log_store[i].interrupt_duty);
//		printf("%f,",log_store[i].V_battery);
//	    printf("%f,",log_store[i].duty_r);
//	    printf("%f,",log_store[i].duty_l);
//	    printf("%f,",log_store[i].speed_r);
//	    printf("%f,",log_store[i].speed_l);
//	    printf("%f,",log_store[i].angle_r);
//	    printf("%f,",log_store[i].angle_l);
//	    printf("%f,",log_store[i].correct_r);
//	    printf("%f,",log_store[i].correct_l);
//	    printf("%f,",log_store[i].target_speed_m);
//	    printf("%f,",log_store[i].speed_m);
//	    printf("%f,",log_store[i].ideal_d_m);
//	    printf("%f,",log_store[i].real_d_m);
//	    printf("%f,",log_store[i].accel_m);
//	    printf("%f,",log_store[i].target_speed_w);
//	    printf("%f,",log_store[i].speed_w);
//	    printf("%f,",log_store[i].ideal_d_w);
//	    printf("%f,",log_store[i].real_d_w);
//	    printf("%f,",log_store[i].front_sensor_r);
//	    printf("%f,",log_store[i].front_sensor_l);
//	    printf("%f,",log_store[i].side_sensor_r);
//	    printf("%f,",log_store[i].side_sensor_l);
//	    printf("\r\n"); // 改行
	    HAL_Delay(50);
	}
}

/* ---------------------------------------------------------------
	有線でのパラメータ確認関数
--------------------------------------------------------------- */
/* ---------------------------------------------------------------
	各機能の動作確認用関数
--------------------------------------------------------------- */
void module_test( void )
{
	char temp_str[500];

	while( 1 ) {

		  snprintf(temp_str,500,"<Boot Time> %8.3f[s]\r\n"
				  "<Interrupt> %3.1f[%%] (MAX : %3.1f[%%])\r\n,"
				  "<Battery> %3.2f[V]\r\n"
				  "<IR Sensor> Front_R: %4d, Side_R: %4d, Side_L: %4d, Front_L: %4d\r\n"
				  "<IR Sensor Map> Front_R: %6.5f, Side_R: %6.5f, Side_L: %6.5f, Front_L: %6.5f\r\n"
				  "<IMU> Accel_X: %5.3f[m/s^2], Gyro_Z: %6.3f[rad/s]\r\n,"
				  "<Encoder> L: %5.1d[deg],  R: %5.1d[deg]\r\n",

				  Interrupt_GetBootTime(),
				  (float)Interrupt_GetDuty()/10.f, (float)Interrupt_GetDuty_Max()/10.f,
				  Battery_GetVoltage(),
				  Sensor_GetValue(front_right), Sensor_GetValue(side_right), Sensor_GetValue(side_left), Sensor_GetValue(front_left),
				  get_wall_dis_table(Sensor_GetValue(front_right), front_right),get_wall_dis_table(Sensor_GetValue(side_right), side_right),
				  	  get_wall_dis_table(Sensor_GetValue(side_left), side_left),get_wall_dis_table(Sensor_GetValue(front_left), front_left),
				  IMU_GetAccel_X(), IMU_GetGyro_Z(),
				  encoder_Getangle_r(),encoder_Getangle_l());

				  SEGGER_RTT_WriteString(0,temp_str);

				  HAL_Delay(500);
	}
}

