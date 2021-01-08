#ifndef PARAM_H_
#define PARAM_H_

//定数
#define G					(9.80665f)					//重量加速度[m/s^2]
#define PI					(3.1415926f)				//円周率
#define SQRT2				(1.41421356237f)			//ルート2
#define SQRT3				(1.73205080757f)			//ルート3
#define SQRT5				(2.2360679775f)				//ルート5
#define SQRT7				(2.64575131106f)			//ルート7
#define dt                  (0.001)
#define dt2                 (0.000001)
#define dt3                 (0.000000001)
#define dt4                 (0.000000000001)

//マクロ関数
#define DEG2RAD(x)			(((x)/180.0f)*PI)			                //度数法からラジアンに変換
#define RAD2DEG(x)			(180.0f*((x)/PI))			                //ラジアンから度数法に変換
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))	//aとbの入れ替え
#define ABS(x) 				((x) < 0 ? -(x) : (x))		                // 絶対値
#define SIGN(x)				((x) < 0 ? -1 : 1)			                // 符号
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		                // 2つのうち大きい方を返します
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		                // 2つのうち小さい方を返します
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))


/////////////////////////////////////
//           パラメータ             //
/////////////////////////////////////
/*common*/
#define Sampling_cycle      (0.001)  //サンプリング周期

/*module_test*/
#define log_count_lim		(1000)	//データ取得する期間[ms]
#define log_count_step		(1)	    //データを取得する時間間隔[ms]

#define DATA_DEFAULT              //測定モード１
//#define DATA_SIDE                 //測定モード２
//#define DATA_MAZE                 //測定モード３

//maze
#define x_size              (10)     //x軸方向の壁(縦壁)の枚数+1
#define y_size              (10)     //y軸方向の壁(横壁)の枚数+1
#define g_size              (4)     //ゴールのマスの数
#define goal_cordinate      {4, 4, 5, 5, 3, 3, 4, 4, 4,\
                             	 	 	 	  4, 5, 4, 5, 2, 3, 1, 2, 3}
										//一行目、ゴールのx座標
										//二行目、ゴールのy座標
//ir
#define front_th            (300) //前壁有無判定の閾値
#define right_th            (300) //右壁有無判定の閾値
#define left_th             (400) //左壁有無判定の閾値

//mode
#define mode_count_up_th 	(0.10f)     //モードカウントアップの右タイヤ速度閾値
#define mode_count_down_th 	(-0.10f)    //モードカウントダウンの右タイヤ速度閾値
#define mode_stanby_th 		(0.10f)     //モードカウントアップの左タイヤ速度閾値

//mouse_state
#define ave_num 			(5)	    //速度の移動平均フィルタの長さ
#define cut_off_w           (400)       //角速度のカットオフ周波数[Hz]
#define Tire_diameter		(0.013084040316358f)//タイヤの直径

//kalman
#define IMU_X_dispersion     (0.50045771)         //IMUのX方向加速度分散値の二乗
#define encorder_dispersion  (0.000184912)        //エンコーダの速度分散値の二乗


//target
//探索パラメータ
#define search_move_accel          (1.5f)      //移動加速度[m/ss]
//#define search_rotate_accel         (25*PI)      //角加速度[rad/ss]
//#define search_move_speed_max      (0.2f)      //最大移動速度[m/s]
//#define search_rotate_speed_max     (3*PI)      //最大角速度[rad/s]
#define search_rotate_accel         (80*PI)      //角加速度[rad/ss]
#define search_move_speed_max      (0.3f)      //最大移動速度[m/s]
#define search_rotate_speed_max     (4*PI)      //最大角速度[rad/s]
#define search_move_speed_slow     (0.03)      //スロー走行時の速度[m/s]
#define move_speed_slow (0.01)						//停止前速度

//直進
#define straight_move_accel          (2.5f)      //移動加速度[m/ss]
#define straight_rotate_accel         (80*PI)      //角加速度[rad/ss]
#define straight_move_speed_max      (1.0f)      //最大移動速度[m/s]
#define straight_rotate_speed_max     (4*PI)      //最大角速度[rad/s]
#define straight_move_speed_slow     (0.03)      //スロー走行時の速度[m/s]


//control
//FF制御
#define ff_rate_w             (1.0)       //角速度FF制御の割合
#define ff_gain_a_w           (0.005)      //角加速度に対するゲイン
#define ff_gain_v_w           (0.043)     //角速度に対するゲイン

#define ff_rate_m (1.0)
#define ff_gain_a (0.25)
#define ff_gain_v (1.05)
#define ff_gain_f (0.2)

//FB制御
#define fb_rate_m (1.0)
#define fb_rate_w (1.0)

#define move_speed_P		(5.0f)	//移動速度制御のPゲイン
#define move_speed_I		(50.0f)	//移動速度制御のIゲイン
#define rotate_speed_P		(0.5f)	//角速度制御のPゲイン
#define rotate_speed_I		(20.0f)	//角速度制御のIゲイン

//movement
#define move_comp_th        (0.001f)   //移動完了の閾値
#define rotate_comp_th      (0.001f)    //回転完了の閾値

///slalom

//#define slalom_front_wall_adj (0.0545) //探索90における前距離補正時の前壁距離
#define slalom_front_wall_adj (0.04) //探索90における前距離補正時の前壁距離

//90度時計周り
#define slalom_clk_90_before_offset (0.011)      //前オフセット距離
#define slalom_clk_90_after_offset (0.011)       //後オフセット距離
#define slalom_clk_90_offset    (0.0573)          //ターン中の並進距離
//90度反時計回り
#define slalom_conclk_90_before_offset (0.011)      //前オフセット距離
#define slalom_conclk_90_after_offset (0.011)       //後オフセット距離
#define slalom_conclk_90_offset    (0.0573)          //ターン中の並進距離

//探索90
#define slalom_s90_v_1 (0.3)
#define slalom_s90_w_1 (4*PI)
#define slalom_s90_wa_1 (80*PI)
#define slalom_s90_bo_1 (0.0133)
#define slalom_s90_ao_1 (0.0133)
//斜め45
#define slalom_45_v_1 (0.5)
#define slalom_45_w_1 (4*PI)
#define slalom_45_wa_1 (80*PI)
#define slalom_45_bo_1 (0.016)
#define slalom_45_ao_1 (0.034)
//大廻90
#define slalom_90_v_1 (0.5)
#define slalom_90_w_1 (3*PI)
#define slalom_90_wa_1 (80*PI)
#define slalom_90_bo_1 (0.02725)
#define slalom_90_ao_1 (0.02725)
//V90
#define slalom_v90_v_1 (0.5)
#define slalom_v90_w_1 (5*PI)
#define slalom_v90_wa_1 (80*PI)
#define slalom_v90_bo_1 (0.0152)
#define slalom_v90_ao_1 (0.0152)
//斜め135
#define slalom_135_v_1 (0.5)
#define slalom_135_w_1 (4*PI)
#define slalom_135_wa_1 (80*PI)
#define slalom_135_bo_1 (0.025)
#define slalom_135_ao_1 (0.0175)
//大廻90
#define slalom_180_v_1 (0.5)
#define slalom_180_w_1 (3.61*PI)
#define slalom_180_wa_1 (60*PI)
#define slalom_180_bo_1 (0.031)
#define slalom_180_ao_1 (0.031)

//adjust
#define front_sensor_r_ref      (0.01f)	        //前壁補正時の右前距離目標値(1cm)
#define front_sensor_l_ref      (0.007f)	        //前壁補正時の左前距離目標値(1cm)
//#define front_sensor_r_ref      (0.0325f)	        //前壁補正時の右前距離目標値(1cm)
//#define front_sensor_l_ref      (0.0325f)	        //前壁補正時の左前距離目標値(1cm)
#define chassis_width           (0.036787f)         //シャシー幅
#define front_sensor_move_KP	(50.412292054546f)	//前壁距離のPゲイン
#define front_sensor_move_KI	(26.2444668681538f)	//前壁距離のIゲイン
//#define front_sensor_move_KI	(1000.0f)	//前壁距離のIゲイン
#define front_sensor_move_KD	(3.28431998107001f)	//前壁距離のDゲイン
#define front_sensor_move_fil	(23.9567963129851f)	//前壁距離フィルタ係数
//#define front_sensor_rotate_KP	(6.40482660641506f)	//前壁角度のPゲイン
#define front_sensor_rotate_KP	(3.0f)	//前壁角度のPゲイン

#define front_sensor_rotate_KI	(2.52037335129739f)	//前壁角度のIゲイン
#define front_sensor_rotate_KD	(0.206728439324594f)//前壁補正のDゲイン
#define front_sensor_rotate_fil	(44.5570260812328f)	//前壁角度フィルタ係数
#define front_sensor_th     	(0.0002)			//前壁補正時の補正閾値(0.3mm)
#define calib_tim               (200)				//前壁補正時間(100ms)
#define calib_tim_lim           (500)              //前壁補正最大時間

#define side_sensor_th          (0.025)               //横壁センサ値の閾値[m]
#define side_sensor_th_add      (0.020)               //横壁センサ値の閾値変化量[m]
#define side_sensor_diff_th     (0.0001)           //横壁センサ変化量の閾値
#define side_wall_P             (150.0)               //横壁制御　Pゲイン
#define side_wall_D             (3)                //横壁制御　Dゲイン

#define ir_diff_ave_num         (3)                //IRセンサの変動値の移動平均期間

#define wall_break_calib_right_th (0.029) 	//壁切れ補正の右閾値
#define wall_break_calib_left_th (0.032)		//壁切れ補正の左閾値
#define wall_break_calib_margin (0.01)		//壁切れ補正の猶予
#define wall_break_calib_ref_dis (0.0408)   //壁切れ位置と車体の位置の差


//imu
#define REFFERENCE_NUM		(1000)		//何回の平均をもってジャイロのリファレンス電圧とするか
#define GYRO_Z_SIGN			(-1.f)		//ジャイロの出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define GYRO_Z_SENSITIVITY	(16.8f)	
#define ACCEL_X_SIGN		(1.f)		//加速度計の出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define ACCEL_X_SENSITIVITY	(4096.f)

//encorder
#define pinion_gear_num		(9.0f)								//ピニオンギヤ数
#define wheel_gear_num		(37.0f)								//ホイールギア数
#define gear_rate			(wheel_gear_num/pinion_gear_num)	//変速比（ギア比）

//exvol
#define rate_side_wall      (0.6)                //壁制御時の壁制御による印加電圧の割合
#define side_wall_cont_speed_th (0.1)             //壁制御時の最低速度

//motor
#define MOT_DUTY_MIN	(30)						//モータの最低Duty
#define MOT_DUTY_MAX	(800)						//モータの最大Duty

//failsafe
#define speed_m_err_th (0.025f)					//速度偏差の閾値(m/s)
#define theta_err_th (0.1f)							//角度偏差の閾値(rad)
#define mouse_state_err_count_th 	(50)		//速度、角度偏差によるエラー検出のカウンタ値(ms)
#define front_wall_err_count_th 	(1000)		//前壁制御時のエラー検出のカウンタ値(ms)



#endif /* PARAM_H_*/
