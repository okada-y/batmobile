#include "main.h"
#include "tim.h"
#include "adc.h"
#include "ir_sensor.h"
#include "SEGGER_RTT.h"
#include "stm32f4xx_hal.h"


typedef struct {	// センサ情報用構造体
	uint16_t	off;		// LEDオフ時の値
	uint16_t	on;			// LEDオン時の値
} t_sensor_value;

uint8_t			sensor_mode;		// センシングパターン

static uint16_t			adc_value[5];		// AD変換値
static uint16_t			battery_value;		// a バッテリ電圧の生データ
static t_sensor_value	sensor_value[4];	// IRセンサの生データ

uint8_t mode_number_int = 0; //m モードごとの割り込み用フラグ　（0は使わないこと

/* ---------------------------------------------------------------
	赤外センサの初期設定関数
--------------------------------------------------------------- */
void Sensor_Initialize( void )
{
	//センサ値をDMA開始
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, sizeof(adc_value)/sizeof(uint16_t));
	//IRセンサの点灯パターン制御用タイマの起動
	Sensor_StartADC();
}


/* ---------------------------------------------------------------
	赤外センサによる計測を開始する関数
--------------------------------------------------------------- */
void Sensor_StartADC( void )
{

	sensor_mode = sensor_all_off_mode;

	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim1,0);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);

	__HAL_TIM_ENABLE(&htim1);

}

/* ---------------------------------------------------------------
	赤外センサによる計測を停止する関数
--------------------------------------------------------------- */
void Sensor_StopADC( void )
{
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
	SENSOR_ALL_OFF();
}


void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc )
{
//	printf("DMA_complete\r\n");//デバッグ用

//	ADC1->SR &= ~ADC_SR_EOC_Msk;//ADC完了フラグをクリア

	/* 割り込み処理を書く */
	switch (sensor_mode) {
	case sensor_all_off_mode:
		battery_value 					= adc_value[0];
		sensor_value[front_left].off 	= adc_value[1];
		sensor_value[side_left].off		= adc_value[2];
		sensor_value[side_right].off 	= adc_value[3];
		sensor_value[front_right].off 	= adc_value[4];
		//HAL_GPIO_WritePin(GPIOA,LED2_Pin,GPIO_PIN_SET);
	break;

	case sensor_front_on_mode:
		sensor_value[front_left].on 	= adc_value[1];
		sensor_value[front_right].on 	= adc_value[4];
		//HAL_GPIO_WritePin(GPIOA,LED3_Pin,GPIO_PIN_SET);
	break;

	case sensor_side_on_mode:
		sensor_value[side_left].on		= adc_value[2];
		sensor_value[side_right].on 	= adc_value[3];
		//HAL_GPIO_WritePin(GPIOA,LED4_Pin,GPIO_PIN_SET);
	break;

	}
    //センサモードのインクリメント
    sensor_mode = (sensor_mode + 1 ) %3;
	//HAL_GPIO_WritePin(GPIOA,LED2_Pin,GPIO_PIN_SET);

}


/* ---------------------------------------------------------------
	バッテリのAD値を取得する関数
--------------------------------------------------------------- */
uint16_t Sensor_GetBatteryValue( void )
{
	return( battery_value );
}

/* ---------------------------------------------------------------
	赤外センサの偏差値を取得する関数
--------------------------------------------------------------- */
int16_t Sensor_GetValue( uint8_t dir )
{
	if( 0 <= dir || dir <= 3 ) {
		return sensor_value[dir].on - sensor_value[dir].off;
	} else {
		return -1;
	}
}



//機能	: IRセンサ値を距離に変換する
//引数	: 変換するセンサ番号(0:左前,1:左横,2:右横,3:右前)
//返り値	: 距離[m]
double SensorValue2length( uint8_t dir )
{
	double length_tmp = 0;
	double sensor_tmp = (double)Sensor_GetValue(dir);
	double a = 0;
	double b = 0;

	//前センサ
	if ((dir == 0) || (dir == 3))
	{
		if(sensor_tmp <= 100){
			length_tmp = 0.09;
		}
		else{
			//左前
			if(dir == 0){
				a = 0.7;
				b = 0.0847;
			}
			//右前
			if(dir == 3){
				a = 0.7;
				b = 0.082;
				}
			//線形近似　a/ln(AD) -b
			length_tmp = a / log(sensor_tmp) - b ;
		}
	}
	//横センサ	
	else if((dir == 1) || (dir == 2)){
		if(sensor_tmp <= 100){
			length_tmp = 0.050;
		}
		else{
			a = 0.015;
			b = 0.027;
			//線形近似　0.09-(a*ln(AD)-b)
			length_tmp = 0.09-(a*log(sensor_tmp)-b);
		}		

	}

	//リミット処理
	if(length_tmp < 0) 
	{
		length_tmp = 0;
	}

	return length_tmp;

}



/* ---------------------------------------------------------------
	デバッグ用
--------------------------------------------------------------- */
void Sensor_DebugPrintf( void )
{
	printf( "%5d (%4d - %3d), %5d (%4d - %3d), %5d (%4d - %3d), %5d (%4d - %3d)\r\n",
			sensor_value[3].on - sensor_value[3].off, sensor_value[3].on, sensor_value[3].off,
			sensor_value[2].on - sensor_value[2].off, sensor_value[2].on, sensor_value[2].off,
			sensor_value[1].on - sensor_value[1].off, sensor_value[1].on, sensor_value[1].off,
			sensor_value[0].on - sensor_value[0].off, sensor_value[0].on, sensor_value[0].off );

}





