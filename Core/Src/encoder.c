

#include <stdio.h>
#include "main.h"
#include "spi.h"
#include "param.h"
#include "SEGGER_RTT.h"
#include "encoder.h"

static uint8_t encoder_l_data[2];		//左エンコーダーからの生データ
static uint8_t encoder_r_data[2];		//右エンコーダーからの生データ

static uint16_t encoder_l_value;	//左エンコーダの値
static uint16_t encoder_r_value;	//右エンコーダの値

//右エンコーダの補正値
int16_t encoder_r_tablel[table_num_encoder] = {
		0,
		-100,
		-200,
		-250,
		-220,
		-200,
		-180,
		-180,
		-160,
		-70,
		70,
		210,
		450,
		620,
		750,
		830,
		830,
		800,
		800,
		780,
		730,
		600,
		470,
		350,
		300,
		250,
		230,
		210,
		200,
		180,
		150,
		70,
		0,
};

//左エンコーダの補正値
int16_t encoder_l_tablel[table_num_encoder] = {
		0,
		-80,
		-130,
		-190,
		-200,
		-250,
		-305,
		-240,
		-150,
		-80,
		-30,
		90,
		100,
		30,
		-80,
		-160,
		-400,
		-610,
		-820,
		-940,
		-1150,
		-1250,
		-1270,
		-1180,
		-1050,
		-820,
		-600,
		-400,
		-200,
		-70,
		50,
		60,
		0
};

/* ---------------------------------------------------------------
	左エンコーダの角度を取得する関数[uint16_tでカウンタ値を返す。]
--------------------------------------------------------------- */
uint16_t encoder_Getangle_l( void )
{
	uint8_t dummy_data[2] = {0,0};

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);//CSpinをLowに、SPI通信開始
	//HAL_SPI_Receive(&hspi3,encoder_l_data,2,10000);
	HAL_SPI_TransmitReceive(&hspi3, dummy_data, encoder_l_data, 2, 10000);
	encoder_l_value=((uint16_t)encoder_l_data[0]<< 8)|((uint16_t)encoder_l_data[1]);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);//CSpinをHiに、SPI通信停止
	return encoder_l_value;
}

/* ---------------------------------------------------------------
	右エンコーダの角度を取得する関数[uint16_tでカウンタ値を返す。]
--------------------------------------------------------------- */
uint16_t encoder_Getangle_r( void )
{
	uint8_t dummy_data[2] = {0,0};

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);//CSpinをLowに、SPI通信開始
	//HAL_SPI_Receive(&hspi1,encoder_r_data,2,10000);
	HAL_SPI_TransmitReceive(&hspi1, dummy_data, encoder_r_data, 2, 10000);
	encoder_r_value=((uint16_t)encoder_r_data[0]<< 8)|((uint16_t)encoder_r_data[1]);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);//CSpinをHiに、SPI通信停止
	return encoder_r_value;
}

/* ---------------------------------------------------------------
	左エンコーダの角度を補正する関数[uint16_tで補正後のカウンタ値を返す。]
--------------------------------------------------------------- */
uint16_t encoder_correct_angle_l( uint16_t encoder_l_orig )
{
    uint16_t table_int = 0;//テーブルの整数部
    uint16_t table_deci = 0;//テーブルの少数部
    float table_tilt = 0;//テーブル間の傾き
    int16_t encoder_diff_l = 0; //エンコーダの補正量
    uint16_t encoder_correct_l = 0; //エンコーダの補正結果

    table_int = encoder_l_orig / table_step_encoder;
    table_deci = encoder_l_orig % table_step_encoder;
    table_tilt = (encoder_l_tablel[table_int+1]-encoder_l_tablel[table_int])/(float)table_step_encoder;

    encoder_diff_l =  (int16_t)(encoder_l_tablel[table_int] + table_tilt*table_deci);
    encoder_correct_l = encoder_l_orig - encoder_diff_l;

    return encoder_correct_l;

}

/* ---------------------------------------------------------------
	左エンコーダの角度を補正する関数[uint16_tで補正後のカウンタ値を返す。]
--------------------------------------------------------------- */
uint16_t encoder_correct_angle_r( uint16_t encoder_r_orig )
{
    uint16_t table_int = 0;//テーブルの整数部
    uint16_t table_deci = 0;//テーブルの少数部
    float table_tilt = 0;//テーブル間の傾き
    int16_t encoder_diff_r = 0; //エンコーダの補正量
    uint16_t encoder_correct_r = 0; //エンコーダの補正結果

    table_int = encoder_r_orig / table_step_encoder;
    table_deci = encoder_r_orig % table_step_encoder;
    table_tilt = (encoder_r_tablel[table_int+1]-encoder_r_tablel[table_int])/(float)table_step_encoder;

    encoder_diff_r =  (int16_t)(encoder_r_tablel[table_int] + table_tilt*table_deci);
    encoder_correct_r = encoder_r_orig - encoder_diff_r;

    return encoder_correct_r;

}
