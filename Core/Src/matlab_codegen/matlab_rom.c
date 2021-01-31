#include "matlab_rom.h"
#include "index.h"
/*matlab��ł̓���֐��̒�`
 *�n���h�ɂ�C���ŋL�q��������֐����e�֐�����Ăяo������
 */


//@param ���H�Ǐ��
void m_rom_write(void)
{
	//割り込み停止
	 HAL_TIM_Base_Stop_IT(&htim6);
	 //ジャイロ更新停止
	 IMU_refresh_flg = 0;
    //C���Ŗ��H�f�[�^�L�^���������邱�ƁB
	  //ADCのDMA停止
	  Sensor_StopADC();
	  //現在の迷路情報をromに書き込み
	  writeFlash(start_address, (uint8_t*)&maze_data, sizeof(maze_data_t));
	  //ADCのDMA再開
	  Sensor_StartADC();
	  //書き込み完了時、ブザーを鳴らす
	  set_buzzer_flg(2);
	  //ジャイロ初期化
	  IMU_Initialize();
	  //ジャイロリファレンス取得
	  IMU_ResetReference();
	  //マウスをリセット
	  mouse_reset();
	  //割り込み再開
	  HAL_TIM_Base_Start_IT( &htim6 );

}
