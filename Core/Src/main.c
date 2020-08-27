/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"index.h"
#include "maze_init.h"
#include "maze_solve.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	search_mode 	= 0,
 	fust_run_mode 	= 1,
} maze_search_flg;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t temp[2];
int16_t temp1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	char temp_str[200];
	  /*迷路変数定義*/
//	  static uint8_t maze_x_size = x_size;//x方向の壁の枚数(x方向のマスの数+1)
//	  static uint8_t maze_y_size = y_size;//y方向の壁の枚数(y方向のマスの数+1)
//	  static uint8_t goal_size = g_size; //ゴールのマスの数
//	  static uint8_t maze_goal[18] = goal_cordinate;
//	  static uint8_t m_wall_tmp[1024];//迷路情報格納用配列
//	  static uint8_t m_search_tmp[1024];//探索情報格納用配列

	//static uint8_t run_mode = search_mode;
	uint8_t maze_data_mode = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
int16_t i = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_RTT_Init();
  IMU_Initialize();
  TIM2_PWM_START();
  Motor_Initialize();
  Interrupt_Initialize();


  printf("while_start\r\n" );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  	  while(1){
//	  		  	  IMU_Receive();
//	  		  	  IMU_GetGyro_Z();
//	  		  	  IMU_GetAccel_X();
//	  		  	  snprintf(temp_str,200,"%f,%f\r\n",
//	  		  						  IMU_GetGyro_Z(),
//	  		  						  IMU_GetAccel_X());
//	  		  	 SEGGER_RTT_WriteString(0,temp_str);
//	  		  	 HAL_Delay(500);
//	  	  }
	  	  mode_main();//モードの選定、開始処理

	  	  /*ここからモードごとの処理に移行*/
		  switch(get_mode_number()){

		  case 0:
		  	  data_read();
			  break;

		    case 1:
		    	log_init();
		    			set_duty_l(100);
		    			set_duty_r(100);
		    			HAL_Delay(1100);

		    	break;

		    case 2:
		        log_init();
//		 	    set_mode_ctrl(trace);
//		 	    set_target_turn_param(search, 0.72, 0);
//		 	   HAL_Delay(2000);
//		        start_acceleration();
		        half_acceleration();
		        constant_speed();
		        slalom_clock_90(1);
		        constant_speed();
		        for(uint8_t i = 0;i<2;i++){
					constant_speed();
		        }
				half_deceleration();
				HAL_Delay(2000);
		    	break;

		  case 3:
			  	log_init();
				/*回転方向設定*/
			  	set_target_turn_param(search, 0, 0);
				set_rotation_mode(clockwise);
			    set_target_angle(-10*PI*2);

			    /*360度回転するまで待機*/
			    while (1)
			    {
			    	if(rotate_comp_jud())
			    	{
			    		break;
			    	}
			    }
			    HAL_Delay(4000);
			  break;

		  case 4:
				set_mode_ctrl(front_wall); 		//制御モードを前壁補正モードに
				while(1){

				}
			  break;

		  case 5:
			  log_init();
		      set_mode_ctrl(trace);
		      half_acceleration();
		      move_front (already,2,1);
		      move_front (already,0,1);
		      move_front (already,2,1);
		      half_deceleration();
		      HAL_Delay(4000);
			  break;

		  case 6:
		       log_init();
		       set_target_turn_param(search, 0.0, 0.0);
		       while(1){

		       }
			  break;

		  case 7://モジュールテスト
			  module_test();
			  break;

		  case 8://FF用テスト
			  log_init();
			  temp1 = 100;//最高duty
			  		    	for(i = 0.5*temp1;i<temp1;i++){
			  		    			set_duty_l(i);
			  		    			set_duty_r(i);
			  		    			HAL_Delay(10);
			  		    	}
			  		    	HAL_Delay(1000);
			  		    	for(i = temp1;i>0.5*temp1;i--){
			  		    			set_duty_l(i);
			  		    			set_duty_r(i);
			  		    			HAL_Delay(10);
			  		    	}
			  		    	HAL_Delay(1000);
			  		    	for(i = 0.5*temp1;i>0;i--){
			  		    			set_duty_l(i);
			  		    			set_duty_r(i);
			  		    			HAL_Delay(10);
			  		    	}
			  		    	HAL_Delay(3000);
			  break;

		  case 13: //迷路データ初期化
			  maze_init(maze_data.maze_y_size, maze_data.maze_x_size, maze_data.m_wall_tmp, maze_data.m_search_tmp);
			  break;

		  case 14:	//迷路走行

			  //フェイルセーフ有効化
			  set_failsafe_flg(1);
			  //決定されたモードで走行
			  maze_solve(maze_data.m_wall_tmp, maze_data.m_search_tmp, maze_data.maze_y_size, maze_data.maze_x_size,
					  	  maze_data.goal_size,maze_data.maze_goal, run_mode_1,run_mode_2,maze_data.contour_map,maze_data.row_num_node,maze_data.col_num_node);
			  break;

		  case 15:	//迷路データの書き込み、読み込み、消去
			  maze_data_mode = 0;

			  while(!button_state){
				  maze_data_mode = select_num_r_tire (maze_data_mode);
				  Indicator_number(maze_data_mode);
			  }

			  for(int i=0; i<2; i++){ // モード処理終了時、LEDを2回点灯
				  LED_ALL_ON();
				  HAL_Delay(700);
				  LED_ALL_OFF();
				  HAL_Delay(300);
			  }

			  switch(maze_data_mode){
			  case 0://迷路データの出力
				  maze_data_output();
				  break;
			  case 1://読み込み
				  loadFlash(start_address, (uint8_t*)&maze_data, sizeof(maze_data_t));
				  break;
			  case 2://書き込み
				  writeFlash(start_address, (uint8_t*)&maze_data, sizeof(maze_data_t));
				  break;
			  case 3://消去
				  eraseFlash()	;
				  break;
			  case 4://迷路データの初期化
				  maze_init(maze_data.maze_y_size, maze_data.maze_x_size, maze_data.m_wall_tmp, maze_data.m_search_tmp);
				  break;
			  }

			  break;
		  }

		  //モード終了処理
		  clr_mode_state();
		  Sensor_StopADC();
		  set_duty_r(0);	//motor_r 停止
		  set_duty_l(0);	//motor_l 停止

		  for(int i=0; i<3; i++){ // モード処理終了時、LEDを3回点灯
			  LED_ALL_ON();
			  HAL_Delay(700);
			  LED_ALL_OFF();
			  HAL_Delay(300);
		  }

	  HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
