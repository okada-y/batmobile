/*
 * maze.c
 * matlabとCの統合
 *  Created on: 2019/10/07
 *      Author: 岡田 泰裕
 */
#include "index.h"

maze_data_t maze_data = {x_size,y_size,g_size,goal_cordinate,0,0,0,0,0};

/* ---------------------------------------------------------------
	迷路データの出力
--------------------------------------------------------------- */
void maze_data_output()
{
	printf("maze_data_output\r\n") ;	//ログタイトル出力
	//タイトルが出力されたのを確認し、ボタンを押してデータを吐き出す
	while(!button_state){
		HAL_Delay(1);
	}

	printf("%d\r\n",maze_data.maze_x_size);
	printf("%d\r\n",maze_data.maze_y_size);
	printf("%d\r\n",maze_data.goal_size);

	for(uint8_t i = 0;i<18;i++){
		printf("%d,",maze_data.maze_goal[i]);
		if(i%9 == 8){
			printf("\r\n");
		}
	}

	for(uint16_t j = 0;j<1024;j++){
		printf("%d,",maze_data.m_wall_tmp[j]);
		if(j%32 == 31){
			printf("\r\n");
		}
	}
	while(button_state){
		HAL_Delay(1);
	}

	for(uint16_t k = 0;k<1024;k++){
		printf("%d,",maze_data.m_search_tmp[k]);
		if(k%32 == 31){
			printf("\r\n");
		}
	}

}
