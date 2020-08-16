#ifndef MAZE_H_
#define MAZE_H_

void maze_data_output();

typedef struct{
	 uint8_t maze_x_size;//x方向の壁の枚数(x方向のマスの数+1)
	 uint8_t maze_y_size;//y方向の壁の枚数(y方向のマスの数+1)
	 uint8_t goal_size; //ゴールのマスの数
	 uint8_t maze_goal[18];
	 uint8_t m_wall_tmp[1024];//迷路情報格納用配列
	 uint8_t m_search_tmp[1024];//探索情報格納用配列
	 uint16_t contour_map[1024];//歩数マップ格納用配列
	 uint16_t row_num_node[1056];//斜め時行方向歩数マップ格納用配列
	 uint16_t col_num_node[1056];//斜め時列方向歩数マップ格納用配列

}maze_data_t;

extern maze_data_t maze_data;

#endif /* MAZE_H_*/
