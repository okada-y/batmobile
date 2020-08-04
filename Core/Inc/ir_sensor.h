#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_
#include <stdio.h>
#include <math.h>

void Sensor_Initialize( void );
void Sensor_StartADC( void );
void Sensor_StopADC( void );
uint16_t Sensor_GetBatteryValue( void );
int16_t Sensor_GetValue( uint8_t dir );
double SensorValue2length( uint8_t dir );
void Sensor_DebugPrintf( void );

#define SENSOR_ALL_OFF()			HAL_GPIO_WritePin(GPIOB, FR_LED_Pin|SL_LED_Pin|SR_LED_Pin|FL_LED_Pin, GPIO_PIN_RESET)
#define SENSOR_FRONT_ON()		HAL_GPIO_WritePin(GPIOB, FR_LED_Pin|FL_LED_Pin, GPIO_PIN_SET)
#define SENSOR_FRONT_OFF()		HAL_GPIO_WritePin(GPIOB, FL_LED_Pin|FR_LED_Pin, GPIO_PIN_RESET)
#define SENSOR_SIDE_ON()			HAL_GPIO_WritePin(GPIOB, SL_LED_Pin|SR_LED_Pin, GPIO_PIN_SET)
#define SENSOR_SIDE_OFF()		HAL_GPIO_WritePin(GPIOB, SL_LED_Pin|SR_LED_Pin, GPIO_PIN_RESET)


typedef enum {
	sensor_all_off_mode 	= 0,
	sensor_front_on_mode 	= 1,
	sensor_side_on_mode 	= 2,
} t_sensor_mode;

typedef enum {
	front_left = 0,
	side_left 	= 1,
	side_right	= 2,
	front_right	= 3,
} t_sensor_dir;

extern uint8_t sensor_mode;

#endif /* IR_SENSOR_H_*/
