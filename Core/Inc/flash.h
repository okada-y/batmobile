/*
 * flash.h
 *
 *  Created on: 2020/08/04
 *      Author: 岡田 泰裕
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

void writeFlash(uint32_t , uint8_t* , uint32_t );
void eraseFlash( void );
void loadFlash(uint32_t, uint8_t*, uint32_t );

#endif /* INC_FLASH_H_ */
