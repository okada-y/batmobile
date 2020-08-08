/*
 * flash.c
 *
 *  Created on: 2020/08/03
 *      Author: 岡田 泰裕
 */

#include "index.h"

//flash用
const uint32_t start_address = 0x08160000; //sentor15 start address
const uint32_t end_adress = 0x0817FFFF; // sector15 end address

/*
 *@brief erase sector15
*/
void eraseFlash( void )
{
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;	// select sector
	erase.Sector = FLASH_SECTOR_15;		       // set selector11
	erase.NbSectors = 1;		// set to erase one sector
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;	// set voltage range (2.7 to 3.6V)

	uint32_t pageError = 0;

	HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
}

/*
 * @brief write flash(sector15)
 * @param uint32_t address sector15 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
*/
void writeFlash(uint32_t address, uint8_t *data, uint32_t size  )
{
	HAL_FLASH_Unlock();		// unlock flash
	eraseFlash();			// erease sector15

  for ( uint32_t add = address; add < (address + size); add++ )
  {
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, add, *data); // write byte
    data++;  // add data pointer
	}

	HAL_FLASH_Lock();		// lock flash
}

/*
 * @brief write flash(sector15)
 * @param uint32_t address sector15 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
*/
void loadFlash(uint32_t address, uint8_t *data, uint32_t size )
{
	memcpy(data, (uint8_t*)address, size); // copy data
}
