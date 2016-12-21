#include "flash.h"

#include <stdint.h>
#include "stm32f2xx.h"
#include <stm32f2xx_hal_flash_ex.h>
#include <stm32f2xx_hal_flash.h>
int flash_erase_sec(int sec_no)
{
	FLASH_EraseInitTypeDef er = {
		.TypeErase = TYPEERASE_SECTORS,
		.Sector = sec_no,
		.NbSectors = 1,
		.VoltageRange = VOLTAGE_RANGE_3
	};
	uint32_t fault_sec = 0;
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef res = HAL_FLASHEx_Erase(&er, &fault_sec);
	HAL_FLASH_Lock();
	return res == HAL_OK ? 0 : -1;
}

int flash_write(unsigned addr, void const* data, unsigned sz)
{
	HAL_StatusTypeDef res = HAL_OK;
	HAL_FLASH_Unlock();
	for (; addr % 4 && sz >= 1; sz -= 1, addr += 1) {
		res = HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr, *(uint8_t const*)data);
		data = (uint8_t const*)data + 1;
		if (res != HAL_OK)
			goto done;
	}
	for (; sz >= 4; sz -= 4, addr += 4) {
		res = HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, *(uint32_t const*)data);
		data = (uint32_t const*)data + 1;
		if (res != HAL_OK)
			goto done;
	}
	for (; sz >= 1; sz -= 1, addr += 1) {
		res = HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr, *(uint8_t const*)data);
		data = (uint8_t const*)data + 1;
		if (res != HAL_OK)
			goto done;
	}
done:
	HAL_FLASH_Lock();
	return res == HAL_OK ? 0 : -1;
}

int flash_write_bytes(unsigned addr, void const* data, unsigned sz)
{
	HAL_StatusTypeDef res = HAL_OK;
	HAL_FLASH_Unlock();
	for (; sz >= 1; sz -= 1, addr += 1) {
		res = HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr, *(uint8_t const*)data);
		data = (uint8_t const*)data + 1;
		if (res != HAL_OK)
			goto done;
	}
done:
	HAL_FLASH_Lock();
	return res == HAL_OK ? 0 : -1;
}
