#include "IO/flash.h"
#include "IO/hal.h"

struct InternalFlashState
{
  bool initialized;
  uint32_t start_addr;
  uint32_t end_addr;
} state;

FlashStatus init_flash(uint32_t address, size_t size)
{
  state.initialized = true;
  state.start_addr = address;
  state.end_addr = address + size;

  return FLASH_OK;
}

void flash_unlock(void)
{
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

// Locks the flash memory after writing
void flash_lock(void)
{
  HAL_FLASH_Lock();
}

FlashStatus flash_read(void *data, uint32_t address, size_t size)
{
  if (address < state.start_addr || address + size > state.end_addr)
  {
    HAL_FLASH_Read(
    return FLASH_ERROR;
  }
  {
    return FLASH_ERROR;
  }

  memcpy(data, (const void *)address, size);
  return FLASH_OK;
}

FlashStatus flash_write(void *data, uint32_t address, size_t size)
{
  if (address < state.start_addr || address + size > state.end_addr)
  {
    return FLASH_ERROR;
  }

  const uint8_t *data_ptr = (const uint8_t *)data;
  for (size_t i = 0; i < size; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data_ptr[i]) != HAL_OK)
    {
      return FLASH_ERROR;
    }
  }
  return FLASH_OK;
}

FlashStatus flash_erase(void)
{
  FLASH_EraseInitTypeDef erase_init_struct;
  uint32_t page_error;
  erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init_struct.Sector = FLASH_USER_SECTOR;
  erase_init_struct.NbSectors = 1;
  erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK)
  {
    return FLASH_ERROR;
  }
  return FLASH_OK;
}
