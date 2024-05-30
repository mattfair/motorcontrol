#include "IO/flash.h"

#include <IO/hal.h>
#include <string.h>

static uint32_t flashAddress;
static size_t flashSize;
static FlashLockStatus lockStatus;

FlashStatus flash_init(uint32_t address, size_t size)
{
  flashAddress = address;
  flashSize = size;
  lockStatus = FLASH_LOCKED;

  return FLASH_OK;
}

uint32_t flash_get_addr(void)
{
  return flashAddress;
}

FlashStatus flash_erase(void)
{
  memset((void *)flashAddress, 0xFF, flashSize);
  return FLASH_OK;
}

void flash_lock(void)
{
  // HAL_FLASH_Unlock();
  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  lockStatus = FLASH_LOCKED;
}

void flash_unlock(void)
{
  lockStatus = FLASH_UNLOCKED;
}

FlashLockStatus flash_get_status(void)
{
  return lockStatus;
}

FlashStatus flash_write(void *data, uint32_t address, size_t size)
{
  if (lockStatus == FLASH_LOCKED)
  {
    return FLASH_ERROR;
  }

  memcpy((void *)address, data, size);
  return FLASH_ERROR;
}

FlashStatus flash_read(void *data, uint32_t address, size_t size)
{
  memcpy(data, (void *)address, size);
  return FLASH_OK;
}
