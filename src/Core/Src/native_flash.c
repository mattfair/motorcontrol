#include "flash.h"
#include <string.h>

struct InternalFlashState
{
  bool initialized;
  bool locked;
  uint32_t start_addr;
  uint32_t end_addr;
} state;

void init_flash(uint32_t start_addr, uint32_t end_addr)
{
  state.initialized = true;
  state.locked = false;
  state.start_addr = start_addr;
  state.end_addr = end_addr;
}

uint32_t flash_get_start_addr(void) { return state.start_addr; }
uint32_t flash_get_end_addr(void) { return state.end_addr; }

void flash_unlock(void) { state.locked = false; }

void flash_lock(void) { state.locked = true; }

bool flash_read(void *data, uint32_t address, size_t size)
{
  if (!state.initialized)
  {
    return false;
  }

  if (address < state.start_addr || address + size > state.end_addr)
  {
    return false;
  }

  memcpy(data, (const void *)address, size);
  return true;
}

bool flash_write(void *data, uint32_t address, size_t size)
{
  if (!state.initialized)
  {
    return false;
  }

  if (address < state.start_addr || address + size > state.end_addr)
  {
    return false;
  }

  if (state.locked)
  {
    return false;
  }

  const uint8_t *data_ptr = (const uint8_t *)data;
  memcpy((void *)address, data_ptr, size);
  return true;
}

bool flash_erase(void)
{
  if (!state.initialized)
  {
    return false;
  }

  if (state.locked)
  {
    return false;
  }

  memset((void *)state.start_addr, 0, state.end_addr - state.start_addr);
  return true;
}
