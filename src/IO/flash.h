/*
 * Defines the flash memory interface for writing and reading data to persistent memory
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
  FLASH_OK = 0,
  FLASH_ERROR = 1
} FlashStatus;

typedef enum
{
  FLASH_LOCKED,
  FLASH_UNLOCKED
} FlashLockStatus;

// Initializes the flash memory interface
FlashStatus flash_init(uint32_t address, size_t size);

// Returns the start and end address of the flash memory
uint32_t flash_get_addr(void);

// Erases the flash memory sector where registers are stored
FlashStatus flash_erase(void);

// Locks the flash memory after writing
void flash_lock(void);

// Unlocks the flash memory for writing
void flash_unlock(void);

// returns flash lock state
FlashLockStatus flash_get_status(void);

// Function to write data to flash memory
FlashStatus flash_write(void *data, uint32_t address, size_t size);

// Function to read data from flash memory
FlashStatus flash_read(void *data, uint32_t address, size_t size);

#ifdef __cplusplus
}
#endif

