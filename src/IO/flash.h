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

// Initializes the flash memory interface
FlashStatus flash_init(uint32_t address, size_t size);

// DeInitializes the flash memory interface
FlashStatus flash_destroy(void);

// Returns the start and end address of the flash memory
uint32_t flash_get_addr(void);

// return the size of the flash memory
uint32_t flash_get_size(void);

// Erases the flash memory sector where registers are stored
FlashStatus flash_erase(void);

// Locks the flash memory after writing
FlashStatus flash_lock(void);

// Unlocks the flash memory for writing
FlashStatus flash_unlock(void);

// Function to write data to flash memory
FlashStatus flash_write(void *data, uint32_t address, size_t size);

// Function to read data from flash memory
FlashStatus flash_read(void *data, uint32_t address, size_t size);

//function so that we can swap out some hardware specific memory
extern void (*flash_clear_flags)(void);

#ifdef __cplusplus
}
#endif

