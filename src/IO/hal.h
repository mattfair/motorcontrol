#pragma once

#if defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#define FLASH_USER_SECTOR FLASH_SECTOR_7
#endif
