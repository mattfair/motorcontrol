// Validates the Mock HAL functions

#include "mock_stm32f4xx_hal.h"
#include "mock_stm32f4xx_hal_flash.h"
#include "mock_stm32f4xx_hal_flash_ex.h"

void test_HAL_Init( void )
{
    HAL_Init_ExpectAndReturn( HAL_OK );
    HAL_Init();
}

void test_HAL_InitTick( void )
{
    HAL_InitTick_ExpectAndReturn( 1000, HAL_OK );
    HAL_InitTick( 1000 );
}

void test_HAL_DeInit( void )
{
    HAL_DeInit_ExpectAndReturn( HAL_OK );
    HAL_DeInit();
}

void test_HAL_MspInit( void )
{
    HAL_MspInit_Expect();
    HAL_MspInit();
}

void test_HAL_MspDeInit( void )
{
    HAL_MspDeInit_Expect();
    HAL_MspDeInit();
}

void test_HAL_Delay( void )
{
    uint32_t ms = 1000;
    HAL_Delay_Expect( ms );
    HAL_Delay( ms );
}

void test_HAL_GetTick( void )
{
    HAL_GetTick_ExpectAndReturn( 1000 );
    HAL_GetTick();
}

void test_HAL_FLASH_Program( void )
{
    uint32_t typeProgram = FLASH_TYPEPROGRAM_WORD;
    uint32_t address = 0x08000000;
    uint64_t data = 0x12345678ABCDEF00;

    HAL_FLASH_Program_ExpectAndReturn( typeProgram, address, data, HAL_OK );
    HAL_FLASH_Program( typeProgram, address, data );
}

void test_HAL_FLASH_Lock( void )
{
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );
    HAL_FLASH_Lock();
}

void test_HAL_FLASH_Unlock( void )
{
    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    HAL_FLASH_Unlock();
}

void test_HAL_FLASHEx_Erase( void )
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = 7U;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASHEx_Erase_ExpectAndReturn( &erase, &page_error, HAL_OK );
    HAL_FLASHEx_Erase( &erase, &page_error );
}
