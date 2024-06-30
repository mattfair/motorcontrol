#include <IO/flash.h>
#include <IO/hal.h>
#include <stdio.h>
#include <string.h>

void flash_clear_flags_impl( void );

struct InternalFlashState
{
    uint32_t start_addr;
    uint32_t end_addr;
} flash_state;

FlashStatus flash_init( uint32_t address, size_t size )
{
    flash_state.start_addr = address;
    flash_state.end_addr = address + size;

    return FLASH_OK;
}

FlashStatus flash_destroy( void )
{
    return FLASH_OK;
}

uint32_t flash_get_addr( void )
{
    return flash_state.start_addr;
}

uint32_t flash_get_size( void )
{
    return flash_state.end_addr - flash_state.start_addr;
}

FlashStatus flash_erase( void )
{
    FLASH_EraseInitTypeDef erase_init_struct;
    erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init_struct.Sector = FLASH_SECTOR_7;
    erase_init_struct.NbSectors = 1;
    erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init_struct.Banks = 0;
    uint32_t page_error = 0;

    if ( HAL_FLASHEx_Erase( &erase_init_struct, &page_error ) != HAL_OK )
    {
        return FLASH_ERROR;
    }

    // Verification code
    uint32_t* sector_ptr = (uint32_t*) flash_state.start_addr;
    uint32_t size  = (flash_state.end_addr - flash_state.start_addr)/sizeof(uint32_t);
    for (size_t i = 0; i < size; ++i)
    {
        if (sector_ptr[i] != 0xFFFFFFFF)
        {
            return FLASH_ERROR;
        }
    }

    return FLASH_OK;
}

void flash_clear_flags_impl( void )
{
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                            FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR );
}
void ( *flash_clear_flags )( void ) = flash_clear_flags_impl;

FlashStatus flash_lock( void )
{
    if ( HAL_FLASH_Lock() == HAL_OK )
    {
        return FLASH_OK;
    }
    else
    {
        return FLASH_ERROR;
    }
}

FlashStatus flash_unlock( void )
{
    if ( HAL_FLASH_Unlock() == HAL_OK )
    {
        flash_clear_flags();
        return FLASH_OK;
    }
    else
    {
        return FLASH_ERROR;
    }
}

FlashStatus flash_read( void* data, uint32_t address, size_t size, DeSerializeFunctionPointer deserialize )
{
    // Ensure the address is within the bounds of the flash memory
    if ( address < flash_state.start_addr || address + size > flash_state.end_addr )
    {
        return FLASH_ERROR;
    }

    if ( deserialize )
    {
        void* source = (void*)address;
        deserialize( data, source, &size );
    }
    else
    {
        memcpy( data, (void*)address, size );
    }

    return FLASH_OK;
}

FlashStatus flash_write( void* data, uint32_t address, size_t size )
{
    if ( address < flash_state.start_addr || address + size > flash_state.end_addr )
    {
        return FLASH_ERROR;
    }

    const uint8_t* data_ptr = (const uint8_t*)data;
    for ( size_t i = 0; i < size; i++ )
    {
        //printf( "data_ptr[%d]: %d\n", i, data_ptr[i] );
        if ( HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE, address + i, data_ptr[i] ) != HAL_OK )
        {
            return FLASH_ERROR;
        }
    }
    return FLASH_OK;
}
