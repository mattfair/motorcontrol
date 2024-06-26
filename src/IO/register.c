/*
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Copyright (c) 2024 Matt Fair
 */
#include "IO/register.h"
#include <inttypes.h>
#include <stdio.h>
#include "IO/flash.h"
#include "Core/Inc/main.h"

static uint32_t register_start_addr = 0;
static uint32_t register_end_addr = 0;
static size_t memory_size = 0;
static size_t register_size = 0;
static size_t register_count = 0;
static size_t max_register_count = 0;
static uavcan_register_Name_1_0* registerNames;
static bool* registerImmutable;
static uint8_t serializeBuffer[mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };

#define PASTE3_IMPL( x, y, z ) x##y##z
#define PASTE3( x, y, z ) PASTE3_IMPL( x, y, z )

typedef struct RegisterIndex
{
    uint32_t value;
    enum
    {
        VALID = 0,
        REGISTER_NOT_FOUND
    } status;
} RegisterIndex;

uint32_t GetOffsetAddress( uint32_t index );
RegisterIndex FindRegisterIndex( const char* name );
bool Write( const char* name, const uavcan_register_Value_1_0* value, bool immutable );
void ReadRegisters();

void RegisterInit( uint32_t start_addr, size_t count )
{
    register_start_addr = start_addr;
    register_size = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    memory_size = register_size * count;
    register_end_addr = start_addr + count * register_size;
    register_count = 0;
    max_register_count = count;
    registerNames =
        (uavcan_register_Name_1_0*)malloc( count * uavcan_register_Name_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ );
    registerImmutable = (bool*)malloc( count * sizeof( bool ) );
    flash_init( register_start_addr, memory_size );
    ReadRegisters();
}

void RegisterDestroy()
{
    flash_destroy();

    free( registerImmutable );
    free( registerNames );

    register_start_addr = 0;
    register_end_addr = 0;
    register_size = 0;
    register_count = 0;
}

void ReadRegisters()
{
    FlashRegister reg;
    for ( uint32_t i = 0; i < max_register_count; i++ )
    {
        mattfair_storage_Register_1_0_initialize_( &reg );
        if ( !RegisterReadByIndex( i, &reg ) )
        {
            printf( "Failed to read register %" PRIu32 "\r\n", i );
            break;
        }

        if ( reg.name.name.count == 0 || reg.name.name.count == 255 )
        {
            break;
        }

        printf("Register %.*s\r\n", reg.name.name.count, reg.name.name.elements);
        registerNames[register_count] = reg.name;
        registerImmutable[register_count] = reg.isImmutable;
        register_count++;
    }
}

uint32_t RegisterStartAddress()
{
    return register_start_addr;
}
uint32_t RegisterEndAddress()
{
    return register_end_addr;
}

size_t RegisterSize()
{
    return register_size;
}

size_t RegisterCount()
{
    return register_count;
}

bool RegisterWrite( const char* name, const uavcan_register_Value_1_0* value, bool immutable )
{
    RegisterIndex index = FindRegisterIndex( name );
    if ( index.status == VALID )
    {
        if ( registerImmutable[index.value] )
        {
            printf( "Attempt to write immutable register %s denied\r\n", name );
            return false;
        }
    }
    else if ( index.status == REGISTER_NOT_FOUND )
    {
        index.status = VALID;
        index.value = register_count++;
    }

    uavcan_register_Name_1_0* storedName = &registerNames[index.value];
    uavcan_register_Name_1_0_initialize_( storedName );
    storedName->name.count = strlen( name );
    memset( storedName->name.elements, 0, sizeof( storedName->name.elements ) );
    memcpy( storedName->name.elements, name, storedName->name.count );
    registerImmutable[index.value] = immutable;

    FlashRegister reg = { .name = *storedName, .value = *value, .isImmutable = immutable };

    size_t size = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    memset( serializeBuffer, 0, size );
    if ( mattfair_storage_Register_1_0_serialize_( &reg, serializeBuffer, &size ) != NUNAVUT_SUCCESS )
    {
        return false;
    }

    flash_unlock();
    bool success = flash_write( serializeBuffer, GetOffsetAddress( index.value ), RegisterSize() ) == FLASH_OK;
    flash_lock();

    if ( !success )
    {
        printf( "Failed to write register %s\r\n", name );
        return false;
    }

    return true;
}

RegisterIndex FindRegisterIndex( const char* name )
{
    RegisterIndex notFound = { .status = REGISTER_NOT_FOUND };
    if ( register_count == 0 )
    {
        return notFound;
    }

    for ( uint32_t i = 0; i < register_count; i++ )
    {
        uavcan_register_Name_1_0* regName = &registerNames[i];
        if ( strncmp( (const char*)regName->name.elements, name, strlen( name ) ) == 0 )
        {
            printf( "Found Name: %s, Index: %d\r\n", name, (int)i );
            RegisterIndex found = { .value = i };
            return found;
        }
    }

    return notFound;
}

int8_t DeserializeFlashRegister( void* const out_obj, const uint8_t* buffer, size_t* const inout_buffer_size_bytes )
{
    return mattfair_storage_Register_1_0_deserialize_( out_obj, buffer, inout_buffer_size_bytes );
}

bool RegisterRead( const char* name, FlashRegister* regOut )
{
    RegisterIndex index = FindRegisterIndex( name );
    if ( index.status == REGISTER_NOT_FOUND )
    {
        return false;
    }

    if ( flash_read( regOut, GetOffsetAddress( index.value ), RegisterSize(), DeserializeFlashRegister ) != FLASH_OK )
    {
        return false;
    }

    return true;
}

bool RegisterReadByIndex( uint32_t index, FlashRegister* regOut )
{
    uint32_t address = GetOffsetAddress( index );

    if ( flash_read( regOut, address, RegisterSize(), DeserializeFlashRegister ) == FLASH_ERROR )
    {
        return false;
    }

    return true;
}

uavcan_register_Name_1_0 RegisterNameByIndex( const uint16_t index )
{
    if ( index >= register_count )
    {
        uavcan_register_Name_1_0 empty = { 0 };
        return empty;
    }

    return registerNames[index];
}

uint32_t GetOffsetAddress( uint32_t index )
{
    return register_start_addr + index * register_size;
}

// Erase all registers such that the defaults are used at the next launch.
void RegisterFactoryReset( void )
{
    if ( flash_unlock() != FLASH_OK )
    {
        printf( "Could not unlock flash memory\r\n" );
    }

    // if we failed to erase the flash memory, don't clear things
    if ( flash_erase() == FLASH_OK )
    {
        memset( registerNames, 0, max_register_count * uavcan_register_Name_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ );
        memset( registerImmutable, 0, max_register_count * sizeof( bool ) );
        register_count = 0;
    }
    else
    {
        printf( "Failed to erase flash memory\r\n" );
    }

    if ( flash_lock() != FLASH_OK )
    {
        printf( "Could not lock flash memory\r\n" );
    }
}

bool RegisterAssign( uavcan_register_Value_1_0* const dst, const uavcan_register_Value_1_0* const src )
{
    if ( uavcan_register_Value_1_0_is_empty_( dst ) )
    {
        *dst = *src;
        return true;
    }
    if ( ( uavcan_register_Value_1_0_is_string_( dst ) && uavcan_register_Value_1_0_is_string_( src ) ) ||
         ( uavcan_register_Value_1_0_is_unstructured_( dst ) && uavcan_register_Value_1_0_is_unstructured_( src ) ) )
    {
        *dst = *src;
        return true;
    }
    if ( uavcan_register_Value_1_0_is_bit_( dst ) && uavcan_register_Value_1_0_is_bit_( src ) )
    {
        nunavutCopyBits( dst->bit.value.bitpacked,
                         0,
                         nunavutChooseMin( dst->bit.value.count, src->bit.value.count ),
                         src->bit.value.bitpacked,
                         0 );
        return true;
    }
    // This is a violation of MISRA/AUTOSAR but it is believed to be less error-prone than manually copy-pasted code.
#define REGISTER_CASE_SAME_TYPE( TYPE )                                                                 \
    if ( PASTE3( uavcan_register_Value_1_0_is_, TYPE, _ )( dst ) &&                                     \
         PASTE3( uavcan_register_Value_1_0_is_, TYPE, _ )( src ) )                                      \
    {                                                                                                   \
        for ( size_t i = 0; i < nunavutChooseMin( dst->TYPE.value.count, src->TYPE.value.count ); ++i ) \
        {                                                                                               \
            dst->TYPE.value.elements[i] = src->TYPE.value.elements[i];                                  \
        }                                                                                               \
        return true;                                                                                    \
    }
    REGISTER_CASE_SAME_TYPE( integer64 )
    REGISTER_CASE_SAME_TYPE( integer32 )
    REGISTER_CASE_SAME_TYPE( integer16 )
    REGISTER_CASE_SAME_TYPE( integer8 )
    REGISTER_CASE_SAME_TYPE( natural64 )
    REGISTER_CASE_SAME_TYPE( natural32 )
    REGISTER_CASE_SAME_TYPE( natural16 )
    REGISTER_CASE_SAME_TYPE( natural8 )
    REGISTER_CASE_SAME_TYPE( real64 )
    REGISTER_CASE_SAME_TYPE( real32 )
    REGISTER_CASE_SAME_TYPE( real16 )
    return false;
}
