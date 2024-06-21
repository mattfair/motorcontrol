#include "IO/register.h"
#include <cmock.h>
#include <unity.h>
#include "mock_stm32f4xx_hal_flash.h"

/**
 * Specify the address and size for the register memory
 * Load register names into a list
 * Look up register index by name
 * Read and write register values
 * Mark registers as immutable
 * Clear all register values
 */

static uint8_t flash_data[1024];

// callback function to make HAL_FLASH_Program write to flash_data
HAL_StatusTypeDef HAL_FLASH_Program_Callback( uint32_t TypeProgram,
                                              uint32_t Address,
                                              uint64_t Data,
                                              int cmock_num_calls )
{
    if ( cmock_num_calls == 0 )
    {
        //  return HAL_ERROR;
    }
    if ( TypeProgram == FLASH_TYPEPROGRAM_BYTE )
    {
        memcpy( (void*)Address, &Data, sizeof( Data ) );
    }
    else
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

void setUp( void )
{
    // Clear the flash data
    for ( size_t i = 0; i < sizeof( flash_data ); i++ )
    {
        flash_data[i] = 0xFF;
    }
    HAL_FLASH_Program_AddCallback( HAL_FLASH_Program_Callback );
    uint32_t address = (uint32_t)flash_data;
    RegisterInit( address, 100 );
}

void tearDown( void )
{
    RegisterDestroy();
}

void WriteFlashBytesExpectAndReturn( uint32_t address, void* data, size_t size, HAL_StatusTypeDef status )
{
    const uint8_t* buffer = (const uint8_t*)data;
    for ( size_t i = 0; i < size; i++ )
    {
        // printf( "expect data_ptr[%d]: %d\n", i, buffer[i] );
        HAL_FLASH_Program_ExpectAndReturn( FLASH_TYPEPROGRAM_BYTE, address + i, buffer[i], status );
        // printf( "cmock memory used: %d, free: %d, capacity: %d\n",
        //         CMock_Guts_MemBytesUsed(),
        //         CMock_Guts_MemBytesFree(),
        //         CMock_Guts_MemBytesCapacity() );
    }
}

void SetName( uavcan_register_Name_1_0* name, const char* str )
{
    name->name.count = strlen( str );
    memset( name->name.elements, 0, sizeof( name->name.elements ) );
    memcpy( name->name.elements, str, name->name.count );
}

void AddRegisterNatural32Value( int index, const char* name, uint32_t value )
{
    FlashRegister reg = { 0 };
    SetName( &reg.name, name );
    reg.value.natural32.value.count = 1;
    reg.value.natural32.value.elements[0] = value;
    reg.isImmutable = false;
    uavcan_register_Value_1_0_select_natural32_( &reg.value );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    size_t size = RegisterSize();
    WriteFlashBytesExpectAndReturn( GetOffsetAddress( index ), buffer, size, HAL_OK );

    TEST_ASSERT_TRUE_MESSAGE( RegisterWrite( name, &reg.value ), "Failed to write natural32 register" );
}

void AddRegisterReal64Value( int index, const char* name, double value )
{
    FlashRegister reg = { 0 };
    SetName( &reg.name, name );
    reg.value.real64.value.count = 1;
    reg.value.real64.value.elements[0] = value;
    reg.isImmutable = false;
    uavcan_register_Value_1_0_select_real64_( &reg.value );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    size_t size = RegisterSize();
    WriteFlashBytesExpectAndReturn( GetOffsetAddress( index ), buffer, size, HAL_OK );

    TEST_ASSERT_TRUE_MESSAGE( RegisterWrite( name, &reg.value ), "Failed to write real64 register" );
}

void AddRegisterStringValue( int index, const char* name, const char* value )
{
    FlashRegister reg = { 0 };
    SetName( &reg.name, name );
    reg.value._string.value.count = strlen( value );
    reg.isImmutable = false;
    memcpy( reg.value._string.value.elements, value, reg.value._string.value.count );
    uavcan_register_Value_1_0_select_real64_( &reg.value );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    size_t size = RegisterSize();
    WriteFlashBytesExpectAndReturn( GetOffsetAddress( index ), buffer, size, HAL_OK );

    TEST_ASSERT_TRUE_MESSAGE( RegisterWrite( name, &reg.value ), "Failed to write string register" );
}

void test_init( void )
{
    uintptr_t address = (uintptr_t)flash_data;
    TEST_ASSERT_EQUAL( address, RegisterStartAddress() );
    TEST_ASSERT_EQUAL( GetOffsetAddress( 100 ), RegisterEndAddress() );
    TEST_ASSERT_EQUAL( 0, RegisterCount() );
}

void test_register_size( void )
{
    TEST_ASSERT_EQUAL(
        uavcan_register_Name_1_0_EXTENT_BYTES_ + uavcan_register_Value_1_0_EXTENT_BYTES_ + sizeof( bool ),
        RegisterSize() );
}

void test_single_write_increments_count( void )
{
    FlashRegister reg = { 0 };
    reg.isImmutable = false;
    reg.name.name.count = 4;
    memcpy( reg.name.name.elements, "test", 4 );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    WriteFlashBytesExpectAndReturn( RegisterStartAddress(), buffer, RegisterSize(), HAL_OK );

    uavcan_register_Value_1_0 value = { 0 };
    TEST_ASSERT_TRUE( RegisterWrite( "test", &value ) );
    TEST_ASSERT_EQUAL( 1, RegisterCount() );
}

void test_empty_GetRegisterNameByIndex( void )
{
    uavcan_register_Name_1_0 name = RegisterNameByIndex( 0 );
    TEST_ASSERT_EQUAL( 0, name.name.count );
    TEST_ASSERT_EQUAL_STRING( "", name.name.elements );
}

void test_single_write_lookup_name_by_index( void )
{
    const char* regName = "test";
    AddRegisterNatural32Value( 0, regName, 0 );

    uavcan_register_Name_1_0 name = RegisterNameByIndex( 0 );
    TEST_ASSERT_EQUAL( 4, name.name.count );
    TEST_ASSERT_EQUAL_STRING_LEN( regName, name.name.elements, name.name.count );
}

void test_multiple_write_lookup_name_by_index( void )
{
    char* names[4] = { "foo", "bar", "abc", "def" };

    // first add the registers
    for ( size_t i = 0; i < 4; i++ )
    {
        AddRegisterNatural32Value( i, names[i], 0 );
    }

    // verify the name lookups
    for ( size_t i = 0; i < 4; i++ )
    {
        uavcan_register_Name_1_0 name = RegisterNameByIndex( i );
        TEST_ASSERT_EQUAL( strlen( names[i] ), name.name.count );
        TEST_ASSERT_EQUAL_STRING_LEN( names[i], name.name.elements, name.name.count );
    }
}

void test_single_rw_natural32_value( void )
{
    const char* name = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( 0, name, answer );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( name, &read ) );
    // TEST_ASSERT_EQUAL( 1, read.value.natural32.value.count );
    TEST_ASSERT_EQUAL( answer, read.value.natural32.value.elements[0] );
}

void test_single_rw_real64_value( void )
{
    const char* name = "pi";
    double pi = 3.14159;
    AddRegisterReal64Value( 0, name, pi );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( name, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( pi, read.value.real64.value.elements[0] );
}

void test_multiple_rw_mixed_values( void )
{
    const char* piName = "pi";
    double pi = 3.14159265359;
    AddRegisterReal64Value( 0, piName, pi );

    const char* tauName = "tau";
    double tau = 6.28318530718;
    AddRegisterReal64Value( 1, tauName, tau );

    const char* phiName = "phi";
    double phi = 1.61803398875;
    AddRegisterReal64Value( 2, phiName, phi );

    const char* actuatorName = "actuator_name";
    const char* actuatorValue = "linear_123";
    AddRegisterStringValue( 3, actuatorName, actuatorValue );

    const char* answerName = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( 4, answerName, answer );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( piName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( pi, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( phiName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( phi, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( tauName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( tau, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( actuatorName, &read ) );
    TEST_ASSERT_EQUAL( strlen( actuatorValue ), read.value._string.value.count );
    TEST_ASSERT_EQUAL_STRING( actuatorValue, read.value._string.value.elements );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( answerName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.natural32.value.count );
    TEST_ASSERT_EQUAL( answer, read.value.natural32.value.elements[0] );
}
