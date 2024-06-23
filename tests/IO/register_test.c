#include "IO/register.h"
#include <cmock.h>
#include <unity.h>
#include "IO/flash.h"
#include "mock_stm32f4xx_hal_flash.h"
#include "mock_stm32f4xx_hal_flash_ex.h"

#define NUM_REGISTERS 100
static uint8_t flash_data[mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ * NUM_REGISTERS];

// callback function to make HAL_FLASH_Program write to flash_data
static HAL_StatusTypeDef CallbackReturn = HAL_OK;
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
    return CallbackReturn;
}

void ( *flash_clear_flags_hw )( void );
void mock_clear_flags( void ) {}

/*
static int dump_count = 1;
static const char* dumpFormat = "dump/flash_data-%0d.bin";

static void dump_flash_data_to_file( void )
{
    char filename[24];
    sprintf( filename, dumpFormat, dump_count++ );

    FILE* fp = fopen( filename, "wb" );
    if ( fp == NULL )
    {
        printf( "Failed to open file %s\n", filename );
        return;
    }
    fwrite( flash_data, mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_, NUM_REGISTERS, fp );
    fclose( fp );
}
*/

void setUp( void )
{
    // Clear the flash data
    for ( size_t i = 0; i < sizeof( flash_data ); i++ )
    {
        flash_data[i] = 0xFF;
    }
    CallbackReturn = HAL_OK;
    HAL_FLASH_Program_AddCallback( HAL_FLASH_Program_Callback );
    flash_clear_flags_hw = flash_clear_flags;
    flash_clear_flags = mock_clear_flags;

    uint32_t address = (uint32_t)flash_data;
    RegisterInit( address, NUM_REGISTERS );
}

void tearDown( void )
{
    RegisterDestroy();
    flash_clear_flags = flash_clear_flags_hw;
}

void WriteFlashBytesExpectAndReturn( uint32_t address, void* data, size_t size, HAL_StatusTypeDef status )
{
    const uint8_t* buffer = (const uint8_t*)data;
    for ( size_t i = 0; i < size; i++ )
    {
        // printf( "expect data_ptr[%d]: %d\n", i, buffer[i] );
        HAL_FLASH_Program_ExpectAndReturn( FLASH_TYPEPROGRAM_BYTE, address + i, buffer[i], status );
    }
}

void SetName( uavcan_register_Name_1_0* name, const char* str )
{
    name->name.count = strlen( str );
    memset( name->name.elements, 0, sizeof( name->name.elements ) );
    memcpy( name->name.elements, str, name->name.count );
}

void add_natural_32( int index, const char* name, uint32_t value, bool isImmutable )
{
    FlashRegister reg = { 0 };
    SetName( &reg.name, name );
    reg.value.natural32.value.count = 1;
    reg.value.natural32.value.elements[0] = value;
    reg.isImmutable = isImmutable;
    uavcan_register_Value_1_0_select_natural32_( &reg.value );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    size_t size = RegisterSize();
    WriteFlashBytesExpectAndReturn( GetOffsetAddress( index ), buffer, size, HAL_OK );

    TEST_ASSERT_TRUE_MESSAGE( RegisterWrite( name, &reg.value, isImmutable ), "Failed to write natural32 register" );
    //dump_flash_data_to_file();
}

void AddRegisterNatural32Value( int index, const char* name, uint32_t value )
{
    add_natural_32( index, name, value, false );
}

void AddImmutableRegisterNatural32Value( int index, const char* name, uint32_t value )
{
    add_natural_32( index, name, value, true );
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

    TEST_ASSERT_TRUE_MESSAGE( RegisterWrite( name, &reg.value, false ), "Failed to write real64 register" );
    //dump_flash_data_to_file();
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

    TEST_ASSERT_TRUE_MESSAGE( RegisterWrite( name, &reg.value, false ), "Failed to write string register" );
    //dump_flash_data_to_file();
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

void test_write_fail( void )
{
    FlashRegister reg = { 0 };
    reg.isImmutable = false;
    reg.name.name.count = 4;
    memcpy( reg.name.name.elements, "test", 4 );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    WriteFlashBytesExpectAndReturn( RegisterStartAddress(), buffer, 1, HAL_ERROR );

    CallbackReturn = HAL_ERROR;
    uavcan_register_Value_1_0 value = { 0 };
    TEST_ASSERT_FALSE( RegisterWrite( "test", &value, false ) );
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
    TEST_ASSERT_TRUE( RegisterWrite( "test", &value, false ) );
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

void test_multiple_mixed_values_read_by_name( void )
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
    TEST_ASSERT_TRUE( RegisterRead( tauName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( tau, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( phiName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( phi, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( actuatorName, &read ) );
    TEST_ASSERT_EQUAL( strlen( actuatorValue ), read.value._string.value.count );
    TEST_ASSERT_EQUAL_STRING( actuatorValue, read.value._string.value.elements );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( answerName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.natural32.value.count );
    TEST_ASSERT_EQUAL( answer, read.value.natural32.value.elements[0] );
}

void test_multiple_mixed_values_read_by_index( void )
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
    TEST_ASSERT_TRUE( RegisterReadByIndex( 0, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( pi, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( 1, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( tau, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( 2, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( phi, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( 3, &read ) );
    TEST_ASSERT_EQUAL( strlen( actuatorValue ), read.value._string.value.count );
    TEST_ASSERT_EQUAL_STRING( actuatorValue, read.value._string.value.elements );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( 4, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.natural32.value.count );
    TEST_ASSERT_EQUAL( answer, read.value.natural32.value.elements[0] );
}

void test_write_strings( void )
{
    for ( size_t i = 0; i < 10; i++ )
    {
        char name[10];
        char value[10];
        sprintf( name, "name_%d", i );
        sprintf( value, "value_%d", i );
        AddRegisterStringValue( i, name, value );
    }

    for ( size_t i = 0; i < 10; i++ )
    {
        char name[10];
        char value[10];
        sprintf( name, "name_%d", i );
        sprintf( value, "value_%d", i );

        FlashRegister read = { 0 };
        TEST_ASSERT_TRUE( RegisterRead( name, &read ) );
        TEST_ASSERT_EQUAL( strlen( value ), read.value._string.value.count );
        TEST_ASSERT_EQUAL_STRING( value, read.value._string.value.elements );
    }
}

void test_write_strings_and_real64( void )
{
    for ( size_t i = 0; i < 20; i++ )
    {
        if ( i % 2 == 0 )
        {
            char real_name[10];
            double real_value = 3.14159265359;
            sprintf( real_name, "real_%d", i );
            AddRegisterReal64Value( i, real_name, real_value );
        }
        else
        {
            char str_name[10];
            char str_value[10];
            sprintf( str_name, "string_%d", i );
            sprintf( str_value, "value_%d", i );
            AddRegisterStringValue( i, str_name, str_value );
        }
    }
    TEST_ASSERT_EQUAL( 20, RegisterCount() );

    for ( size_t i = 0; i < 20; i++ )
    {
        if ( i % 2 == 0 )
        {
            char real_name[10];
            sprintf( real_name, "real_%d", i );

            FlashRegister read = { 0 };
            TEST_ASSERT_TRUE( RegisterRead( real_name, &read ) );
            TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
            TEST_ASSERT_EQUAL( 3.14159265359, read.value.real64.value.elements[0] );
        }
        else
        {
            char string_name[10];
            char string_value[10];
            sprintf( string_name, "string_%d", i );
            sprintf( string_value, "value_%d", i );

            FlashRegister read = { 0 };
            TEST_ASSERT_TRUE( RegisterRead( string_name, &read ) );
            TEST_ASSERT_EQUAL( strlen( string_value ), read.value._string.value.count );
            TEST_ASSERT_EQUAL_STRING( string_value, read.value._string.value.elements );
        }
    }
}

void test_create_immutable_register( void )
{
    const char* name = "immutable";
    AddImmutableRegisterNatural32Value( 0, name, 42 );

    // verify the register is immutable
    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( name, &read ) );
    TEST_ASSERT_TRUE( read.isImmutable );

    // now try to write to the immutable register
    FlashRegister reg = { 0 };
    SetName( &reg.name, name );
    reg.isImmutable = false;
    reg.value.natural32.value.count = 1;
    reg.value.natural32.value.elements[0] = 1234;
    uavcan_register_Value_1_0_select_natural32_( &reg.value );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    TEST_ASSERT_FALSE( RegisterWrite( name, &reg.value, false ) );
}

void test_factory_reset( void )
{
    const char* name = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( 0, name, answer );

    TEST_ASSERT_EQUAL( 1, RegisterCount() );

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    FLASH_EraseInitTypeDef erase_init_struct;
    erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init_struct.Sector = FLASH_SECTOR_7;
    erase_init_struct.NbSectors = 1;
    erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init_struct.Banks = 0;
    uint32_t page_error = 0;

    HAL_FLASHEx_Erase_ExpectAndReturn( &erase_init_struct, &page_error, HAL_OK );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    RegisterFactoryReset();

    TEST_ASSERT_EQUAL( 0, RegisterCount() );
}

void test_factory_reset_failure( void )
{
    const char* name = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( 0, name, answer );

    TEST_ASSERT_EQUAL( 1, RegisterCount() );

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    FLASH_EraseInitTypeDef erase_init_struct;
    erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init_struct.Sector = FLASH_SECTOR_7;
    erase_init_struct.NbSectors = 1;
    erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init_struct.Banks = 0;
    uint32_t page_error = 0;

    HAL_FLASHEx_Erase_ExpectAndReturn( &erase_init_struct, &page_error, HAL_ERROR );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    RegisterFactoryReset();

    // should have not reset anything because there was an error
    TEST_ASSERT_EQUAL( 1, RegisterCount() );
}

void test_load_on_startup( void )
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

    TEST_ASSERT_EQUAL( 5, RegisterCount() );

    // shutdown
    RegisterDestroy();
    TEST_ASSERT_EQUAL( 0, RegisterCount() );

    // reinitialize
    uintptr_t address = (uintptr_t)flash_data;
    RegisterInit( address, 100 );
    TEST_ASSERT_EQUAL( 5, RegisterCount() );
}
