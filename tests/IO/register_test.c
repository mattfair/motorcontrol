#include "IO/register.h"
#include <cmock.h>
#include <o1heap.h>
#include <unity.h>
#include "IO/flash.h"
#include "mock_stm32f4xx_hal_flash.h"
#include "mock_stm32f4xx_hal_flash_ex.h"

#define NUM_REGISTERS 100
static uint8_t flash_data[mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ * NUM_REGISTERS];
uint32_t address_ptr = (uint32_t)flash_data;

_Alignas( O1HEAP_ALIGNMENT ) static uint8_t heap_arena[1024 * 50] = { 0 };

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

FlashStatus ( *flash_erase_hw )( void );
FlashStatus mock_flash_erase( void )
{
    return FLASH_OK;
}

RegisterInstance* instance;
O1HeapInstance* heap;

static void* RegisterAllocate( RegisterInstance* const ins, const size_t amount )
{
    assert( o1heapDoInvariantsHold( ins->heap ) );
    return o1heapAllocate( ins->heap, amount );
}

static void RegisterFree( RegisterInstance* const ins, void* const pointer )
{
    printf( "Freeing item: %p\r\n", (void*)pointer );
    o1heapFree( ins->heap, pointer );
}

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

    heap = o1heapInit( heap_arena, sizeof( heap_arena ) );

    HAL_FLASH_Program_AddCallback( HAL_FLASH_Program_Callback );
    address_ptr = (uint32_t)flash_data;

    flash_clear_flags_hw = flash_clear_flags;
    flash_clear_flags = mock_clear_flags;
    flash_erase_hw = flash_erase;
    flash_erase = mock_flash_erase;

    uint32_t address = (uint32_t)flash_data;
    instance = RegisterInit( address, NUM_REGISTERS, heap, RegisterAllocate, RegisterFree );
}

void tearDown( void )
{
    RegisterDestroy( instance );
    flash_clear_flags = flash_clear_flags_hw;
    flash_erase = flash_erase_hw;
}

void WriteFlashBytesExpectAndReturn( void* data, size_t size, HAL_StatusTypeDef status )
{
    (void)data;
    // const uint8_t* buffer = (const uint8_t*)data;
    for ( size_t i = 0; i < size; i++ )
    {
        HAL_FLASH_Program_IgnoreAndReturn( status );
        // printf( "expect data_ptr[%d]: %d\n", i, buffer[i] );
        // HAL_FLASH_Program_ExpectAndReturn( FLASH_TYPEPROGRAM_BYTE, address_ptr + i, buffer[i], status );
    }

    address_ptr += size;
}

void SetName( uavcan_register_Name_1_0* name, const char* str )
{
    name->name.count = strlen( str );
    memset( name->name.elements, 0, sizeof( name->name.elements ) );
    memcpy( name->name.elements, str, name->name.count );
}

void add_natural_32( const char* name, uint32_t value, bool isImmutable )
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

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    WriteFlashBytesExpectAndReturn( buffer, bufferSize, HAL_OK );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    TEST_ASSERT_TRUE_MESSAGE( RegisterAdd( instance, name, &reg.value, isImmutable ),
                              "Failed to write natural32 register" );
    TEST_ASSERT_TRUE( RegisterFlush( instance ) );
    // dump_flash_data_to_file();
}

void AddRegisterNatural32Value( const char* name, uint32_t value )
{
    add_natural_32( name, value, false );
}

void AddImmutableRegisterNatural32Value( const char* name, uint32_t value )
{
    add_natural_32( name, value, true );
}

void AddRegisterReal64Value( const char* name, double value )
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

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    WriteFlashBytesExpectAndReturn( buffer, bufferSize, HAL_OK );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    TEST_ASSERT_TRUE_MESSAGE( RegisterAdd( instance, name, &reg.value, false ), "Failed to write real64 register" );
    TEST_ASSERT_TRUE( RegisterFlush( instance ) );
    // dump_flash_data_to_file();
}

void AddRegisterStringValue( const char* name, const char* value )
{
    FlashRegister reg = { 0 };
    SetName( &reg.name, name );
    reg.value._string.value.count = strlen( value );
    reg.isImmutable = false;
    memcpy( reg.value._string.value.elements, value, reg.value._string.value.count );
    uavcan_register_Value_1_0_select_string_( &reg.value );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    WriteFlashBytesExpectAndReturn( buffer, bufferSize, HAL_OK );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    TEST_ASSERT_TRUE_MESSAGE( RegisterAdd( instance, name, &reg.value, false ), "Failed to write string register" );
    TEST_ASSERT_TRUE( RegisterFlush( instance ) );
    // dump_flash_data_to_file();
}

void test_init( void )
{
    uintptr_t address = (uintptr_t)flash_data;
    TEST_ASSERT_EQUAL( address, RegisterStartAddress( instance ) );
    TEST_ASSERT_EQUAL( GetOffsetAddress( instance, 100 ), RegisterEndAddress( instance ) );
    TEST_ASSERT_EQUAL( 0, RegisterCount( instance ) );
}

void test_register_size( void )
{
    TEST_ASSERT_EQUAL(
        uavcan_register_Name_1_0_EXTENT_BYTES_ + uavcan_register_Value_1_0_EXTENT_BYTES_ + sizeof( bool ),
        RegisterSize( instance ) );
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

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    WriteFlashBytesExpectAndReturn( buffer, 1, HAL_ERROR );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    CallbackReturn = HAL_ERROR;
    uavcan_register_Value_1_0 value = { 0 };
    TEST_ASSERT_FALSE( RegisterAdd( instance, "test", &value, false ) );
}

void test_write_lock_fail( void )
{
    FlashRegister reg = { 0 };
    reg.isImmutable = false;
    reg.name.name.count = 4;
    memcpy( reg.name.name.elements, "test", 4 );

    size_t bufferSize = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[bufferSize];
    memset( buffer, 0, bufferSize );
    mattfair_storage_Register_1_0_serialize_( &reg, buffer, &bufferSize );

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_ERROR );
    uavcan_register_Value_1_0 value = { 0 };
    TEST_ASSERT_FALSE( RegisterAdd( instance, "test", &value, false ) );
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

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    WriteFlashBytesExpectAndReturn( buffer, bufferSize, HAL_OK );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    uavcan_register_Value_1_0 value = { 0 };
    TEST_ASSERT_TRUE( RegisterAdd( instance, "test", &value, false ) );
    TEST_ASSERT_EQUAL( 1, RegisterCount( instance ) );
}

void test_empty_GetRegisterNameByIndex( void )
{
    uavcan_register_Name_1_0 name = RegisterNameByIndex( instance, 0 );
    TEST_ASSERT_EQUAL( 0, name.name.count );
    TEST_ASSERT_EQUAL_STRING( "", name.name.elements );
}

void test_single_write_lookup_name_by_index( void )
{
    const char* regName = "test";
    AddRegisterNatural32Value( regName, 0 );

    uavcan_register_Name_1_0 name = RegisterNameByIndex( instance, 0 );
    TEST_ASSERT_EQUAL( 4, name.name.count );
    TEST_ASSERT_EQUAL_STRING_LEN( regName, name.name.elements, name.name.count );
}

void test_multiple_write_lookup_name_by_index( void )
{
    char* names[4] = { "foo", "bar", "abc", "def" };

    // first add the registers
    for ( size_t i = 0; i < 4; i++ )
    {
        AddRegisterNatural32Value( names[i], 0 );
    }

    // verify the name lookups
    for ( size_t i = 0; i < 4; i++ )
    {
        uavcan_register_Name_1_0 name = RegisterNameByIndex( instance, i );
        TEST_ASSERT_EQUAL( 3, name.name.count );

        bool found = false;
        for ( size_t j = 0; i < 4; i++ )
        {
            if ( strncmp( names[j], (const char*)name.name.elements, name.name.count ) == 0 )
            {
                found = true;
                break;
            }
        }
        TEST_ASSERT_TRUE( found );
    }
}

void test_single_rw_natural32_value( void )
{
    const char* name = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( name, answer );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( instance, name, &read ) );
    // TEST_ASSERT_EQUAL( 1, read.value.natural32.value.count );
    TEST_ASSERT_EQUAL( answer, read.value.natural32.value.elements[0] );
}

void test_single_rw_real64_value( void )
{
    const char* name = "pi";
    double pi = 3.14159;
    AddRegisterReal64Value( name, pi );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( instance, name, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( pi, read.value.real64.value.elements[0] );
}

void test_multiple_mixed_values_read_by_name( void )
{
    const char* piName = "pi";
    double pi = 3.14159265359;
    AddRegisterReal64Value( piName, pi );

    const char* tauName = "tau";
    double tau = 6.28318530718;
    AddRegisterReal64Value( tauName, tau );

    const char* phiName = "phi";
    double phi = 1.61803398875;
    AddRegisterReal64Value( phiName, phi );

    const char* actuatorName = "actuator_name";
    const char* actuatorValue = "linear_123";
    AddRegisterStringValue( actuatorName, actuatorValue );

    const char* answerName = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( answerName, answer );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( instance, piName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( pi, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( instance, tauName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( tau, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( instance, phiName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.real64.value.count );
    TEST_ASSERT_EQUAL( phi, read.value.real64.value.elements[0] );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( instance, actuatorName, &read ) );
    TEST_ASSERT_EQUAL( strlen( actuatorValue ), read.value._string.value.count );
    TEST_ASSERT_EQUAL_STRING( actuatorValue, read.value._string.value.elements );

    memset( &read, 0, sizeof( read ) );
    TEST_ASSERT_TRUE( RegisterRead( instance, answerName, &read ) );
    TEST_ASSERT_EQUAL( 1, read.value.natural32.value.count );
    TEST_ASSERT_EQUAL( answer, read.value.natural32.value.elements[0] );
}

void test_multiple_mixed_values_read_by_index( void )
{
    const char* piName = "pi";
    double pi = 3.14159265359;
    AddRegisterReal64Value( piName, pi );

    const char* tauName = "tau";
    double tau = 6.28318530718;
    AddRegisterReal64Value( tauName, tau );

    const char* phiName = "phi";
    double phi = 1.61803398875;
    AddRegisterReal64Value( phiName, phi );

    const char* actuatorName = "actuator_name";
    const char* actuatorValue = "linear_123";
    AddRegisterStringValue( actuatorName, actuatorValue );

    const char* answerName = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( answerName, answer );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterReadByIndex( instance, 0, &read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( instance, 1, &read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( instance, 2, &read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( instance, 3, &read ) );
    TEST_ASSERT_TRUE( RegisterReadByIndex( instance, 4, &read ) );
}

void test_write_single_string( void )
{
    AddRegisterStringValue( "test", "hello world" );
    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( instance, "test", &read ) );
    TEST_ASSERT_TRUE( uavcan_register_Value_1_0_is_string_( &read.value ) );
    TEST_ASSERT_EQUAL( 11, read.value._string.value.count );
    TEST_ASSERT_EQUAL_STRING( "hello world", read.value._string.value.elements );
}

void test_write_strings( void )
{
    for ( size_t i = 0; i < 10; i++ )
    {
        char name[10];
        char value[10];
        sprintf( name, "name_%d", i );
        sprintf( value, "value_%d", i );
        AddRegisterStringValue( name, value );
    }

    for ( size_t i = 0; i < 10; i++ )
    {
        char name[10];
        char value[10];
        sprintf( name, "name_%d", i );
        sprintf( value, "value_%d", i );

        FlashRegister read = { 0 };
        TEST_ASSERT_TRUE( RegisterRead( instance, name, &read ) );
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
            AddRegisterReal64Value( real_name, real_value );
        }
        else
        {
            char str_name[10];
            char str_value[10];
            sprintf( str_name, "string_%d", i );
            sprintf( str_value, "value_%d", i );
            AddRegisterStringValue( str_name, str_value );
        }
    }
    TEST_ASSERT_EQUAL( 20, RegisterCount( instance ) );

    for ( size_t i = 0; i < 20; i++ )
    {
        if ( i % 2 == 0 )
        {
            char real_name[10];
            sprintf( real_name, "real_%d", i );

            FlashRegister read = { 0 };
            TEST_ASSERT_TRUE( RegisterRead( instance, real_name, &read ) );
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
            TEST_ASSERT_TRUE( RegisterRead( instance, string_name, &read ) );
            TEST_ASSERT_EQUAL( strlen( string_value ), read.value._string.value.count );
            TEST_ASSERT_EQUAL_STRING( string_value, read.value._string.value.elements );
        }
    }
}

void test_create_immutable_register( void )
{
    const char* name = "immutable";
    AddImmutableRegisterNatural32Value( name, 42 );

    // verify the register is immutable
    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( instance, name, &read ) );
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

    TEST_ASSERT_FALSE( RegisterAdd( instance, name, &reg.value, false ) );
}

void test_factory_reset( void )
{
    const char* name = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( name, answer );

    TEST_ASSERT_EQUAL( 1, RegisterCount( instance ) );

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    RegisterFactoryReset( instance );

    TEST_ASSERT_EQUAL( 0, RegisterCount( instance ) );
}

void test_factory_reset_failure( void )
{
    const char* name = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( name, answer );

    TEST_ASSERT_EQUAL( 1, RegisterCount( instance ) );

    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );

    RegisterFactoryReset( instance );

    // should have not reset anything because there was an error
    TEST_ASSERT_EQUAL( 1, RegisterCount( instance ) );
}

void test_load_on_startup( void )
{
    const char* piName = "pi";
    double pi = 3.14159265359;
    AddRegisterReal64Value( piName, pi );

    const char* tauName = "tau";
    double tau = 6.28318530718;
    AddRegisterReal64Value( tauName, tau );

    const char* phiName = "phi";
    double phi = 1.61803398875;
    AddRegisterReal64Value( phiName, phi );

    const char* actuatorName = "actuator_name";
    const char* actuatorValue = "linear_123";
    AddRegisterStringValue( actuatorName, actuatorValue );

    const char* answerName = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( answerName, answer );

    TEST_ASSERT_EQUAL( 5, RegisterCount( instance ) );

    // shutdown
    RegisterDestroy( instance );
    TEST_ASSERT_EQUAL( 0, RegisterCount( instance ) );

    // reinitialize
    uintptr_t address = (uintptr_t)flash_data;
    RegisterInstance* newInit = RegisterInit( address, NUM_REGISTERS, heap, RegisterAllocate, RegisterFree );
    TEST_ASSERT_EQUAL( 5, RegisterCount( newInit ) );
}

void test_update_value( void )
{
    const char* name = "answer_to_everything";
    int answer = 42;
    AddRegisterNatural32Value( name, answer );

    FlashRegister read = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( instance, name, &read ) );
    TEST_ASSERT_EQUAL( answer, read.value.natural32.value.elements[0] );

    int newAnswer = 123456789;
    AddRegisterNatural32Value( name, newAnswer );

    FlashRegister newRead = { 0 };
    TEST_ASSERT_TRUE( RegisterRead( instance, name, &newRead ) );
    TEST_ASSERT_EQUAL_MESSAGE( newAnswer, newRead.value.natural32.value.elements[0], "value did not update" );
}
