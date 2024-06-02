/*
 * Read/Write/Erase persistent flash memory using HAL library
 */

#include <IO/flash.h>
#include <string.h>
#include <unity.h>
#include "mock_stm32f4xx_hal_flash.h"
#include "mock_stm32f4xx_hal_flash_ex.h"

#define TEST_CASE( ... )

static bool flags_cleared;
void ( *flash_clear_flags_hw )( void );

uint8_t flash_data[1024];

void mock_clear_flags( void )
{
    flags_cleared = true;
}

void setUp( void )
{
    // Clear the flash data
    for ( size_t i = 0; i < sizeof( flash_data ); i++ )
    {
        flash_data[i] = 0xFF;
    }

    flash_clear_flags_hw = flash_clear_flags;
    flash_clear_flags = mock_clear_flags;

    uintptr_t address = (uintptr_t)flash_data;
    flash_init( address, 1024 );
    flags_cleared = false;
}

void tearDown( void )
{
    flash_destroy();
    flash_clear_flags = flash_clear_flags_hw;
}

void test_flash_init( void )
{
    TEST_ASSERT_EQUAL_INT( FLASH_OK, flash_init( 100, 70 ) );
}

void test_flash_get_addr( void )
{
    uintptr_t address = (uintptr_t)flash_data;
    TEST_ASSERT_EQUAL_INT( address, flash_get_addr() );
}

void test_flash_get_size( void )
{
    TEST_ASSERT_EQUAL_INT( 1024, flash_get_size() );
}

void test_flash_lock_OK( void )
{
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );
    TEST_ASSERT_EQUAL( FLASH_OK, flash_lock() );
    TEST_ASSERT_FALSE( flags_cleared );
}

TEST_CASE( HAL_ERROR )
TEST_CASE( HAL_BUSY )
TEST_CASE( HAL_TIMEOUT )
void test_flash_lock( HAL_StatusTypeDef status )
{
    HAL_FLASH_Lock_ExpectAndReturn( status );
    TEST_ASSERT_EQUAL( FLASH_ERROR, flash_lock() );
    TEST_ASSERT_FALSE( flags_cleared );
}

void test_flash_unlock_OK( void )
{
    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    TEST_ASSERT_EQUAL( FLASH_OK, flash_unlock() );
    TEST_ASSERT_TRUE( flags_cleared );
}

void test_flash_unlock_ERROR( void )
{
    HAL_FLASH_Unlock_ExpectAndReturn( HAL_ERROR );
    TEST_ASSERT_EQUAL( FLASH_ERROR, flash_unlock() );
    TEST_ASSERT_FALSE( flags_cleared );
}

void test_lock_again( void )
{
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );
    TEST_ASSERT_EQUAL( FLASH_OK, flash_lock() );
    TEST_ASSERT_FALSE( flags_cleared );
}

void test_flash_lock_unlock_lock( void )
{
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );
    HAL_FLASH_Unlock_ExpectAndReturn( HAL_OK );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    TEST_ASSERT_EQUAL( FLASH_OK, flash_lock() );
    TEST_ASSERT_FALSE( flags_cleared );
    TEST_ASSERT_EQUAL( FLASH_OK, flash_unlock() );
    TEST_ASSERT_TRUE( flags_cleared );
    flags_cleared = false;
    TEST_ASSERT_EQUAL( FLASH_OK, flash_lock() );
    TEST_ASSERT_FALSE( flags_cleared );
}

void test_flash_lock_unlock_fail_lock( void )
{
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );
    HAL_FLASH_Unlock_ExpectAndReturn( HAL_ERROR );
    HAL_FLASH_Lock_ExpectAndReturn( HAL_OK );

    TEST_ASSERT_EQUAL( FLASH_OK, flash_lock() );
    TEST_ASSERT_FALSE( flags_cleared );
    TEST_ASSERT_EQUAL( FLASH_ERROR, flash_unlock() );
    TEST_ASSERT_FALSE( flags_cleared );
    TEST_ASSERT_EQUAL( FLASH_OK, flash_lock() );
    TEST_ASSERT_FALSE( flags_cleared );
}

TEST_CASE( HAL_OK, FLASH_OK )
TEST_CASE( HAL_ERROR, FLASH_ERROR )
TEST_CASE( HAL_BUSY, FLASH_ERROR )
TEST_CASE( HAL_TIMEOUT, FLASH_ERROR )
void test_flash_erase( HAL_StatusTypeDef status, FlashStatus expected )
{
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Banks = 0;
    uint32_t page_error = 0;
    HAL_FLASHEx_Erase_ExpectAndReturn( &erase, &page_error, status );
    TEST_ASSERT_EQUAL( expected, flash_erase() );
}

void test_flash_read_out_of_bounds_start( void )
{
    uint8_t data[4];
    uint32_t address = flash_get_addr();
    TEST_ASSERT_EQUAL(FLASH_ERROR, flash_read( &data, address - 1, 4 ));
}

void test_flash_read_out_of_bounds_end( void )
{
    uint8_t data[4];
    uint32_t address = flash_get_addr();
    size_t size = flash_get_size();
    TEST_ASSERT_EQUAL(FLASH_ERROR, flash_read( &data, address + size, 4 ));
}

void test_flash_read_empty( void )
{
    uint8_t data[4];
    uint32_t address = flash_get_addr();
    flash_read( &data, address, 4 );

    for ( size_t i = 0; i < 4; i++ )
    {
        TEST_ASSERT_EQUAL_UINT8( 0xFF, data[i] );
    }
}

TEST_CASE( 0 )
TEST_CASE( 10 )
TEST_CASE( 100 )
TEST_CASE( 1000 )
void test_flash_read( uint32_t offset)
{
   //let's put some data in the flash
   for ( size_t i = 0; i < sizeof(flash_data); i++ )
   {
       flash_data[i] = i % 256;
   }

    //read a chunk of data
    uint8_t data[4];
    uint32_t address = flash_get_addr() + offset;
    flash_read( &data, address, 4 );

    for ( size_t i = 0; i < 4; i++ )
    {
        uint8_t expected = (i + offset) % 256;
        TEST_ASSERT_EQUAL_UINT8( expected, data[i] );
    }

    // read should be successful
    TEST_ASSERT_EQUAL(FLASH_OK, flash_read(&data, address, 4));
}

TEST_CASE( 0 )
TEST_CASE( 1 )
TEST_CASE( 2 )
TEST_CASE( 3 )
void test_flash_write_with_error( uint16_t failOnByte )
{
    uint8_t data[4] = { 0x01, 0x02, 0x03, 0x04 };
    size_t size = sizeof( data );
    uint32_t address = flash_get_addr();

    if ( failOnByte > size )
    {
        TEST_FAIL_MESSAGE( "Invalid test case" );
    }

    // Expect that the flash will write sequentially
    for ( size_t i = 0; i < size; i++ )
    {
        if ( i == failOnByte )
        {
            // and error occures
            HAL_FLASH_Program_ExpectAndReturn( FLASH_TYPEPROGRAM_BYTE, address + i, data[i], HAL_ERROR );

            // no more writes should occur
            break;
        }
        else
        {
            HAL_FLASH_Program_ExpectAndReturn( FLASH_TYPEPROGRAM_BYTE, address + i, data[i], HAL_OK );
        }
    }

    // the end result should be an error
    TEST_ASSERT_EQUAL( FLASH_ERROR, flash_write( &data, address, size ) );
}

void test_flash_success( void )
{
    uint8_t data[4] = { 0x01, 0x02, 0x03, 0x04 };
    size_t size = sizeof( data );
    uint32_t address = flash_get_addr();

    for ( size_t i = 0; i < size; i++ )
    {
        HAL_FLASH_Program_ExpectAndReturn( FLASH_TYPEPROGRAM_BYTE, address + i, data[i], HAL_OK );
    }

    TEST_ASSERT_EQUAL( FLASH_OK, flash_write( &data, address, size ) );
}

void test_flash_write_out_of_bounds_start_should_fail( void )
{

    uint8_t data[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint32_t address = flash_get_addr() - 1;
    TEST_ASSERT_EQUAL( FLASH_ERROR, flash_write( &data, address, 4 ) );
}

void test_flash_write_out_of_bounds_end_should_fail( void )
{
    uint8_t data[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint32_t address = flash_get_addr();
    size_t size = flash_get_size();
    TEST_ASSERT_EQUAL( FLASH_ERROR, flash_write( &data, address + size, 4 ) );
}
