/*
 * Read/Write/Erase persistent flash memory using HAL library
 */

#include <CppUTest/CommandLineTestRunner.h>
#include <IO/flash.h>
#include <string.h>

TEST_GROUP(FlashTest)
{
 protected:
  char data[10] = "123456789";
  size_t size;
  uint32_t address;

  void setup() override
  {
    size = sizeof(data);
    address = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&data));
    flash_init(address, 10);
  }

  void teardown() override
  {
  }
};

TEST(FlashTest, InitializeMemory)
{
  CHECK_EQUAL(FLASH_OK, flash_init(address, size));
}

TEST(FlashTest, GetAddress)
{
  CHECK_EQUAL(address, flash_get_addr());
}

TEST(FlashTest, EraseMemory)
{
  CHECK_EQUAL(FLASH_OK, flash_erase());

  // verify memory erases to 0xFF
  char expected[10];
  memset(expected, 0xFF, sizeof(expected));
  CHECK_EQUAL(0, strncmp(expected, data, 10));
}

TEST(FlashTest, DefaultLockStatus)
{
  CHECK_EQUAL(FLASH_LOCKED, flash_get_status());
}

TEST(FlashTest, LockMemoryAgain)
{
  CHECK_EQUAL(FLASH_LOCKED, flash_get_status());
  flash_lock();
  CHECK_EQUAL(FLASH_LOCKED, flash_get_status());
}

TEST(FlashTest, LockUnlock)
{
  CHECK_EQUAL(FLASH_LOCKED, flash_get_status());
  flash_unlock();
  CHECK_EQUAL(FLASH_UNLOCKED, flash_get_status());
}
