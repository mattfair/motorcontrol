#include <gtest/gtest.h>
#include <IO/register.h>

/**
 * Specify the address and size for the register memory
 * Load register names into a list
 * Look up register index by name
 * Read and write register values
 * Mark registers as immutable
 * Clear all register values
 */


class RegisterTest : public ::testing::Test
{
 protected:
  uint8_t memory[1024];
  uint32_t startAddress;
  uint32_t endAddress;
  void SetUp() override
  {
    startAddress = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&memory));
    endAddress = startAddress + sizeof(memory);
  }

  void TearDown() override
  {
    // Nothing to do
  }
};

TEST_F(RegisterTest, DefaultToUninitialized) { ASSERT_FALSE(RegisterIsInitialized()); }

TEST_F(RegisterTest, InitialLoad)
{
  RegisterInit(startAddress, endAddress);
  ASSERT_TRUE(RegisterIsInitialized());
}

TEST_F(RegisterTest, GetAddress)
{
  RegisterInit(startAddress, endAddress);
  ASSERT_EQ(startAddress, RegisterGetStart());
  ASSERT_EQ(endAddress, RegisterGetEnd());
}

