#include <IO/register.h>
#include <unity.h>

/**
 * Specify the address and size for the register memory
 * Load register names into a list
 * Look up register index by name
 * Read and write register values
 * Mark registers as immutable
 * Clear all register values
 */


void setup(void)
{
}

void teardown(void)
{
}

void test_RegisterInit(void)
{
    RegisterInit(0x080000, 1000);

    uintptr_t address = (uintptr_t)0x080000;
    TEST_ASSERT_EQUAL(address, RegisterGetStartAddress());
    TEST_ASSERT_EQUAL(50, RegisterCount());
}
