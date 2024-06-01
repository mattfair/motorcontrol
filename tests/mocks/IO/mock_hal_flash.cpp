#include <IO/hal.h>
#include <CppUTestExt/MockSupport.h>

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  return (HAL_StatusTypeDef)mock()
      .actualCall("HAL_FLASH_Program")
      .withParameter("TypeProgram", TypeProgram)
      .withParameter("Address", Address)
      .withParameter("Data", Data)
      .returnIntValue();
}

HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  return (HAL_StatusTypeDef)mock()
      .actualCall("HAL_FLASH_Program_IT")
      .withParameter("TypeProgram", TypeProgram)
      .withParameter("Address", Address)
      .withParameter("Data", Data)
      .returnIntValue();
}

void HAL_FLASH_IRQHandler(void)
{
  mock().actualCall("HAL_FLASH_IRQHandler");
}

void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  mock().actualCall("HAL_FLASH_EndOfOperationCallback").withParameter("ReturnValue", ReturnValue);
}

void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
  mock().actualCall("HAL_FLASH_OperationErrorCallback").withParameter("ReturnValue", ReturnValue);
}

HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
  return (HAL_StatusTypeDef)mock().actualCall("HAL_FLASH_Unlock").returnIntValue();
}

HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
  return (HAL_StatusTypeDef)mock().actualCall("HAL_FLASH_Lock").returnIntValue();
}

HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void)
{
  return (HAL_StatusTypeDef)mock().actualCall("HAL_FLASH_OB_Unlock").returnIntValue();
}

HAL_StatusTypeDef HAL_FLASH_OB_Lock(void)
{
  return (HAL_StatusTypeDef)mock().actualCall("HAL_FLASH_OB_Lock").returnIntValue();
}

HAL_StatusTypeDef HAL_FLASH_OB_Launch(void)
{
  return (HAL_StatusTypeDef)mock().actualCall("HAL_FLASH_OB_Launch").returnIntValue();
}

uint32_t HAL_FLASH_GetError(void)
{
  return (uint32_t)mock().actualCall("HAL_FLASH_GetError").returnUnsignedIntValue();
}

HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout)
{
  return (HAL_StatusTypeDef)mock().actualCall("FLASH_WaitForLastOperation").withParameter("Timeout", Timeout).returnIntValue();
}
