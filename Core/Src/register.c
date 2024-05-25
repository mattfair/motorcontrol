/*
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Copyright (c) 2024 Matt Fair
 */
#include "register.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#define PASTE3_IMPL(x, y, z) x##y##z
#define PASTE3(x, y, z) PASTE3_IMPL(x, y, z)

#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define FLASH_USER_SECTOR FLASH_SECTOR_7
#define FLASH_USER_START_ADDR ADDR_FLASH_SECTOR_7
#define FLASH_SECTOR_SIZE 128 * 1024
#define FLASH_USER_END_ADDR (ADDR_FLASH_SECTOR_7 + FLASH_SECTOR_SIZE - 1)

typedef struct
{
  uavcan_register_Name_1_0 name;
  uavcan_register_Value_1_0 value;
} Register;

#define REGISTER_SIZE sizeof(Register)
#define REGISTER_MAX_COUNT ((FLASH_USER_END_ADDR - FLASH_USER_START_ADDR + 1) / REGISTER_SIZE)
#define REGISTER_FLASH_SIZE (REGISTER_MAX_COUNT * REGISTER_SIZE)

static uavcan_register_Name_1_0 registerNames[REGISTER_MAX_COUNT];
static int validRegisterCount = 0;

// Unlocks the flash memory for writing
static bool flash_unlock(void)
{
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  return true;
}

// Locks the flash memory after writing
static void flash_lock(void) { HAL_FLASH_Lock(); }

int serializeRegister(const Register *reg, uint8_t *buffer, size_t buffer_size)
{
  size_t size_used = 0;
  int8_t status;

  // Serialize Name
  size_t name_size = buffer_size - sizeof(uavcan_register_Value_1_0);
  status = uavcan_register_Name_1_0_serialize_(&reg->name, buffer, &name_size);
  if (status < 0)
  {
    printf("Failed to serialize name %d\r\n", status);
    return status; // Handle error
  }
  size_used += name_size;

  printf("Name %s, Size: %d, Used: %d\r\n", reg->name.name.elements, name_size, size_used);

  // Serialize Value
  size_t value_size = buffer_size - size_used;
  status = uavcan_register_Value_1_0_serialize_(&reg->value, buffer + size_used, &value_size);
  if (status < 0)
  {
    printf("Failed to serialize value %d\r\n", status);
    return status; // Handle error
  }
  size_used += buffer_size;

  return size_used; // Return the total size used in the buffer
}

int deserializeRegister(Register *reg, const uint8_t *buffer, size_t buffer_size)
{
  size_t size_consumed = 0;
  int8_t status;

  // Deserialize Name
  status = uavcan_register_Name_1_0_deserialize_(&reg->name, buffer, &buffer_size);
  if (status < 0) return status; // Handle error
  size_consumed += buffer_size;

  // Deserialize Value
  status = uavcan_register_Value_1_0_deserialize_(&reg->value, buffer + size_consumed, &buffer_size);
  if (status < 0) return status; // Handle error
  size_consumed += buffer_size;

  return size_consumed; // Return the total size consumed from the buffer
}

int findRegisterIndex(const char *name)
{
  // printf("Register size: %d\r\n", REGISTER_SIZE);
  // printf("Register max count: %ld\r\n", REGISTER_MAX_COUNT);
  // printf("Register flash size: %ld\r\n", REGISTER_FLASH_SIZE);
  printf("Finding register index for %s\r\n", name);

  if (validRegisterCount == 0)
  {
    // printf("No valid registers found\r\n");
    return -1;
  }

  for (int i = 0; i < validRegisterCount; i++)
  {
    uavcan_register_Name_1_0 *regName = &registerNames[i];
    if (strncmp(regName->name.elements, name, regName->name.count) == 0)
    {
      printf("Found Name: %s, Index: %d\r\n", name, i);
      return i;
    }
  }

  // printf("Name: %s, Index: -1\r\n", name);
  return -1; // Not found
}

// Function to calculate the address for a given register index
static uint32_t get_register_address(uint16_t index)
{
  if (index >= REGISTER_MAX_COUNT)
  {
    printf("index %d >= REGISTER_MAX_COUNT %ld\r\n", index, REGISTER_MAX_COUNT);
    return 0; // Invalid index
  }
  return FLASH_USER_START_ADDR + (index * REGISTER_SIZE);
}

// Function to read data from flash memory
static bool flash_read(uint32_t address, void *data, size_t size)
{
  if (address < FLASH_USER_START_ADDR || address + size > FLASH_USER_END_ADDR)
  {
    return false;
  }

  memcpy(data, (const void *)address, size);
  return true;
}

// Function to write data to flash memory
static bool flash_write(uint32_t address, const void *data, size_t size)
{
  if (address < FLASH_USER_START_ADDR || address + size > FLASH_USER_END_ADDR)
  {
    return false;
  }

  const uint8_t *data_ptr = (const uint8_t *)data;
  for (size_t i = 0; i < size; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data_ptr[i]) != HAL_OK)
    {
      return false;
    }
  }
  return true;
}

// Erases the flash memory sector where registers are stored
static bool flash_erase(void)
{
  FLASH_EraseInitTypeDef erase_init_struct;
  uint32_t page_error;
  erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init_struct.Sector = FLASH_USER_SECTOR;
  erase_init_struct.NbSectors = 1;
  erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK)
  {
    return false;
  }
  return true;
}

int createIndex(const char *name, const uavcan_register_Value_1_0 *value)
{
  int index = -1;
  printf("Creating index for %s\r\n", name);
  if (validRegisterCount >= REGISTER_MAX_COUNT)
  {
    printf("Max register count reached\r\n");
    return index;
  }

  uavcan_register_Name_1_0 registerName = {0};
  memcpy(&registerName.name.elements[0], name, strlen(name));
  registerName.name.count = strlen(name);
  memcpy(&registerNames[validRegisterCount], &registerName, REGISTER_SIZE);
  index = validRegisterCount++;

  return index;
}

// Reads the specified register from the persistent storage into `value`.
bool RegisterRead(const char *name, uavcan_register_Value_1_0 *value)
{
  assert(value != NULL);
  bool init_required = !uavcan_register_Value_1_0_is_empty_(value);

  printf("Reading register %s\r\n", name);
  int index = findRegisterIndex(name);
  if (index == -1)
  {
    index = createIndex(name, value);
    init_required = true;
  }

  uint32_t address = get_register_address(index);
  if (address == 0)
  {
    printf("Invalid address for register %s\r\n", name);
    return false;
  }

  uint8_t serialized[REGISTER_SIZE] = {0};
  size_t sr_size = REGISTER_SIZE;
  if (flash_read(address, &serialized[0], sr_size))
  {
    Register reg = {0};
    const int size = deserializeRegister(&reg, serialized, sr_size);
    if (size >= 0)
    {
      init_required |= !RegisterAssign(value, &reg.value);
    }
  }
  else
  {
    printf("could not read flash memory\r\n");
    return false;
  }

  if (init_required)
  {
    // Store the default value in the persistent storage
    return RegisterWrite(name, value);
  }

  return true;
}

// Store the given register value into the persistent storage.
bool RegisterWrite(const char *name, const uavcan_register_Value_1_0 *value)
{
  printf("Write register %s\r\n", name);
  int index = findRegisterIndex(name);

  if (index == -1)
  {
    index = createIndex(name, value);

    if (index == -1)
    {
      printf("Could not write register %s, index not found\r\n", name);
      return false;
    }
  }

  uint32_t address = get_register_address(index);
  if (address == 0)
  {
    printf("Invalid address for register %s\r\n", name);
    return false;
  }

  Register reg = {0};
  memcpy(&reg.name.name.elements[0], name, strlen(name));
  reg.name.name.count = strlen(name);
  memcpy(&reg.value, value, sizeof(uavcan_register_Value_1_0));

  uint8_t buffer[REGISTER_SIZE];
  size_t sr_size = serializeRegister(&reg, buffer, REGISTER_SIZE);

  if (sr_size > 0)
  {
    flash_unlock();
    bool success = flash_write(address, buffer, REGISTER_SIZE);
    flash_lock();

    printf("Writing register %s to index %d...\r\n", name, index);
    if (success)
    {
      printf("Write successful\r\n");
    }

    return success;
  }
  else
  {
    printf("Could not serialize register %s\r\n", name);
    return false;
  }
}

bool RegisterAssign(uavcan_register_Value_1_0 *const dst, const uavcan_register_Value_1_0 *const src)
{
  if (uavcan_register_Value_1_0_is_empty_(dst))
  {
    *dst = *src;
    return true;
  }
  if ((uavcan_register_Value_1_0_is_string_(dst) && uavcan_register_Value_1_0_is_string_(src)) ||
      (uavcan_register_Value_1_0_is_unstructured_(dst) && uavcan_register_Value_1_0_is_unstructured_(src)))
  {
    *dst = *src;
    return true;
  }
  if (uavcan_register_Value_1_0_is_bit_(dst) && uavcan_register_Value_1_0_is_bit_(src))
  {
    nunavutCopyBits(dst->bit.value.bitpacked, 0, nunavutChooseMin(dst->bit.value.count, src->bit.value.count), src->bit.value.bitpacked, 0);
    return true;
  }
  // This is a violation of MISRA/AUTOSAR but it is believed to be less error-prone than manually copy-pasted code.
#define REGISTER_CASE_SAME_TYPE(TYPE)                                                                             \
  if (PASTE3(uavcan_register_Value_1_0_is_, TYPE, _)(dst) && PASTE3(uavcan_register_Value_1_0_is_, TYPE, _)(src)) \
  {                                                                                                               \
    for (size_t i = 0; i < nunavutChooseMin(dst->TYPE.value.count, src->TYPE.value.count); ++i)                   \
    {                                                                                                             \
      dst->TYPE.value.elements[i] = src->TYPE.value.elements[i];                                                  \
    }                                                                                                             \
    return true;                                                                                                  \
  }
  REGISTER_CASE_SAME_TYPE(integer64)
  REGISTER_CASE_SAME_TYPE(integer32)
  REGISTER_CASE_SAME_TYPE(integer16)
  REGISTER_CASE_SAME_TYPE(integer8)
  REGISTER_CASE_SAME_TYPE(natural64)
  REGISTER_CASE_SAME_TYPE(natural32)
  REGISTER_CASE_SAME_TYPE(natural16)
  REGISTER_CASE_SAME_TYPE(natural8)
  REGISTER_CASE_SAME_TYPE(real64)
  REGISTER_CASE_SAME_TYPE(real32)
  REGISTER_CASE_SAME_TYPE(real16)
  return false;
}

uavcan_register_Name_1_0 RegisterGetNameByIndex(const uint16_t index)
{
  Register reg;
  uint32_t address = FLASH_USER_START_ADDR + REGISTER_SIZE * index;
  uint16_t buffer_size = sizeof(Register);
  uint8_t buffer[buffer_size];

  memcpy(buffer, (void *)address, buffer_size);
  deserializeRegister(&reg, buffer, buffer_size);

  if (reg.name.name.count == 255)
  {
    uavcan_register_Name_1_0 empty;
    uavcan_register_Name_1_0_initialize_(&empty);
    return empty;
  }

  return reg.name;
}

// Erase all registers such that the defaults are used at the next launch.
void RegisterDoFactoryReset(void)
{
  flash_unlock();
  flash_erase();
  flash_lock();
}

// Function to read register data from flash
void RegisterInit(void)
{
  uint16_t buffer_size = sizeof(Register);
  uint8_t buffer[buffer_size];
  Register reg;
  uint32_t address = FLASH_USER_START_ADDR;

  // TODO: remove
  // RegisterDoFactoryReset();

  for (int i = 0; i < REGISTER_MAX_COUNT; i++)
  {
    memcpy(buffer, (void *)address, buffer_size);
    deserializeRegister(&reg, buffer, buffer_size);

    if (reg.name.name.count == 255)
    {
      printf("Found %d registers\r\n", i);
      break; // End of valid data
    }

    printf("Register %d: %.*s\r\n", i, reg.name.name.count, reg.name.name.elements);

    registerNames[i] = reg.name;
    validRegisterCount++;
    address += buffer_size;
  }
}
