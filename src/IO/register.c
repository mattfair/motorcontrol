/*
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Copyright (c) 2024 Matt Fair
 */
#include "IO/register.h"
#include <stdio.h>
#include "IO/flash.h"

static uint32_t register_start_addr = 0;
static uint32_t register_end_addr = 0;
static size_t memory_size = 0;
static size_t register_size = 0;
static size_t register_count = 0;
static uavcan_register_Name_1_0* registerNames;
static uint16_t registerNamesCount = 0;
static uint8_t serializeBuffer[mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };

uint32_t GetOffsetAddress( uint32_t index );
int32_t FindRegisterIndex( const char* name );

void RegisterInit( uint32_t start_addr, size_t count )
{
    register_start_addr = start_addr;
    register_size = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    memory_size = register_size * count;
    register_end_addr = start_addr + count * register_size;
    register_count = 0;
    registerNamesCount = 0;
    registerNames =
        (uavcan_register_Name_1_0*)malloc( count * uavcan_register_Name_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ );
    flash_init( start_addr, memory_size );
}

void RegisterDestroy()
{
    register_start_addr = 0;
    register_end_addr = 0;
    register_size = 0;
    register_count = 0;
    free( registerNames );
    flash_destroy();
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

bool RegisterWrite( const char* name, const uavcan_register_Value_1_0* value )
{
    uavcan_register_Name_1_0* storedName = &registerNames[registerNamesCount++];
    uavcan_register_Name_1_0_initialize_( storedName );
    storedName->name.count = strlen( name );
    memset( storedName->name.elements, 0, sizeof( storedName->name.elements ) );
    memcpy( storedName->name.elements, name, storedName->name.count );

    FlashRegister reg =
    {
      .name = *storedName,
      .value = *value,
      .isImmutable = false
    };

    size_t size = sizeof( reg );
    memset( serializeBuffer, 0, size);
    if ( mattfair_storage_Register_1_0_serialize_( &reg, serializeBuffer, &size ) != NUNAVUT_SUCCESS )
    {
        return false;
    }

    if ( flash_write( serializeBuffer, GetOffsetAddress( register_count ), RegisterSize() ) == FLASH_ERROR )
    {
        return false;
    }

    register_count++;
    return true;
}

int32_t FindRegisterIndex( const char* name )
{
    if ( registerNamesCount == 0 )
    {
        return -1;
    }

    for ( int i = 0; i < registerNamesCount; i++ )
    {
        uavcan_register_Name_1_0* regName = &registerNames[i];
        if ( strncmp( (const char*)regName->name.elements, name, regName->name.count ) == 0 )
        {
            //printf( "Found Name: %s, Index: %d\r\n", name, i );
            return i;
        }
    }

    return -1;  // Not found
}

int8_t DeserializeFlashRegister( void* const out_obj, const uint8_t* buffer, size_t* const inout_buffer_size_bytes )
{
    return mattfair_storage_Register_1_0_deserialize_( out_obj, buffer, inout_buffer_size_bytes );
}

bool RegisterRead( const char* name, FlashRegister* regOut )
{
    int32_t index = FindRegisterIndex( name );
    if ( index == -1 )
    {
        return false;
    }

    if ( flash_read( regOut, GetOffsetAddress( (uint32_t)index ), RegisterSize(), DeserializeFlashRegister ) ==
         FLASH_ERROR )
    {
        return false;
    }

    return true;
}

uavcan_register_Name_1_0 RegisterNameByIndex( const uint16_t index )
{
    if ( index >= registerNamesCount )
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

#if 0
#include <assert.h>
#include <stdalign.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flash.h"

#define PASTE3_IMPL( x, y, z ) x##y##z
#define PASTE3( x, y, z ) PASTE3_IMPL( x, y, z )

#define FLASH_USER_SECTOR FLASH_SECTOR_7

typedef struct
{
  uavcan_register_Name_1_0 name;
  uavcan_register_Value_1_0 value;
} Register;

const uint32_t registerSize = sizeof(Register);
const uint32_t REGISTER_MAX_COUNT = ((FLASH_USER_END_ADDR - FLASH_USER_START_ADDR + 1) / registerSize);

static int validRegisterCount = 0;
static uint32_t maxRegisterCount = 0;
static uavcan_register_Name_1_0 registerNames[maxRegisterCount];

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

// Function to calculate the address for a given register index
static uint32_t get_register_address(uint16_t index)
{
  if (index >= maxRegisterCount)
  {
    printf("index %d >= maxRegisterCount %ld\r\n", index, maxRegisterCount);
    return 0; // Invalid index
  }
  return FLASH_USER_START_ADDR + (index * registerSize);
}


int createIndex(const char *name, const uavcan_register_Value_1_0 *value)
{
  int index = -1;
  printf("Creating index for %s\r\n", name);
  if (validRegisterCount >= maxRegisterCount)
  {
    printf("Max register count reached\r\n");
    return index;
  }

  uavcan_register_Name_1_0 registerName = {0};
  memcpy(&registerName.name.elements[0], name, strlen(name));
  registerName.name.count = strlen(name);
  memcpy(&registerNames[validRegisterCount], &registerName, registerSize);
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
  size_t buffer_size = registerSize;
  uint8_t buffer[buffer_size];
  if (flash_read(address, &buffer[0], buffer_size))
  {
    Register reg = {0};
    assert(address != 0);
    memcpy(buffer, (void *)address, buffer_size);
    int status = deserializeRegister(&reg, buffer, &buffer_size);
    if (status >= 0)
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

bool register_write(const char *name, const uavcan_register_Value_1_0 *value, bool imutable)
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

  Register reg = {0};
  uint32_t address = get_register_address(index);
  size_t buffer_size = registerSize;
  uint8_t buffer[buffer_size];

  assert(address != 0);
  memcpy(buffer, (void *)address, buffer_size);
  int status = deserializeRegister(&reg, buffer, buffer_size);
  if (status < 0)
  {
    printf("Could not deserialize register %s\r\n", name);
    return false;
  }

  // Check if the register is immutable
  //if (reg.isImmutable)
  //{
  //  printf("Attempt to write immutable register %s denied\r\n", name);
  //  return false;
 // }

  if (reg.name.name.count == 255)
  {
    reg.name = registerNames[index];
  }

  status = serializeRegister(&reg, buffer, &buffer_size);
  if (status > 0)
  {
    flash_unlock();
    bool success = flash_write(address, buffer, registerSize);
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

// Store the given register value into the persistent storage.
bool RegisterWrite(const char *name, const uavcan_register_Value_1_0 *value) { return register_write(name, value, false); }

// Store the given register value into the persistent storage and mark it as immutable.
bool RegisterImutableWrite(const char *name, const uavcan_register_Value_1_0 *value) { return register_write(name, value, true); }

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
  uint32_t address = flash_get_start_addr() + registerSize * index;
  size_t buffer_size = registerSize;
  uint8_t buffer[buffer_size];

  memcpy(buffer, (void *)address, buffer_size);
  int status = deserializeRegister(&reg, buffer, buffer_size);
  if (status < 0)
  {
    printf("Failed to deserialize register %d\r\n", index);
    uavcan_register_Name_1_0 empty;
    uavcan_register_Name_1_0_initialize_(&empty);
    return empty;
  }

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
void RegisterInit(uint32_t start_addr, uint32_t end_addr)
{
  init_flash(start_addr, end_addr);
  maxRegisterCount = (end_addr - start_addr + 1) / registerSize;

  Register reg = {0};;
  size_t buffer_size = registerSize;
  uint8_t buffer[buffer_size];
  uint32_t address = start_addr;

  // TODO: remove
  //RegisterDoFactoryReset();

  for (int i = 0; i < maxRegisterCount; i++)
  {
    memcpy(buffer, (void *)address, buffer_size);
    int status = deserializeRegister(&reg, buffer, &buffer_size);
    if (status < 0 || reg.name.name.count == 255)
    {
      //printf("Found %d registers\r\n", i);
      break; // End of valid data
    }

    printf("Register %d: %.*s\r\n", i, reg.name.name.count, reg.name.name.elements);

    registerNames[i] = reg.name;
    validRegisterCount++;
    address += buffer_size;
  }
}
#endif
