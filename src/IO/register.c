/*
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Copyright (c) 2024 Matt Fair
 */
#include "IO/register.h"
#include <inttypes.h>
#include <o1heap.h>
#include <stdio.h>
#include "Core/Inc/main.h"
#include "IO/flash.h"

#define PASTE3_IMPL( x, y, z ) x##y##z
#define PASTE3( x, y, z ) PASTE3_IMPL( x, y, z )

#define REGISTER_ERROR_OUT_OF_MEMORY 1
#define REGISTER_ERROR_IMMUTABLE 2

// private functions
void clearTree( RegisterInstance* inst, RegisterTree* tree );
RegisterTreeItem* registerAllocateQueueItem( RegisterInstance* const ins, const FlashRegister* reg );
bool serializeAndWriteTree( RegisterTreeItem* item, uint32_t* address );
void updateIndexToNodeMap( RegisterInstance* instance, RegisterTreeItem* item, size_t* index );

Cavl* avlTrivialRegisterFactory( void* const user_reference );
int8_t indexByNameAVLPredicate( void* const user_reference, const RegisterTreeNode* const node );
int8_t searchByNameAVLPredicate( void* const user_reference, const RegisterTreeNode* const node );
int32_t registerPush( RegisterInstance* inst, FlashRegister* reg );
RegisterTreeItem* registerPop( RegisterTree* const que, const RegisterTreeItem* item );
const RegisterTreeItem* getItemByIndex( RegisterInstance* inst, uint32_t index );
void printTreeNode( const RegisterTreeNode* node );

uint32_t GetOffsetAddress( RegisterInstance* inst, uint32_t index );
void ReadRegisters( RegisterInstance* inst );
int8_t DeserializeFlashRegister( void* const out_obj, const uint8_t* buffer, size_t* const inout_buffer_size_bytes );

// static elements
uint8_t serializeBuffer[1024];

Cavl* avlTrivialRegisterFactory( void* const user_reference )
{
    return (RegisterTreeNode*)user_reference;
}

void clearTree( RegisterInstance* inst, RegisterTree* tree )
{
    assert( inst != NULL );
    while ( tree->size > 0 )
    {
        const RegisterTreeItem* root = (const RegisterTreeItem*)(void*)tree->root;
        if ( root == NULL )
        {
            break;
        }
        RegisterTreeItem* item = registerPop( tree, root );
        inst->memory_free( inst, item );
    }
}

/// The item is only allocated and initialized, but NOT included into the queue! The caller needs to do that.
RegisterTreeItem* registerAllocateQueueItem( RegisterInstance* const ins, const FlashRegister* reg )
{
    assert( ins != NULL );

    RegisterTreeItem* const out = (RegisterTreeItem*)ins->memory_allocate( ins, sizeof( RegisterTreeItem ) );
    if ( out != NULL )
    {
        out->base.up = NULL;
        out->base.lr[0] = NULL;
        out->base.lr[1] = NULL;
        out->base.bf = 0;
        out->value = *reg;
    }
    printf( "Allocated item: %p\r\n", (void*)out );
    return out;
}
int8_t indexByNameAVLPredicate( void* const user_reference, const RegisterTreeNode* const node )
{
    const RegisterTreeItem* target = (const RegisterTreeItem*)user_reference;
    const RegisterTreeItem* const other = (const RegisterTreeItem*)(const void*)node;
    assert( ( target != NULL ) && ( other != NULL ) );
    return (int8_t)strcmp( (const char*)target->value.name.name.elements,
                           (const char*)other->value.name.name.elements );
}

int8_t searchByNameAVLPredicate( void* const user_reference, const RegisterTreeNode* const node )
{
    const char* target = (const char*)user_reference;
    const RegisterTreeItem* const other = (const RegisterTreeItem*)(const void*)node;
    assert( ( target != NULL ) && ( other != NULL ) );
    return (int8_t)strcmp( target, (const char*)other->value.name.name.elements );
}

void printTreeNode( const RegisterTreeNode* node )
{
    if ( node == NULL )
    {
        return;
    }

    printf( "Node:%p, up:%p, left:%p, right:%p, bf:%d\r\n",
            (void*)node,
            (void*)node->up,
            (void*)node->lr[0],
            (void*)node->lr[1],
            node->bf );

    const RegisterTreeItem* item = (const RegisterTreeItem*)node;
    printf( "Name: %.*s\r\n", item->value.name.name.count, item->value.name.name.elements );
    PrintValue( &item->value.value );

    printTreeNode( (const RegisterTreeNode*)node->lr[0] );
    printTreeNode( (const RegisterTreeNode*)node->lr[1] );
}

int32_t registerPush( RegisterInstance* inst, FlashRegister* reg )
{
    int32_t out = 0;

    //printf( "pushing size %d, capacity %d\r\n", inst->registersByName.size, inst->registersByName.capacity );
    RegisterTreeItem* const item =
        inst->registersByName.size < inst->registersByName.capacity ? registerAllocateQueueItem( inst, reg ) : NULL;

    if ( item != NULL )
    {
        // Insert the newly created register item into the proper AVL trees.

        // printf( "Name Before:\r\n" );
        // printTreeNode( inst->registersByName.root );
        const RegisterTreeNode* const nameNode = cavlSearch(
            &inst->registersByName.root, (void*)&item->base, &indexByNameAVLPredicate, &avlTrivialRegisterFactory );

        assert( inst->registersByName.root != NULL );
        assert( inst->registersByName.size <= inst->registersByName.capacity );

        if ( nameNode != &item->base )
        {
            printf( "Node already exists, updating...\r\n" );
            inst->memory_free( inst, item );
            RegisterTreeItem* existingItem = (RegisterTreeItem*)(void*)nameNode;

            if ( existingItem->value.isImmutable )
            {
                printf( "Cannot update immutable register %.*s\r\n",
                        existingItem->value.name.name.count,
                        existingItem->value.name.name.elements );
                return -REGISTER_ERROR_IMMUTABLE;
            }
            existingItem->value = *reg;
        }
        else
        {
            inst->registersByName.size++;
        }

        // printf( "Name After:\r\n" );
        // printTreeNode( inst->registersByName.root );

        // reindex the registers by index
        size_t index = 0;
        memset( inst->registersByIndex, 0, sizeof( uint32_t ) * inst->registersByName.size );
        RegisterTreeItem* rootItem = (RegisterTreeItem*)(void*)inst->registersByName.root;
        updateIndexToNodeMap( inst, rootItem, &index );

        //printf( "Pushed item: %p\r\n", (void*)item );
        out = 1;
    }
    else
    {
        printf( "Out of memory\r\n" );
        out = -REGISTER_ERROR_OUT_OF_MEMORY;
    }
    assert( ( out < 0 ) || ( out == 1 ) );
    return out;
}

RegisterTreeItem* registerPop( RegisterTree* const que, const RegisterTreeItem* item )
{
    RegisterTreeItem* out = NULL;
    if ( ( que != NULL ) && ( item != NULL ) )
    {
        // Intentional violation of MISRA: casting away const qualifier. This is considered safe because the API
        // contract dictates that the pointer shall point to a mutable entity in RAM previously allocated by the
        // memory manager. It is difficult to avoid this cast in this context.
        out = (RegisterTreeItem*)item;  // NOSONAR casting away const qualifier.

        // Paragraph 6.7.2.1.15 of the C standard says:
        //     A pointer to a structure object, suitably converted, points to its initial member, and vice versa.
        // Note that the highest-priority frame is always a leaf node in the AVL tree, which means that it is very
        // cheap to remove.
        cavlRemove( &que->root, &item->base );
        que->size--;
    }
    return out;
}

void updateIndexToNodeMap( RegisterInstance* instance, RegisterTreeItem* item, size_t* index )
{
    if ( item == NULL )
    {
        // found leaf node
        return;
    }

    instance->registersByIndex[*index] = (uint32_t)(uintptr_t)item;
    ( *index )++;

    RegisterTreeNode* node = &item->base;
    RegisterTreeItem* leftItem = (RegisterTreeItem*)(const void*)node->lr[0];
    RegisterTreeItem* rightItem = (RegisterTreeItem*)(const void*)node->lr[1];

    updateIndexToNodeMap( instance, leftItem, index );
    updateIndexToNodeMap( instance, rightItem, index );
}

bool serializeAndWriteTree( RegisterTreeItem* item, uint32_t* address )
{
    if ( item == NULL || address == NULL )
    {
        return true;
    }

    //printf( "Serializing item %.*s\r\n", item->value.name.name.count, item->value.name.name.elements );
    size_t size = sizeof( serializeBuffer );
    mattfair_storage_Register_1_0_serialize_( &item->value, serializeBuffer, &size );
    assert( size <= sizeof( serializeBuffer ) );
    if ( flash_write( serializeBuffer, *address, size ) != FLASH_OK )
    {
        return false;
    }
    *address += size;

    RegisterTreeNode* node = &item->base;
    RegisterTreeItem* leftItem = (RegisterTreeItem*)(const void*)node->lr[0];
    RegisterTreeItem* rightItem = (RegisterTreeItem*)(const void*)node->lr[1];

    if ( !serializeAndWriteTree( leftItem, address ) )
    {
        return false;
    }
    return serializeAndWriteTree( rightItem, address );
}

RegisterInstance* RegisterInit( uint32_t start_addr,
                                size_t count,
                                O1HeapInstance* heap,
                                RegisterMemoryAllocate memory_allocate,
                                RegisterMemoryFree memory_free )
{
    RegisterInstance* inst = (RegisterInstance*)malloc( sizeof( RegisterInstance ) );
    assert( inst != NULL );

    RegisterState* state = malloc( sizeof( RegisterState ) );
    state->register_start_addr = start_addr;
    state->register_size = mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    state->memory_size = state->register_size * count;
    state->register_end_addr = start_addr + count * state->register_size;
    state->max_register_count = count;

    inst->user_reference = state;

    inst->registersByName.size = 0;
    inst->registersByName.capacity = count;
    inst->registersByName.root = NULL;

    inst->registersByIndex = (uint32_t*)malloc( sizeof( uint32_t ) * count );

    inst->memory_allocate = memory_allocate;
    inst->memory_free = memory_free;
    inst->heap = heap;
    assert( inst->heap != NULL );

    flash_init( state->register_start_addr, state->memory_size );

    // RegisterFactoryReset(inst);
    ReadRegisters( inst );
    // printTreeNode( inst->registersByName.root );

    return inst;
}

void RegisterDestroy( RegisterInstance* inst )
{
    clearTree( inst, &inst->registersByName );

    flash_destroy();
    free( inst->registersByIndex );
    free( (RegisterState*)inst->user_reference );
    free( inst );
    inst = NULL;
}

void ReadRegisters( RegisterInstance* inst )
{
    size_t size = 0;

    uint32_t address = flash_get_addr();
    uint32_t endAddress = address + flash_get_size();

    uint32_t numRegisters = 0;
    size = sizeof( uint32_t );
    flash_read( &numRegisters, address, &size, NULL );
    address += size;

    // empty flash is all 0xFF
    assert( size == 4 );
    if ( numRegisters == 0xFFFFFFFF )
    {
        printf( "No registers found in flash memory\r\n" );
        return;
    }

    printf( "Reading %d registers from flash memory\r\n", numRegisters );

    FlashRegister reg = { 0 };
    uint32_t count = 0;
    while ( address < endAddress && count++ < numRegisters )
    {
        size = RegisterSize( inst );
        memset( &reg, 0, sizeof( FlashRegister ) );

        if ( flash_read( &reg, address, &size, DeserializeFlashRegister ) != FLASH_OK )
        {
            printf( "Failed to read register at address %d\r\n", address );
            break;
        }

        printf( "Register %.*s\r\n", reg.name.name.count, reg.name.name.elements );

        registerPush( inst, &reg );
        address += size;
    }
}

uint32_t RegisterStartAddress( RegisterInstance* inst )
{
    RegisterState* state = (RegisterState*)inst->user_reference;
    return state->register_start_addr;
}
uint32_t RegisterEndAddress( RegisterInstance* inst )
{
    RegisterState* state = (RegisterState*)inst->user_reference;
    return state->register_end_addr;
}

size_t RegisterSize( RegisterInstance* inst )
{
    RegisterState* state = (RegisterState*)inst->user_reference;
    return state->register_size;
}

size_t RegisterCount( RegisterInstance* inst )
{
    return inst->registersByName.size;
}

bool RegisterAdd( RegisterInstance* inst, const char* name, const uavcan_register_Value_1_0* value, bool immutable )
{
    FlashRegister reg = { 0 };
    reg.name.name.count = strlen( name );
    memcpy( reg.name.name.elements, name, reg.name.name.count );
    reg.value = *value;
    reg.isImmutable = immutable;

    if ( registerPush( inst, &reg ) < 0 )
    {
        printf( "Failed to push register %s\r\n", name );
        return false;
    }

    return true;
}

bool RegisterFlush( RegisterInstance* inst )
{
    // Write the register to flash memory
    if ( flash_unlock() != FLASH_OK )
    {
        printf( "Could not unlock flash memory\r\n" );
        return false;
    }
    if ( flash_erase() == FLASH_OK )
    {
        RegisterTreeItem* rootItem = (RegisterTreeItem*)(void*)inst->registersByName.root;
        uint32_t startAddress = flash_get_addr();
        uint32_t address = startAddress + sizeof( uint32_t );
        uint32_t numRegisters = inst->registersByName.size;

        if ( flash_write( &numRegisters, startAddress, sizeof( size_t ) ) != FLASH_OK )
        {
            flash_lock();
            printf( "Failed to write register count to flash memory\r\n" );
            return false;
        }

        bool success = serializeAndWriteTree( rootItem, &address );
        //printf( "end address: %p\r\n", (void*)address );

        flash_lock();

        if ( !success )
        {
            printf( "Failed to serialize registers to flash memory\r\n" );
            return false;
        }
    }
    return true;
}

int8_t DeserializeFlashRegister( void* const out_obj, const uint8_t* buffer, size_t* const inout_buffer_size_bytes )
{
    return mattfair_storage_Register_1_0_deserialize_( out_obj, buffer, inout_buffer_size_bytes );
}

bool RegisterRead( RegisterInstance* inst, const char* name, FlashRegister* regOut )
{
    // printTreeNode( inst->registersByName.root );
    RegisterTreeItem* item = (RegisterTreeItem*)(void*)cavlSearch(
        &inst->registersByName.root, (void*)name, &searchByNameAVLPredicate, NULL );

    if ( item == NULL )
    {
        return false;
    }

    *regOut = item->value;
    return true;
}

const RegisterTreeItem* getItemByIndex( RegisterInstance* inst, uint32_t index )
{
    if ( index >= inst->registersByName.size )
    {
        //printf( "Could not find register at index %d\r\n", index );
        return NULL;
    }

    return (const RegisterTreeItem*)(void*)inst->registersByIndex[index];
}

bool RegisterReadByIndex( RegisterInstance* inst, uint32_t index, FlashRegister* regOut )
{
    const RegisterTreeItem* item = getItemByIndex( inst, index );

    if ( item == NULL )
    {
        return false;
    }

    //printf( "Found register %.*s at index %d\r\n", item->value.name.name.count, item->value.name.name.elements, index );

    *regOut = item->value;
    return true;
}

uavcan_register_Name_1_0 RegisterNameByIndex( RegisterInstance* inst, const uint16_t index )
{
    FlashRegister reg = { 0 };
    if ( RegisterReadByIndex( inst, index, &reg ) )
    {
        return reg.name;
    }

    return ( uavcan_register_Name_1_0 ){ 0 };
}

uint32_t GetOffsetAddress( RegisterInstance* inst, uint32_t index )
{
    RegisterState* state = (RegisterState*)inst->user_reference;
    return state->register_start_addr + index * state->register_size;
}

// Erase all registers such that the defaults are used at the next launch.
void RegisterFactoryReset( RegisterInstance* inst )
{
    if ( flash_unlock() != FLASH_OK )
    {
        printf( "Could not unlock flash memory\r\n" );
        return;
    }

    // if we failed to erase the flash memory, don't clear things
    if ( flash_erase() == FLASH_OK )
    {
        clearTree( inst, &inst->registersByName );
    }
    else
    {
        printf( "Failed to erase flash memory\r\n" );
    }

    if ( flash_lock() != FLASH_OK )
    {
        printf( "Could not lock flash memory\r\n" );
    }
}

bool RegisterAssign( uavcan_register_Value_1_0* const dst, const uavcan_register_Value_1_0* const src )
{
    if ( uavcan_register_Value_1_0_is_empty_( dst ) )
    {
        *dst = *src;
        return true;
    }
    if ( ( uavcan_register_Value_1_0_is_string_( dst ) && uavcan_register_Value_1_0_is_string_( src ) ) ||
         ( uavcan_register_Value_1_0_is_unstructured_( dst ) && uavcan_register_Value_1_0_is_unstructured_( src ) ) )
    {
        *dst = *src;
        return true;
    }
    if ( uavcan_register_Value_1_0_is_bit_( dst ) && uavcan_register_Value_1_0_is_bit_( src ) )
    {
        nunavutCopyBits( dst->bit.value.bitpacked,
                         0,
                         nunavutChooseMin( dst->bit.value.count, src->bit.value.count ),
                         src->bit.value.bitpacked,
                         0 );
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
    REGISTER_CASE_SAME_TYPE( integer64 )
    REGISTER_CASE_SAME_TYPE( integer32 )
    REGISTER_CASE_SAME_TYPE( integer16 )
    REGISTER_CASE_SAME_TYPE( integer8 )
    REGISTER_CASE_SAME_TYPE( natural64 )
    REGISTER_CASE_SAME_TYPE( natural32 )
    REGISTER_CASE_SAME_TYPE( natural16 )
    REGISTER_CASE_SAME_TYPE( natural8 )
    REGISTER_CASE_SAME_TYPE( real64 )
    REGISTER_CASE_SAME_TYPE( real32 )
    REGISTER_CASE_SAME_TYPE( real16 )
    return false;
}

void PrintValue( const uavcan_register_Value_1_0* value )
{
    if ( uavcan_register_Value_1_0_is_natural16_( value ) )
    {
        printf( "Value: %d\r\n", value->natural16.value.elements[0] );
    }
    else if ( uavcan_register_Value_1_0_is_natural32_( value ) )
    {
        printf( "Value: %d\r\n", value->natural32.value.elements[0] );
    }
    else if ( uavcan_register_Value_1_0_is_natural64_( value ) )
    {
        printf( "Value: %lld\r\n", value->natural64.value.elements[0] );
    }
    else if ( uavcan_register_Value_1_0_is_real32_( value ) )
    {
        printf( "Value: %f\r\n", (double)value->real32.value.elements[0] );
    }
    else if ( uavcan_register_Value_1_0_is_real64_( value ) )
    {
        printf( "Value: %f\r\n", value->real64.value.elements[0] );
    }
    else if ( uavcan_register_Value_1_0_is_string_( value ) )
    {
        printf( "Value: %.*s\r\n", value->_string.value.count, value->_string.value.elements );
    }
    else if ( uavcan_register_Value_1_0_is_empty_( value ) )
    {
        printf( "Value: <empty>\r\n" );
    }
    else
    {
        printf( "Value: <unsupported>\r\n" );
    }
}
