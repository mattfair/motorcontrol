/*
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Copyright (c) 2024 Matt Fair
 */
#pragma once

#include <IO/cavl.h>
#include <mattfair/storage/Register_1_0.h>
#include <o1heap.h>
#include <string.h>
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>
#include <uavcan/_register/Value_1_0.h>

/*
 * Based on the Open Cyphal Demo code by Pavel Kirienko
 * https://github.com/OpenCyphal-Garage/demos/blob/main/differential_pressure_sensor/src/register.h
 */

/**
 * Registers are named values that keep various configuration parameters of the local Cyphal node (application).
 * Some of these parameters are used by the business logic of the application (e.g., PID gains, perfcounters);
 * others are used by the Cyphal stack (e.g., port-IDs, node-ID, transport configuration, introspection, and so on).
 * Registers of the latter category are all named with the same prefix "uavcan.", and their names and semantics
 * are regulated by the Cyphal Specification to ensure consistency across the ecosystem.
 *
 * The Specification doesn't define how the registers are to be stored since this part does not affect network
 * interoperability. In this demo we use a very simple and portable approach where each register is stored as
 * a separate file in the local filesystem; the name of the file matches the name of the register, and the register
 * values are serialized in the DSDL format (i.e., same format that is used for network exchange).
 * Deeply embedded systems may either use the same approach with the help of some compact fault-tolerant filesystem
 * (such as, for example, LittleFS: https://github.com/littlefs-project/littlefs), or they can resort to a low-level
 * specialized approach using on-chip EEPROM or similar (like PX4, Sapog, etc).
 */

#ifdef __cplusplus
extern "C"
{
#endif
    typedef mattfair_storage_Register_1_0 FlashRegister;

    typedef struct RegisterInstance RegisterInstance;
    typedef struct Cavl RegisterTreeNode;

    /// The AVL tree node structure is exposed here to avoid pointer casting/arithmetics inside the library.
    /// The user code is not expected to interact with this type except if advanced introspection is required.

    /// All operations (add, remove) are O(log n); there is exactly one heap allocation per element.
    typedef struct RegisterTree
    {
        /// The maximum number of frames this tree is allowed to contain. An attempt to push more will fail with an
        /// out-of-memory error even if the memory is not exhausted. This value can be changed by the user at any
        /// moment. The purpose of this limitation is to ensure that a blocked tree does not exhaust the heap memory.
        size_t capacity;

        /// The number of frames that are currently contained in the tree, initially zero.
        /// Do not modify this field!
        size_t size;

        /// The root of the tree is NULL if the tree is empty. Do not modify this field!
        RegisterTreeNode* root;

        /// This field can be arbitrarily mutated by the user. It is never accessed by the library.
        /// Its purpose is to simplify integration with OOP interfaces.
        void* user_reference;
    } RegisterTree;

    typedef struct RegisterTreeItem
    {
        /// Internal use only; do not access this field.
        RegisterTreeNode base;
        FlashRegister value;
    } RegisterTreeItem;

    /// A pointer to the memory allocation function. The semantics are similar to malloc():
    ///     - The returned pointer shall point to an uninitialized block of memory that is at least "amount" bytes
    ///     large.
    ///     - If there is not enough memory, the returned pointer shall be NULL.
    ///     - The memory shall be aligned at least at max_align_t.
    ///     - The execution time should be constant (O(1)).
    ///     - The worst-case memory fragmentation should be bounded and easily predictable.
    /// If the standard dynamic memory manager of the target platform does not satisfy the above requirements,
    /// consider using O1Heap: https://github.com/pavel-kirienko/o1heap.
    typedef void* ( *RegisterMemoryAllocate )( RegisterInstance* ins, size_t amount );

    /// The counterpart of the above -- this function is invoked to return previously allocated memory to the allocator.
    /// The semantics are similar to free():
    ///     - The pointer was previously returned by the allocation function.
    ///     - The pointer may be NULL, in which case the function shall have no effect.
    ///     - The execution time should be constant (O(1)).
    typedef void ( *RegisterMemoryFree )( RegisterInstance* ins, void* pointer );

    typedef struct RegisterState
    {
        uint32_t register_start_addr;
        uint32_t register_end_addr;
        size_t memory_size;
        size_t register_size;
        size_t max_register_count;
    } RegisterState;

    /// This is the core structure that keeps all of the states and allocated resources of the library instance.
    struct RegisterInstance
    {
        /// User pointer that can link this instance with other objects.
        /// This field can be changed arbitrarily, the library does not access it after initialization.
        /// The default value is NULL.
        void* user_reference;

        /// Dynamic memory management callbacks. See their type documentation for details.
        /// They SHALL be valid function pointers at all times.
        /// The time complexity models given in the API documentation are made on the assumption that the memory
        /// management functions have constant complexity O(1).
        RegisterMemoryAllocate memory_allocate;
        RegisterMemoryFree memory_free;

        RegisterTree registersByName;
        uint32_t* registersByIndex;
        O1HeapInstance* heap;
    };

    /**
     * @brief Initialize the registers in memory
     * @param start_addr start address of the registers
     * @param count number of registers
     * @param heap pointer to the heap instance
     */
    RegisterInstance* RegisterInit( uint32_t start_addr, size_t count, O1HeapInstance* heap, RegisterMemoryAllocate memory_allocate, RegisterMemoryFree memory_free);

    /**
     * @brief Destroy the registers in memory
     */
    void RegisterDestroy( RegisterInstance* ins );

    /**
     * @brief Get the start address of the registers
     * @return start address of the registers
     */
    uint32_t RegisterStartAddress( RegisterInstance* ins );

    /**
     * @brief Get the end address of the registers
     * @return end address of the registers
     */
    uint32_t RegisterEndAddress( RegisterInstance* ins );

    /**
     * @brief Get the size of a single stored registers
     * @return size of a register
     */
    size_t RegisterSize( RegisterInstance* ins );

    /**
     * @brief Get the current number of registers stored
     * @return number of registers
     */
    size_t RegisterCount( RegisterInstance* ins );

    /**
     * @brief Store the given register valuee as a registry
     * @param name name of the registry value
     * @param value pointer to the value
     * @param immutable if the register is immutable
     * @return true if successful
     * @return false if failed
     */
    bool RegisterAdd( RegisterInstance* inst,
                      const char* name,
                      const uavcan_register_Value_1_0* value,
                      bool immutable );

    /**
     * @brief flush the register to the flash memory
     * @param inst pointer to the register instance
     * @return true if successful
     */
    bool RegisterFlush( RegisterInstance* inst );

    /**
     * @brief  This function is mostly intended for implementing the standard RPC-service uavcan.register.List. It
     * returns the name of the register at the specified index (where the ordering is undefined but guaranteed to be
     * short-term stable), or empty name if the index is out of bounds.
     *
     * @param index index of the register
     * @return name of the register
     * @return empty name if the index is out of bonds
     */
    uavcan_register_Name_1_0 RegisterNameByIndex( RegisterInstance* inst, const uint16_t index );

    /**
     * @brief Read a register stored by name
     * @param name name of the register
     * @param regOut pointer to return the register read
     * @return true if successful
     */
    bool RegisterRead( RegisterInstance* inst, const char* name, FlashRegister* regOut );

    /**
     * @brief Read a register stored by index
     * @param index index of the register
     * @param regOut pointer to return the register read
     * @return true if successful
     */
    bool RegisterReadByIndex( RegisterInstance* inst, uint32_t index, FlashRegister* regOut );

    /**
     * @brief Get the offset address of the register
     * @param index index of the register
     */
    uint32_t GetOffsetAddress( RegisterInstance* inst, uint32_t index );

    /**
     * @brief Erase all registers such that the defaults are used at the next launch.
     */
    void RegisterFactoryReset( RegisterInstance* inst );

    /**
     * @brief Copy one value to the other if their types and dimensionality are the same or automatic conversion is
     * possible. If the destination is empty, it is simply replaced with the source (assignment always succeeds). The
     * return value is true if the assignment has been performed, false if it is not possible (in the latter case the
     * destination is NOT modified).
     *
     * @param dst destination value
     * @param src source value
     * @return true if successful
     * @return false if failed
     */
    bool RegisterAssign( uavcan_register_Value_1_0* const dst, const uavcan_register_Value_1_0* const src );

    /**
     * @brief Print the value of the register
     * @param value pointer to the value
     */
    void PrintValue( const uavcan_register_Value_1_0* value );

#ifdef __cplusplus
}
#endif
