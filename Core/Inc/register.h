/*
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Copyright (c) 2024 Matt Fair
 */
#pragma once

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

/**
 * @brief Reads the specified register from the persistent storage into `value`. If the register does not exist or it cannot be automatically
 * converted to the type of the provided argument, the value will be stored in the persistent storage using @ref registerWrite(), overriding existing
 * value. The default will not be initialized if the argument is empty.
 *
 * @param name name of the registry value
 * @param value pointer to the value
 * @return true if successful
 * @return false if failed
 */
bool RegisterRead(const char* name, uavcan_register_Value_1_0* value);

/**
 * @brief Store the given register value into the persistent storage.
 *
 * @param name name of the registry value
 * @param value pointer to the value
 * @return true if successful
 * @return false if failed
 */
bool RegisterWrite(const char* name, const uavcan_register_Value_1_0* value);

/**
 * @brief Copy one value to the other if their types and dimensionality are the same or automatic conversion is possible. If the destination is empty,
 * it is simply replaced with the source (assignment always succeeds). The return value is true if the assignment has been performed, false if it is
 * not possible (in the latter case the destination is NOT modified).
 *
 * @param dst destination value
 * @param src source value
 * @return true if successful
 * @return false if failed
 */
bool RegisterAssign(uavcan_register_Value_1_0* const dst, const uavcan_register_Value_1_0* const src);

/**
 * @brief  This function is mostly intended for implementing the standard RPC-service uavcan.register.List. It returns the name of the register at the
 * specified index (where the ordering is undefined but guaranteed to be short-term stable), or empty name if the index is out of bounds.
 *
 * @param index index of the register
 * @return name of the register
 * @return empty name if the index is out of bounds
 */
uavcan_register_Name_1_0 RegisterGetNameByIndex(const uint16_t index);

/**
 * @brief Erase all registers such that the defaults are used at the next launch.
 */
void RegisterDoFactoryReset(void);

/**
 * @brief Initialize the registers from flash
 */
void RegisterInit(void);
