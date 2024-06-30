/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby,
/// Shane Colton, David Otten, and others Hardware documentation can be found at
/// build-its.blogspot.com
/// UDRAL Servo implementation based on UDRAL Servo Demo by Pavel Kirienko
/// See
/// https://github.com/OpenCyphal-Garage/demos/blob/main/udral_servo/src/main.c

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "IO/register.h"
#include "adc.h"
#include "can.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>
#include <inttypes.h>
#include <reg/udral/physics/dynamics/translation/LinearTs_0_1.h>
#include <reg/udral/physics/electricity/PowerTs_0_1.h>
#include <reg/udral/service/actuator/common/Feedback_0_1.h>
#include <reg/udral/service/actuator/common/Status_0_1.h>
#include <reg/udral/service/actuator/common/_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>
#include <uavcan/node/ExecuteCommand_1_1.h>
#include <uavcan/node/GetInfo_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/port/List_0_1.h>
#include <uavcan/pnp/NodeIDAllocationData_1_0.h>
#include <unistd.h>

#include "calibration.h"
#include "canard.h"
#include "drv8323.h"
#include "flash_writer.h"
#include "foc.h"
#include "fsm.h"
#include "hw_config.h"
#include "math_ops.h"
#include "o1heap.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "stm32f4xx_flash.h"
#include "structs.h"
#include "user_config.h"

const uint32_t FLASH_USER_START_ADDR = ( (uint32_t)0x08060000 ); /* Base @ of Sector 7, 128 Kbytes */
const uint32_t FLASH_SECTOR_SIZE = 128 * 1024;
const uint32_t FLASH_USER_END_ADDR = FLASH_USER_START_ADDR + FLASH_SECTOR_SIZE - 1;

/// For CAN FD the queue can be smaller.
#define CAN_TX_QUEUE_CAPACITY 100

// Polynomial for CRC-64-WE this is used for the unique-ID hash.
#define CRC64_POLY 0x42F0E1EBA9EA3693ULL

#define MAX_RETRY_LIMIT 5

typedef enum SubjectRole
{
    SUBJECT_ROLE_PUBLISHER,
    SUBJECT_ROLE_SUBSCRIBER,
} SubjectRole;

/// This flag is raised when the node is requested to restart.
static volatile bool g_restart_required = false;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;
State servo_state = { 0 };

/* init but don't allocate calibration arrays */
int* error_array = NULL;
int* lut_array = NULL;

uint8_t Serial2RxBuffer[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef hal_can_transmit( CAN_HandleTypeDef* hcan, const CanardFrame* frame );
HAL_StatusTypeDef hal_can_receive( CAN_HandleTypeDef* hcan, CanardFrame* frame );

// convert port id to name
static char port_buffer[128];
const char* PortToName( const CanardPortID port_id )
{
    if ( port_id == 8166 )
    {
        sprintf( port_buffer, "uavcan.pnp.NodeIDAllocationData (%hu)", port_id );
    }
    else if ( port_id == 7509 )
    {
        sprintf( port_buffer, "uavcan.node.Heartbeat (%hu)", port_id );
    }
    else if ( port_id == 7510 )
    {
        sprintf( port_buffer, "uavcan.node.port.List (%hu)", port_id );
    }
    else if ( port_id == 385 )
    {
        sprintf( port_buffer, "uavcan.node.port.List (%hu)", port_id );
    }
    else if ( port_id == servo_state.port_id.sub.servo_readiness )
    {
        sprintf( port_buffer, "reg.udral.service.common.Readiness (%hu)", port_id );
    }
    else if ( port_id == servo_state.port_id.sub.servo_setpoint )
    {
        sprintf( port_buffer, "reg.udral.physics.dynamics.translation.Linear (%hu)", port_id );
    }
    else if ( port_id == servo_state.port_id.pub.servo_feedback )
    {
        sprintf( port_buffer, "reg.udral.service.actuator.common.Feedback (%hu)", port_id );
    }
    else if ( port_id == servo_state.port_id.pub.servo_status )
    {
        sprintf( port_buffer, "reg.udral.service.actuator.common.Status (%hu)", port_id );
    }
    else if ( port_id == servo_state.port_id.pub.servo_power )
    {
        sprintf( port_buffer, "reg.udral.physics.electricity.PowerTs (%hu)", port_id );
    }
    else if ( port_id == servo_state.port_id.pub.servo_dynamics )
    {
        sprintf( port_buffer, "reg.udral.physics.dynamics.translation.LinearTs (%hu)", port_id );
    }
    else
    {
        sprintf( port_buffer, "unknown (%hu)", port_id );
    }
    return port_buffer;
}

const char* KindToName( const CanardTransferKind kind )
{
    if ( kind == CanardTransferKindMessage )
    {
        return "Message";
    }
    else if ( kind == CanardTransferKindRequest )
    {
        return "Request";
    }
    else if ( kind == CanardTransferKindResponse )
    {
        return "Response";
    }

    return "Unknown";
}

// Function to get the current time in microseconds
CanardMicrosecond GetMonotonicMicroseconds( void )
{
    // Get the current millisecond tick count from the HAL
    uint64_t ms_ticks = (uint64_t)HAL_GetTick();

    // Convert the millisecond tick count to microseconds
    CanardMicrosecond us_time = ms_ticks * 1000;

    return us_time;
}

// Returns the 128-bit unique-ID of the local node. This value is used in
// uavcan.node.GetInfo.Response and during the plug-and-play node-ID allocation
// by uavcan.pnp.NodeIDAllocationData. The function is infallible.
static void GetUniqueID( uint8_t out[uavcan_node_GetInfo_Response_1_0_unique_id_ARRAY_CAPACITY_] )
{
    // Read the unique ID from the STM32's unique ID registers
    uint32_t uid[3];
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
    memcpy( out, uid, sizeof( uid ) );
}

// Compute CRC-64 hash for data
static uint64_t ComputeCRC64( const uint8_t* data, size_t length )
{
    uint64_t crc = 0xFFFFFFFFFFFFFFFFULL;
    for ( size_t i = 0; i < length; i++ )
    {
        uint64_t byte = data[i];
        crc ^= byte << 56;
        for ( int j = 0; j < 8; j++ )
        {
            if ( crc & 0x8000000000000000ULL )
            {
                crc = ( crc << 1 ) ^ CRC64_POLY;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return ~crc;
}

// Get 48-bit hash from the unique ID
static uint64_t GetUniqueIDHash()
{
    uint8_t uniqueID[16];
    GetUniqueID( uniqueID );  // Fetch the 128-bit unique ID from your device

    uint64_t crc = ComputeCRC64( uniqueID, sizeof( uniqueID ) );
    return crc & 0xFFFFFFFFFFFFULL;  // Return the lower 48 bits
}

/// Reads the port-ID from the corresponding standard register. The standard
/// register schema is documented in the Cyphal Specification, section for the
/// standard service uavcan.register.Access. You can also find it here:
/// https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/register/384.Access.1.0.dsdl
/// A very hands-on demo is available in Python:
/// https://pycyphal.readthedocs.io/en/stable/pages/demo.html
static CanardPortID GetSubjectID( const SubjectRole role, const char* const port_name, const char* const type_name )
{
    // Deduce the register name from port name.
    const char* const role_name = ( role == SUBJECT_ROLE_PUBLISHER ) ? "pub" : "sub";
    char register_name[uavcan_register_Name_1_0_name_ARRAY_CAPACITY_] = { 0 };
    snprintf( &register_name[0], sizeof( register_name ), "uavcan.%s.%s.id", role_name, port_name );

    // Set up the default value. It will be used to populate the register if it
    // doesn't exist.
    FlashRegister reg = { 0 };
    uavcan_register_Value_1_0_select_natural16_( &reg.value );
    reg.value.natural16.value.count = 1;
    reg.value.natural16.value.elements[0] = UINT16_MAX;  // This means "undefined", per Specification, which is the

    // Read the register with defensive self-checks.
    RegisterRead( &register_name[0], &reg );
    assert( uavcan_register_Value_1_0_is_natural16_( &reg.value ) && ( reg.value.natural16.value.count == 1 ) );
    const uint16_t result = reg.value.natural16.value.elements[0];

    // This part is NOT required but recommended by the Specification for enhanced
    // introspection capabilities. It is very cheap to implement so all
    // implementations should do so. This register simply contains the name of the
    // type exposed at this port. It should be immutable, but it is not strictly
    // required so in this implementation we take shortcuts by making it mutable
    // since it's behaviorally simpler in this specific case.
    snprintf( &register_name[0], sizeof( register_name ), "uavcan.%s.%s.type", role_name, port_name );
    memset( &reg, 0, sizeof( reg ) );
    uavcan_register_Value_1_0_select_string_( &reg.value );
    reg.value._string.value.count =
        nunavutChooseMin( strlen( type_name ), uavcan_primitive_String_1_0_value_ARRAY_CAPACITY_ );
    memcpy( &reg.value._string.value.elements[0], type_name, reg.value._string.value.count );
    RegisterWrite( &register_name[0], &reg.value, true );

    return result;
}

void Send( State* const state,
           const CanardMicrosecond tx_deadline_usec,
           const CanardTransferMetadata* const metadata,
           const size_t payload_size,
           const void* const payload )
{
    for ( uint8_t ifidx = 0; ifidx < CAN_REDUNDANCY_FACTOR; ifidx++ )
    {
        int32_t result = canardTxPush(
            &state->canard_tx_queues[ifidx], &state->canard, tx_deadline_usec, metadata, payload_size, payload );
        if ( result < 0 )
        {
            // An error has occurred: report it and handle appropriately.
            // The error value is negative, so we need to negate it to get the correct error code.
            printf( "Transmit error: %lu\r\n", -result );
        }
        else
        {
            // The transmission request has been enqueued. Nothing else to do.
            // printf( "sent successfully\r\n" );
            printf( "port_id=%s, transfer_id=%d, priority=%d, kind=%s, remote_node_id=%d\r\n",
                    PortToName( metadata->port_id ),
                    metadata->transfer_id,
                    metadata->priority,
                    KindToName( metadata->transfer_kind ),
                    metadata->remote_node_id );
            /*
            printf( "payload: " );
            for ( size_t i = 0; i < payload_size; ++i )
            {
                const uint8_t payloadByte = ( (const uint8_t*)payload )[i];
                printf( "%02X ", payloadByte );
            }
            printf( "\r\n" );
            */
        }
    }
}

/// Invoked at the rate of the fastest loop.
static void HandleFastLoop( State* const state, const CanardMicrosecond monotonic_time )
{
    // Apply control inputs if armed.
    /*
    if (state->servo.arming.armed)
    {
      fprintf(stderr, "\rp=%.3f m    v=%.3f m/s    a=%.3f (m/s)^2    F=%.3f N    \r", (double)state->servo.position,
    (double)state->servo.velocity, (double)state->servo.acceleration, (double)state->servo.force);
    }
    else
    {
      fprintf(stderr, "\rDISARMED    \r");
    }
    */

    const bool anonymous = state->canard.node_id > CANARD_NODE_ID_MAX;
    const uint64_t servo_transfer_id = state->next_transfer_id.servo_fast_loop++;

    // Publish feedback if the subject is enabled and the node is non-anonymous.
    if ( !anonymous && ( state->port_id.pub.servo_feedback <= CANARD_SUBJECT_ID_MAX ) )
    {
        reg_udral_service_actuator_common_Feedback_0_1 msg = { 0 };
        msg.heartbeat.readiness.value = state->servo.arming.armed ? reg_udral_service_common_Readiness_0_1_ENGAGED
                                                                  : reg_udral_service_common_Readiness_0_1_STANDBY;
        // If there are any hardware or configuration issues, report them here:
        msg.heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
        // Serialize and publish the message:
        uint8_t serialized[reg_udral_service_actuator_common_Feedback_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
        size_t serialized_size = sizeof( serialized );
        const int8_t err =
            reg_udral_service_actuator_common_Feedback_0_1_serialize_( &msg, &serialized[0], &serialized_size );
        assert( err >= 0 );
        if ( err >= 0 )
        {
            const CanardTransferMetadata transfer = {
                .priority = CanardPriorityHigh,
                .transfer_kind = CanardTransferKindMessage,
                .port_id = state->port_id.pub.servo_feedback,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id = (CanardTransferID)servo_transfer_id,
            };
            Send( state, monotonic_time + 10 * KILO, &transfer, serialized_size, &serialized[0] );
        }
    }

    // Publish dynamics if the subject is enabled and the node is non-anonymous.
    if ( !anonymous && ( state->port_id.pub.servo_dynamics <= CANARD_SUBJECT_ID_MAX ) )
    {
        reg_udral_physics_dynamics_translation_LinearTs_0_1 msg = { 0 };
        // Our node does not synchronize its clock with the network, so we cannot timestamp our publications:
        msg.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;
        // A real application would source these values from the hardware; we republish the setpoint for demo purposes.
        // TODO populate real values:
        msg.value.kinematics.position.meter = state->servo.position;
        msg.value.kinematics.velocity.meter_per_second = state->servo.velocity;
        msg.value.kinematics.acceleration.meter_per_second_per_second = state->servo.acceleration;
        msg.value.force.newton = state->servo.force;
        // Serialize and publish the message:
        uint8_t serialized[reg_udral_physics_dynamics_translation_LinearTs_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {
            0
        };
        size_t serialized_size = sizeof( serialized );
        const int8_t err =
            reg_udral_physics_dynamics_translation_LinearTs_0_1_serialize_( &msg, &serialized[0], &serialized_size );
        assert( err >= 0 );
        if ( err >= 0 )
        {
            const CanardTransferMetadata transfer = {
                .priority = CanardPriorityHigh,
                .transfer_kind = CanardTransferKindMessage,
                .port_id = state->port_id.pub.servo_dynamics,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id = (CanardTransferID)servo_transfer_id,
            };
            Send( state, monotonic_time + 10 * KILO, &transfer, serialized_size, &serialized[0] );
        }
    }

    // Publish power if the subject is enabled and the node is non-anonymous.
    if ( !anonymous && ( state->port_id.pub.servo_power <= CANARD_SUBJECT_ID_MAX ) )
    {
        reg_udral_physics_electricity_PowerTs_0_1 msg = { 0 };
        // Our node does not synchronize its clock with the network, so we cannot timestamp our publications:
        msg.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;
        // TODO populate real values:
        msg.value.current.ampere = 20.315F;
        msg.value.voltage.volt = 51.3F;
        // Serialize and publish the message:
        uint8_t serialized[reg_udral_physics_dynamics_translation_LinearTs_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {
            0
        };
        size_t serialized_size = sizeof( serialized );
        const int8_t err =
            reg_udral_physics_electricity_PowerTs_0_1_serialize_( &msg, &serialized[0], &serialized_size );
        assert( err >= 0 );
        if ( err >= 0 )
        {
            const CanardTransferMetadata transfer = {
                .priority = CanardPriorityHigh,
                .transfer_kind = CanardTransferKindMessage,
                .port_id = state->port_id.pub.servo_power,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id = (CanardTransferID)servo_transfer_id,
            };
            Send( state, monotonic_time + 10 * KILO, &transfer, serialized_size, &serialized[0] );
        }
    }
}

/// Invoked every second.
static void HandleOneHzLoop( State* const state, const CanardMicrosecond monotonic_time )
{
    // printf( "OneHzLoop\r\n" );
    const bool anonymous = state->canard.node_id > CANARD_NODE_ID_MAX;
    // Publish heartbeat every second unless the local node is anonymous. Anonymous nodes shall not publish heartbeat.
    if ( !anonymous )
    {
        uavcan_node_Heartbeat_1_0 heartbeat = { 0 };
        heartbeat.uptime = (uint32_t)( ( monotonic_time - state->started_at ) / MEGA );
        heartbeat.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
        const O1HeapDiagnostics heap_diag = o1heapGetDiagnostics( state->heap );
        if ( heap_diag.oom_count > 0 )
        {
            heartbeat.health.value = uavcan_node_Health_1_0_CAUTION;
        }
        else
        {
            heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
        }

        uint8_t serialized[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
        size_t serialized_size = sizeof( serialized );
        const int8_t err = uavcan_node_Heartbeat_1_0_serialize_( &heartbeat, &serialized[0], &serialized_size );
        assert( err >= 0 );
        if ( err >= 0 )
        {
            // printf( "Sending heartbeat...\r\n" );
            const CanardTransferMetadata transfer = {
                .priority = CanardPriorityNominal,
                .transfer_kind = CanardTransferKindMessage,
                .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id = (CanardTransferID)( state->next_transfer_id.uavcan_node_heartbeat++ ),
            };
            Send( state,
                  monotonic_time + MEGA,  // Set transmission deadline 1 second, optimal for heartbeat.
                  &transfer,
                  serialized_size,
                  &serialized[0] );
        }
        else
        {
            printf( "Error serializing heartbeat message\r\n" );
        }
    }
    else  // If we don't have a node-ID, obtain one by publishing allocation request messages until we get a response.
    {
        // The Specification says that the allocation request publication interval shall be randomized.
        // We implement randomization by calling rand() at fixed intervals and comparing it against some threshold.
        // There are other ways to do it, of course. See the docs in the Specification or in the DSDL definition here:
        // https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/pnp/8165.NodeIDAllocationData.2.0.dsdl
        // Note that a high-integrity/safety-certified application is unlikely to be able to rely on this feature.
        // if ( rand() > RAND_MAX / 2 )  // NOLINT
        {
            printf( "Requesting node-ID allocation...\r\n" );
            // Preparing the message for Classic CAN as per the UAVCAN v1.0 specification.
            uavcan_pnp_NodeIDAllocationData_1_0 msg = { 0 };

            // Compute the 48-bit unique ID hash. Assuming GetUniqueIDHash() is a function you define to perform the
            // hashing.
            msg.unique_id_hash = GetUniqueIDHash();

            // Serialize the message.
            uint8_t serialized[uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
            size_t serialized_size = sizeof( serialized );
            const int8_t err = uavcan_pnp_NodeIDAllocationData_1_0_serialize_( &msg, &serialized[0], &serialized_size );
            assert( err >= 0 );

            if ( err >= 0 )
            {
                const CanardTransferMetadata transfer = {
                    .priority = CanardPrioritySlow,
                    .transfer_kind = CanardTransferKindMessage,
                    .port_id = uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,
                    .remote_node_id = CANARD_NODE_ID_UNSET,  // As this is an anonymous message.
                    .transfer_id = (CanardTransferID)( state->next_transfer_id.uavcan_pnp_allocation++ ),
                };

                // Send the message.
                // The response may arrive asynchronously and may require processing the multi-frame response.
                Send( state, monotonic_time + MEGA, &transfer, serialized_size, &serialized[0] );
            }
        }

        const uint64_t servo_transfer_id = state->next_transfer_id.servo_1Hz_loop++;

        if ( !anonymous )
        {
            // Publish the servo status -- this is a low-rate message with low-severity diagnostics.
            reg_udral_service_actuator_common_Status_0_1 msg = { 0 };
            // TODO: POPULATE THE MESSAGE: temperature, errors, etc.
            uint8_t serialized[reg_udral_service_actuator_common_Status_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
            size_t serialized_size = sizeof( serialized );
            const int8_t err =
                reg_udral_service_actuator_common_Status_0_1_serialize_( &msg, &serialized[0], &serialized_size );
            assert( err >= 0 );
            if ( err >= 0 )
            {
                const CanardTransferMetadata transfer = {
                    .priority = CanardPriorityNominal,
                    .transfer_kind = CanardTransferKindMessage,
                    .port_id = state->port_id.pub.servo_status,
                    .remote_node_id = CANARD_NODE_ID_UNSET,
                    .transfer_id = (CanardTransferID)servo_transfer_id,
                };
                Send( state, monotonic_time + MEGA, &transfer, serialized_size, &serialized[0] );
            }
        }

        // Disarm automatically if the arming subject has not been updated in a while.
        if ( state->servo.arming.armed &&
             ( ( monotonic_time - state->servo.arming.last_update_at ) >
               (uint64_t)( reg_udral_service_actuator_common___0_1_CONTROL_TIMEOUT * MEGA ) ) )
        {
            state->servo.arming.armed = false;
            puts( "Disarmed by timeout " );
        }
    }
}

/// This is needed only for constructing uavcan_node_port_List_0_1.
static void FillSubscriptions( const CanardTreeNode* const tree, uavcan_node_port_SubjectIDList_0_1* const obj )
{
    if ( NULL != tree )
    {
        FillSubscriptions( tree->lr[0], obj );
        const CanardRxSubscription* crs = (const CanardRxSubscription*)tree;
        assert( crs->port_id <= CANARD_SUBJECT_ID_MAX );
        assert( obj->sparse_list.count < uavcan_node_port_SubjectIDList_0_1_sparse_list_ARRAY_CAPACITY_ );
        obj->sparse_list.elements[obj->sparse_list.count++].value = crs->port_id;
        FillSubscriptions( tree->lr[1], obj );
    }
}

/// This is needed only for constructing uavcan_node_port_List_0_1.
static void FillServers( const CanardTreeNode* const tree, uavcan_node_port_ServiceIDList_0_1* const obj )
{
    if ( NULL != tree )
    {
        FillServers( tree->lr[0], obj );
        const CanardRxSubscription* crs = (const CanardRxSubscription*)tree;
        assert( crs->port_id <= CANARD_SERVICE_ID_MAX );
        (void)nunavutSetBit( &obj->mask_bitpacked_[0], sizeof( obj->mask_bitpacked_ ), crs->port_id, true );
        FillServers( tree->lr[1], obj );
    }
}

/// Invoked every 10 seconds.
static void HandleOneTenthHzLoop( State* const state, const CanardMicrosecond monotonic_time )
{
    const bool anonymous = state->canard.node_id > CANARD_NODE_ID_MAX;

    // Publish the recommended (not required) port introspection message. No point publishing it if we're anonymous.
    // The message is a bit heavy on the stack (about 2 KiB) but this is not a problem for a modern MCU.
    if ( !anonymous && state->canard.node_id <= CANARD_NODE_ID_MAX )
    {
        uavcan_node_port_List_0_1 m = { 0 };
        uavcan_node_port_List_0_1_initialize_( &m );
        uavcan_node_port_SubjectIDList_0_1_select_sparse_list_( &m.publishers );
        uavcan_node_port_SubjectIDList_0_1_select_sparse_list_( &m.subscribers );

        // Indicate which subjects we publish to. Don't forget to keep this updated if you add new publications!
        {
            size_t* const cnt = &m.publishers.sparse_list.count;
            m.publishers.sparse_list.elements[( *cnt )++].value = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
            m.publishers.sparse_list.elements[( *cnt )++].value = uavcan_node_port_List_0_1_FIXED_PORT_ID_;
            if ( state->port_id.pub.servo_feedback <= CANARD_SUBJECT_ID_MAX )
            {
                m.publishers.sparse_list.elements[( *cnt )++].value = state->port_id.pub.servo_feedback;
            }
            if ( state->port_id.pub.servo_status <= CANARD_SUBJECT_ID_MAX )
            {
                m.publishers.sparse_list.elements[( *cnt )++].value = state->port_id.pub.servo_status;
            }
            if ( state->port_id.pub.servo_power <= CANARD_SUBJECT_ID_MAX )
            {
                m.publishers.sparse_list.elements[( *cnt )++].value = state->port_id.pub.servo_power;
            }
            if ( state->port_id.pub.servo_dynamics <= CANARD_SUBJECT_ID_MAX )
            {
                m.publishers.sparse_list.elements[( *cnt )++].value = state->port_id.pub.servo_dynamics;
            }
        }

        // Indicate which servers and subscribers we implement.
        // We could construct the list manually but it's easier and more robust to just query libcanard for that.
        FillSubscriptions( state->canard.rx_subscriptions[CanardTransferKindMessage], &m.subscribers );
        FillServers( state->canard.rx_subscriptions[CanardTransferKindRequest], &m.servers );
        FillServers( state->canard.rx_subscriptions[CanardTransferKindResponse], &m.clients );  // For regularity.

        // Serialize and publish the message. Use a small buffer because we know that our message is always small.
        uint8_t serialized[512] = { 0 };  // https://github.com/OpenCyphal/nunavut/issues/191
        size_t serialized_size = uavcan_node_port_List_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        if ( uavcan_node_port_List_0_1_serialize_( &m, &serialized[0], &serialized_size ) >= 0 )
        {
            const CanardTransferMetadata transfer = {
                .priority = CanardPriorityOptional,  // Mind the priority.
                .transfer_kind = CanardTransferKindMessage,
                .port_id = uavcan_node_port_List_0_1_FIXED_PORT_ID_,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id = (CanardTransferID)( state->next_transfer_id.uavcan_node_port_list++ ),
            };
            Send( state, monotonic_time + MEGA, &transfer, serialized_size, &serialized[0] );
        }
    }
}

/// https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/servo/_.0.1.dsdl
void ProcessMessageServoSetpoint( State* const state,
                                  const reg_udral_physics_dynamics_translation_Linear_0_1* const msg )
{
    state->servo.position = msg->kinematics.position.meter;
    state->servo.velocity = msg->kinematics.velocity.meter_per_second;
    state->servo.acceleration = msg->kinematics.acceleration.meter_per_second_per_second;
    state->servo.force = msg->force.newton;
}

/// https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/common/Readiness.0.1.dsdl
void ProcessMessageServiceReadiness( State* const state,
                                     const reg_udral_service_common_Readiness_0_1* const msg,
                                     const CanardMicrosecond monotonic_time )
{
    state->servo.arming.armed = msg->value >= reg_udral_service_common_Readiness_0_1_ENGAGED;
    state->servo.arming.last_update_at = monotonic_time;
}

/*
static void ProcessMessagePlugAndPlayNodeIDAllocation(State *const state, const uavcan_pnp_NodeIDAllocationData_2_0
*const msg)
{
  uint8_t uid[uavcan_node_GetInfo_Response_1_0_unique_id_ARRAY_CAPACITY_] = {0};
  GetUniqueID(uid);
  if ((msg->node_id.value <= CANARD_NODE_ID_MAX) && (memcmp(uid, msg->unique_id, sizeof(uid)) == 0))
  {
    printf("Got PnP node-ID allocation: %d\n", msg->node_id.value);
    state->canard.node_id = (CanardNodeID)msg->node_id.value;
    // Store the value into the non-volatile storage.
    uavcan_register_Value_1_0 reg = {0};
    uavcan_register_Value_1_0_select_natural16_(&reg);
    reg.natural16.value.elements[0] = msg->node_id.value;
    reg.natural16.value.count = 1;
    RegisterWrite("uavcan.node.id", &reg);
    // We no longer need the subscriber, drop it to free up the resources (both memory and CPU time).
    (void)canardRxUnsubscribe(&state->canard, CanardTransferKindMessage,
uavcan_pnp_NodeIDAllocationData_2_0_FIXED_PORT_ID_);
  }
  // Otherwise, ignore it: either it is a request from another node or it is a response to another node.
}
*/

void ProcessMessagePlugAndPlayNodeIDAllocation( State* const state,
                                                const uavcan_pnp_NodeIDAllocationData_1_0* const msg )
{
    printf( "Processing PnP node-ID allocation...\r\n" );
    uint64_t uid_hash = GetUniqueIDHash();

    printf( "Incoming UID hash: %lu, local UID hash: %lu\r\n",
            (unsigned long)msg->unique_id_hash,
            (unsigned long)uid_hash );
    if ( ( msg->allocated_node_id.count > 0 ) )  // &&
                                                 // FIXME ( msg->unique_id_hash == uid_hash ) )  // Check if there's an
                                                 // allocated node ID and the UID hashes match
    {
        printf( "Got PnP node-ID allocation: %d\r\n", msg->allocated_node_id.elements[0].value );
        state->canard.node_id = (CanardNodeID)msg->allocated_node_id.elements[0].value;
        // Store the value into non-volatile storage
        uavcan_register_Value_1_0 reg = { 0 };
        uavcan_register_Value_1_0_select_natural16_( &reg );
        reg.natural16.value.elements[0] = msg->allocated_node_id.elements[0].value;
        reg.natural16.value.count = 1;
        RegisterWrite( "uavcan.node.id", &reg, false );
        // We no longer need the subscriber, drop it to free up the resources (both memory and CPU time).
        (void)canardRxUnsubscribe(
            &state->canard, CanardTransferKindMessage, uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_ );
    }
    // Otherwise, ignore it: either it is a request from another node or it is a response to another node.
}

uavcan_node_ExecuteCommand_Response_1_1 ProcessRequestExecuteCommand(
    const uavcan_node_ExecuteCommand_Request_1_1* req )
{
    uavcan_node_ExecuteCommand_Response_1_1 resp = { 0 };
    switch ( req->command )
    {
    case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_BEGIN_SOFTWARE_UPDATE:
    {
        char file_name[uavcan_node_ExecuteCommand_Request_1_1_parameter_ARRAY_CAPACITY_ + 1] = { 0 };
        memcpy( file_name, req->parameter.elements, req->parameter.count );
        file_name[req->parameter.count] = '\0';
        // TODO: invoke the bootloader with the specified file name. See https://github.com/Zubax/kocherga/
        printf( "Firmware update request; filename: '%s' \n", &file_name[0] );
        resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_STATE;  // This is a stub.
        break;
    }
    case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_FACTORY_RESET:
    {
        RegisterFactoryReset();
        resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
        break;
    }
    case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART:
    {
        g_restart_required = true;
        resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
        break;
    }
    case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_STORE_PERSISTENT_STATES:
    {
        // If your registers are not automatically synchronized with the non-volatile storage, use this command
        // to commit them to the storage explicitly. Otherwise, it is safe to remove it.
        // In this demo, the registers are stored in files, so there is nothing to do.
        resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
        break;
    }
        // You can add vendor-specific commands here as well.
    default:
    {
        resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
        break;
    }
    }
    return resp;
}

/// Performance notice: the register storage may be slow to access depending on its implementation (e.g., if it is
/// backed by an uncached filesystem). If your register storage implementation is slow, this may disrupt real-time
/// activities of the device. To avoid this, you can employ either measure:
///
/// - Load registers to memory at startup, synchronize with the storage at reboot/power-down.
///   To implement fast register access you can use https://github.com/pavel-kirienko/cavl.
///   See also uavcan.node.ExecuteCommand.COMMAND_STORE_PERSISTENT_STATES.
///
/// - If an RTOS is used (not a baremetal system), you can run a separate Cyphal processing task for
///   soft-real-time blocking operations (this approach is used in PX4).
///
/// - Document an operational limitation that the register interface should not be accessed while ENGAGED (armed).
///   Cyphal networks usually have no service traffic while the vehicle is operational.
///
uavcan_register_Access_Response_1_0 ProcessRequestRegisterAccess( const uavcan_register_Access_Request_1_0* req )
{
    printf( "Processing register access request...\r\n" );
    char name[uavcan_register_Name_1_0_name_ARRAY_CAPACITY_ + 1] = { 0 };
    assert( req->name.name.count < sizeof( name ) );
    memcpy( &name[0], req->name.name.elements, req->name.name.count );
    name[req->name.name.count] = '\0';

    uavcan_register_Access_Response_1_0 resp = { 0 };
    FlashRegister reg = { 0 };

    uavcan_register_Value_1_0_select_empty_( &reg.value );
    RegisterRead( &name[0], &reg );

    // If we're asked to write a new value, do it now:
    if ( !uavcan_register_Value_1_0_is_empty_( &req->value ) )
    {
        uavcan_register_Value_1_0_select_empty_( &reg.value );
        RegisterRead( &name[0], &reg );
        // If such register exists and it can be assigned from the request value:
        if ( !uavcan_register_Value_1_0_is_empty_( &reg.value ) && RegisterAssign( &resp.value, &req->value ) )
        {
            RegisterWrite( &name[0], &resp.value, reg.isImmutable );
        }
    }

    // Regardless of whether we've just wrote a value or not, we need to read the current one and return it.
    // The client will determine if the write was successful or not by comparing the request value with response.
    uavcan_register_Value_1_0_select_empty_( &reg.value );
    RegisterRead( &name[0], &reg );
    RegisterAssign( &resp.value, &reg.value );

    // Currently, all registers we implement are mutable and persistent. This is an acceptable simplification,
    // but more advanced implementations will need to differentiate between them to support advanced features like
    // exposing internal states via registers, perfcounters, etc.
    resp._mutable = !reg.isImmutable;
    resp.persistent = true;

    // Our node does not synchronize its time with the network so we can't populate the timestamp.
    resp.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;

    return resp;
}

/// Constructs a response to uavcan.node.GetInfo which contains the basic information about this node.
uavcan_node_GetInfo_Response_1_0 ProcessRequestNodeGetInfo()
{
    printf( "Processing GetInfo request...\r\n" );
    uavcan_node_GetInfo_Response_1_0 resp = { 0 };
    resp.protocol_version.major = CANARD_CYPHAL_SPECIFICATION_VERSION_MAJOR;
    resp.protocol_version.minor = CANARD_CYPHAL_SPECIFICATION_VERSION_MINOR;

    // The hardware version is not populated in this demo because it runs on no specific hardware.
    // An embedded node like a servo would usually determine the version by querying the hardware.

    resp.software_version.major = VERSION_MAJOR;
    resp.software_version.minor = VERSION_MINOR;
    resp.software_vcs_revision_id = VCS_REVISION_ID;

    GetUniqueID( resp.unique_id );
    printf( "Unique ID: " );
    for ( size_t i = 0; i < sizeof( resp.unique_id ); i++ )
    {
        printf( "%02X ", resp.unique_id[i] );
    }

    // The node name is the name of the product like a reversed Internet domain name (or like a Java package).
    resp.name.count = strlen( NODE_NAME );
    memcpy( &resp.name.elements, NODE_NAME, resp.name.count );

    // The software image CRC and the Certificate of Authenticity are optional so not populated in this demo.
    return resp;
}

static void* CanardAllocate( CanardInstance* const ins, const size_t amount )
{
    O1HeapInstance* const heap = ( (State*)ins->user_reference )->heap;
    assert( o1heapDoInvariantsHold( heap ) );
    return o1heapAllocate( heap, amount );
}

static void CanardFree( CanardInstance* const ins, void* const pointer )
{
    O1HeapInstance* const heap = ( (State*)ins->user_reference )->heap;
    o1heapFree( heap, pointer );
}

/**
 * Attempt to transmit a CAN frame with exponential backoff retry logic.
 */
HAL_StatusTypeDef attempt_transmit( CAN_HandleTypeDef* hcan, const CanardFrame* frame )
{
    uint32_t retryDelay = 10;
    int retryCount = 0;

    while ( retryCount < MAX_RETRY_LIMIT )
    {
        if ( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) > 0 )
        {  // Ensure there is at least one free mailbox
            HAL_StatusTypeDef status = hal_can_transmit( hcan, frame );
            if ( status == HAL_OK )
            {
                return HAL_OK;  // Frame was successfully transmitted
            }
        }
        HAL_Delay( retryDelay );
        retryDelay *= 2;  // Exponentially increase the delay
        retryCount++;
    }
    return HAL_ERROR;  // Transmission failed after retries
}

extern char** environ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( const int argc, char* const argv[] )
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick.
     */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    //MX_TIM1_Init();
    MX_CAN1_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    /* USER CODE BEGIN 2 */

    /* Load settings from flash */
    preference_writer_init( &prefs, 6 );
    preference_writer_load( prefs );

    // compute the number of registers that can be stored in flash
    uint32_t registerCount = FLASH_SECTOR_SIZE / mattfair_storage_Register_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    RegisterInit( FLASH_USER_START_ADDR, registerCount );

    // RegisterFactoryReset();
    // RegisterDestroy();
    // RegisterInit( FLASH_USER_START_ADDR, registerCount );

    /* Sanitize configs in case flash is empty*/
    if ( E_ZERO == -1 )
    {
        E_ZERO = 0;
    }
    if ( M_ZERO == -1 )
    {
        M_ZERO = 0;
    }
    if ( isnan( I_BW ) || I_BW == -1 )
    {
        I_BW = 1000;
    }
    if ( isnan( I_MAX ) || I_MAX == -1 )
    {
        I_MAX = 40;
    }
    if ( isnan( I_FW_MAX ) || I_FW_MAX == -1 )
    {
        I_FW_MAX = 0;
    }
    if ( CAN_ID == -1 )
    {
        CAN_ID = 1;
    }
    if ( CAN_MASTER == -1 )
    {
        CAN_MASTER = 0;
    }
    if ( CAN_TIMEOUT == -1 )
    {
        CAN_TIMEOUT = 1000;
    }
    if ( isnan( R_NOMINAL ) || R_NOMINAL == -1 )
    {
        R_NOMINAL = 0.0f;
    }
    if ( isnan( TEMP_MAX ) || TEMP_MAX == -1 )
    {
        TEMP_MAX = 125.0f;
    }
    if ( isnan( I_MAX_CONT ) || I_MAX_CONT == -1 )
    {
        I_MAX_CONT = 14.0f;
    }
    if ( isnan( I_CAL ) || I_CAL == -1 )
    {
        I_CAL = 5.0f;
    }
    if ( isnan( PPAIRS ) || PPAIRS == -1 )
    {
        PPAIRS = 21.0f;
    }
    if ( isnan( GR ) || GR == -1 )
    {
        GR = 1.0f;
    }
    if ( isnan( KT ) || KT == -1 )
    {
        KT = 1.0f;
    }
    if ( isnan( KP_MAX ) || KP_MAX == -1 )
    {
        KP_MAX = 500.0f;
    }
    if ( isnan( KD_MAX ) || KD_MAX == -1 )
    {
        KD_MAX = 5.0f;
    }
    if ( isnan( P_MAX ) )
    {
        P_MAX = 12.5f;
    }
    if ( isnan( P_MIN ) )
    {
        P_MIN = -12.5f;
    }
    if ( isnan( V_MAX ) )
    {
        V_MAX = 65.0f;
    }
    if ( isnan( V_MIN ) )
    {
        V_MIN = -65.0f;
    }

    uavcan_node_GetInfo_Response_1_0 versionInfo = ProcessRequestNodeGetInfo();
    printf( "\r\nFirmware Version Number: %d.%d\r\n",
            (int)versionInfo.software_version.major,
            (int)versionInfo.software_version.minor );
    //(unsigned long long)versionInfo.software_vcs_revision_id );

    /* Controller Setup */
    if ( PHASE_ORDER )
    {  // Timer channel to phase mapping
    }
    else
    {
    }

    init_controller_params( &controller );

    /* calibration "encoder" zeroing */
    memset( &comm_encoder_cal.cal_position, 0, sizeof( EncoderStruct ) );

    /* commutation encoder setup */
    comm_encoder.m_zero = M_ZERO;
    comm_encoder.e_zero = E_ZERO;
    comm_encoder.ppairs = PPAIRS;

    if ( EN_ENC_LINEARIZATION )
    {
        memcpy( &comm_encoder.offset_lut, &ENCODER_LUT, sizeof( comm_encoder.offset_lut ) );
    }  // Copy the linearization lookup table
    else
    {
        memset( &comm_encoder.offset_lut, 0, sizeof( comm_encoder.offset_lut ) );
    }
    ps_warmup( &comm_encoder,
               100 );  // clear the noisy data when the encoder first turns on

    // for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}

    /* Turn on ADCs */
    HAL_ADC_Start( &hadc1 );
    HAL_ADC_Start( &hadc2 );
    HAL_ADC_Start( &hadc3 );

    /* DRV8323 setup */
    HAL_GPIO_WritePin( DRV_CS, GPIO_PIN_SET );  // CS high
    HAL_GPIO_WritePin( ENABLE_PIN, GPIO_PIN_SET );
    HAL_Delay( 1 );
    // drv_calibrate(drv);
    HAL_Delay( 1 );
    drv_write_DCR( drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1 );
    HAL_Delay( 1 );
    int CSA_GAIN;
    if ( I_MAX <= 40.0f )
    {
        CSA_GAIN = CSA_GAIN_40;
    }  // Up to 40A use 40X amplifier gain
    else
    {
        CSA_GAIN = CSA_GAIN_20;
    }  // From 40-60A use 20X amplifier gain.  (Make this generic in the future)
    drv_write_CSACR( drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_0_25 );
    HAL_Delay( 1 );
    drv_write_CSACR( drv, 0x0, 0x1, 0x0, CSA_GAIN, 0x1, 0x0, 0x0, 0x0, SEN_LVL_0_25 );
    HAL_Delay( 1 );
    zero_current( &controller );
    HAL_Delay( 1 );
    drv_write_OCPCR( drv, TRETRY_50US, DEADTIME_50NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_0_45 );
    HAL_Delay( 1 );
    drv_disable_gd( drv );
    HAL_Delay( 1 );
    // drv_enable_gd(drv);   */
    printf( "ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset );

    /* Turn on PWM */
    //HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
    //HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
    //HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );

    /*
  HAL_StatusTypeDef status;
  // Enable interrupts for RX FIFO 0 message pending and TX mailbox empty
  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
  CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR);
  if (status != HAL_OK) {
    printf("Could not activate CAN interrupt: %d\n", status);
  }
  */

    /* Set Interrupt Priorities */
    // HAL_NVIC_SetPriority(PWM_ISR, 0x0,0x0); // commutation > communication
    // HAL_NVIC_SetPriority(CAN_ISR, 0x01, 0x01);
    HAL_NVIC_SetPriority( CAN_ISR, 0x00, 0x00 );
    HAL_NVIC_EnableIRQ( CAN_ISR );

    /* Start the FSM */
    servo_state.fsm.state = MENU_MODE;
    servo_state.fsm.next_state = MENU_MODE;
    servo_state.fsm.ready = 1;

    /* Turn on interrupts */
    HAL_UART_Receive_IT( &huart2, (uint8_t*)Serial2RxBuffer, 1 );
    //HAL_TIM_Base_Start_IT( &htim1 );

    // A simple application like a servo node typically does not require more than 20 KiB of heap and 4 KiB of stack.
    // For the background and related theory refer to the following resources:
    // - https://github.com/OpenCyphal/libcanard/blob/master/README.md
    // - https://github.com/pavel-kirienko/o1heap/blob/master/README.md
    // - https://forum.opencyphal.org/t/uavcanv1-libcanard-nunavut-templates-memory-usage-concerns/1118/4
    _Alignas( O1HEAP_ALIGNMENT ) static uint8_t heap_arena[1024 * 20] = { 0 };
    servo_state.heap = o1heapInit( heap_arena, sizeof( heap_arena ) );
    assert( NULL != servo_state.heap );

    // The libcanard instance requires the allocator for managing protocol states.
    servo_state.canard = canardInit( &CanardAllocate, &CanardFree );
    servo_state.canard.user_reference = &servo_state;  // Make the state reachable from the canard instance.

    FlashRegister reg = { 0 };

    // The names of the standard registers are regulated by the Specification.
    if ( !RegisterRead( "uavcan.node.id", &reg ) )
    {
        // Restore the node-ID from the corresponding standard register. Default to anonymous.
        uavcan_register_Value_1_0_select_natural16_( &reg.value );
        reg.value.natural16.value.count = 1;

        // This means undefined (anonymous), per Specification/libcanard.
        reg.value.natural16.value.elements[0] = UINT16_MAX;
        RegisterWrite( "uavcan.node.id", &reg.value, false );
    }
    assert( uavcan_register_Value_1_0_is_natural16_( &reg.value ) && ( reg.value.natural16.value.count == 1 ) );
    servo_state.canard.node_id = ( reg.value.natural16.value.elements[0] > CANARD_NODE_ID_MAX )
                                     ? CANARD_NODE_ID_UNSET
                                     : (CanardNodeID)reg.value.natural16.value.elements[0];
    printf( "CAN ID: %d\r\n", servo_state.canard.node_id );

    /* CAN setup */
    CAN_FilterTypeDef sFilterConfig = { 0 };
    sFilterConfig.FilterBank = 0;                       // Specify the filter bank
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   // Use identifier mask mode
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // Use 32-bit scale for the filter
    sFilterConfig.FilterIdHigh = 0x0000;                // High 16 bits of the ID filter
    sFilterConfig.FilterIdLow = 0x0000;                 // Low 16 bits of the ID filter
    sFilterConfig.FilterMaskIdHigh = 0x0000;            // High 16 bits of the ID mask
    sFilterConfig.FilterMaskIdLow = 0x0000;             // Low 16 bits of the ID mask
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // Assign the filter to FIFO 0
    sFilterConfig.FilterActivation = ENABLE;            // Enable the filter

    if ( HAL_CAN_ConfigFilter( &CAN_H, &sFilterConfig ) != HAL_OK )
    {
        // Handle error
        Error_Handler();
    }

    // Enable CAN RX interrupt
    /*
    if (HAL_CAN_ActivateNotification(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      // Handle error
      Error_Handler();
    }
    */

    // start CAN
    if ( HAL_CAN_Start( &CAN_H ) != HAL_OK )
    {
        // Handle error
        Error_Handler();
    }

    //__HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING); // Start can
    // interrupt
    __HAL_CAN_ENABLE_IT( &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR );

    servo_state.canbus[0] = hcan1;

    // The description register is optional but recommended because it helps constructing/maintaining large networks.
    // It simply keeps a human-readable description of the node that should be empty by default.
    uavcan_register_Value_1_0_select_string_( &reg.value );
    reg.value._string.value.count = 0;

    // We don't need the value, we just need to ensure it exists.
    if ( !RegisterRead( "uavcan.node.description", &reg ) )
    {
        RegisterWrite( "uavcan.node.description", &reg.value, false );
    }

    if ( !RegisterRead( "udral.pnp.cookie", &reg ) )
    {
        // The UDRAL cookie is used to mark nodes that are auto-configured by a specific auto-configuration authority.
        // We don't use this value, it is managed by remote nodes; our only responsibility is to persist it across
        // reboots. This register is entirely optional though; if not provided, the node will have to be configured
        // manually.
        uavcan_register_Value_1_0_select_string_( &reg.value );

        // The value should be empty by default, meaning that the node is not configured.
        reg.value._string.value.count = 0;

        RegisterWrite( "udral.pnp.cookie", &reg.value, false );
    }

    // Announce which UDRAL network services we support by populating appropriate registers.
    uavcan_register_Value_1_0_select_string_( &reg.value );
    strcpy( (char*)reg.value._string.value.elements, "servo" );  // The prefix in port names like "servo.feedback", etc.
    reg.value._string.value.count = strlen( (const char*)reg.value._string.value.elements );
    RegisterWrite( "reg.udral.service.actuator.servo", &reg.value, true );

    if ( !RegisterRead( "uavcan.can.mtu", &reg ) )
    {
        // Configure the transport by reading the appropriate standard registers.
        uavcan_register_Value_1_0_select_natural16_( &reg.value );
        reg.value.natural16.value.count = 1;
        reg.value.natural16.value.elements[0] = CANARD_MTU_CAN_CLASSIC;  // Default to 8 bytes
        RegisterWrite( "uavcan.can.mtu", &reg.value, false );
    }
    assert( uavcan_register_Value_1_0_is_natural16_( &reg.value ) && ( reg.value.natural16.value.count == 1 ) );

    for ( uint8_t ifidx = 0; ifidx < CAN_REDUNDANCY_FACTOR; ifidx++ )
    {
        servo_state.canard_tx_queues[ifidx] =
            canardTxInit( CAN_TX_QUEUE_CAPACITY, reg.value.natural16.value.elements[0] );
    }

    // Load the port-IDs from the registers. You can implement hot-reloading at runtime if desired. Specification here:
    // https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/servo/_.0.1.dsdl
    // https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/README.md
    // As follows from the Specification, the register group name prefix can be arbitrary; here we just use "servo".
    // Publications:
    servo_state.port_id.pub.servo_feedback =  // High-rate status information: all good or not, engaged or sleeping.
        GetSubjectID( SUBJECT_ROLE_PUBLISHER,
                      "servo.feedback",
                      reg_udral_service_actuator_common_Feedback_0_1_FULL_NAME_AND_VERSION_ );
    servo_state.port_id.pub.servo_status =  // A low-rate high-level status overview: temperatures, fault flags, errors.
        GetSubjectID( SUBJECT_ROLE_PUBLISHER,
                      "servo.status",
                      reg_udral_service_actuator_common_Status_0_1_FULL_NAME_AND_VERSION_ );
    servo_state.port_id.pub.servo_power =  // Electric power input measurements (voltage and current).
        GetSubjectID(
            SUBJECT_ROLE_PUBLISHER, "servo.power", reg_udral_physics_electricity_PowerTs_0_1_FULL_NAME_AND_VERSION_ );
    servo_state.port_id.pub.servo_dynamics =  // Position/speed/acceleration/force feedback.
        GetSubjectID( SUBJECT_ROLE_PUBLISHER,
                      "servo.dynamics",
                      reg_udral_physics_dynamics_translation_LinearTs_0_1_FULL_NAME_AND_VERSION_ );
    // Subscriptions:
    servo_state.port_id.sub
        .servo_setpoint =  // This message actually commands the servo setpoint with the motion profile.
        GetSubjectID( SUBJECT_ROLE_SUBSCRIBER,
                      "servo.setpoint",
                      reg_udral_physics_dynamics_translation_Linear_0_1_FULL_NAME_AND_VERSION_ );
    servo_state.port_id.sub.servo_readiness =  // Arming subject: whether to act upon the setpoint or to stay idle.
        GetSubjectID(
            SUBJECT_ROLE_SUBSCRIBER, "servo.readiness", reg_udral_service_common_Readiness_0_1_FULL_NAME_AND_VERSION_ );

    // Set up subject subscriptions and RPC-service servers.
    // Message subscriptions:
    static const CanardMicrosecond servo_transfer_id_timeout = 100 * KILO;
    if ( servo_state.canard.node_id > CANARD_NODE_ID_MAX )
    {
        static CanardRxSubscription rx;
        const int8_t res = canardRxSubscribe( &servo_state.canard,
                                              CanardTransferKindMessage,
                                              uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,
                                              uavcan_pnp_NodeIDAllocationData_1_0_EXTENT_BYTES_,
                                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                              &rx );
        if ( res < 0 )
        {
            printf( "Failed to subscribe to PnP node-ID allocation: %d\r\n", res );
            return -res;
        }
        printf( "PnP node-ID allocation port ID: %d\r\n", uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_ );
    }
    else
    {
        printf( "Node ID already set, skipping PnP subscription\r\n" );
    }

    if ( servo_state.port_id.sub.servo_setpoint <= CANARD_SUBJECT_ID_MAX )  // Do not subscribe if not configured.
    {
        static CanardRxSubscription rx;
        const int8_t res = canardRxSubscribe( &servo_state.canard,
                                              CanardTransferKindMessage,
                                              servo_state.port_id.sub.servo_setpoint,
                                              reg_udral_physics_dynamics_translation_Linear_0_1_EXTENT_BYTES_,
                                              servo_transfer_id_timeout,
                                              &rx );
        if ( res < 0 )
        {
            printf( "Failed to subscribe to servo setpoint: %d\r\n", res );
            return -res;
        }
        printf( "Servo setpoint port ID: %d\r\n", servo_state.port_id.sub.servo_setpoint );
    }
    else
    {
        printf( "Servo setpoint subscription is not configured\r\n" );
    }

    if ( servo_state.port_id.sub.servo_readiness <= CANARD_SUBJECT_ID_MAX )  // Do not subscribe if not configured.
    {
        static CanardRxSubscription rx;
        const int8_t res = canardRxSubscribe( &servo_state.canard,
                                              CanardTransferKindMessage,
                                              servo_state.port_id.sub.servo_readiness,
                                              reg_udral_service_common_Readiness_0_1_EXTENT_BYTES_,
                                              servo_transfer_id_timeout,
                                              &rx );
        if ( res < 0 )
        {
            printf( "Failed to subscribe to servo readiness: %d\r\n", res );
            return -res;
        }
        printf( "Servo readiness port ID: %d\r\n", servo_state.port_id.sub.servo_readiness );
    }
    else
    {
        printf( "Servo readiness subscription is not configured\r\n" );
    }

    // Service servers:
    {
        static CanardRxSubscription rx;
        const int8_t res = canardRxSubscribe( &servo_state.canard,
                                              CanardTransferKindRequest,
                                              uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
                                              uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_,
                                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                              &rx );
        if ( res < 0 )
        {
            printf( "Failed to subscribe to GetInfo: %d\r\n", res );
            return -res;
        }
        printf( "GetInfo port ID: %d\r\n", uavcan_node_GetInfo_1_0_FIXED_PORT_ID_ );
    }
    {
        static CanardRxSubscription rx;
        const int8_t res = canardRxSubscribe( &servo_state.canard,
                                              CanardTransferKindRequest,
                                              uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_,
                                              uavcan_node_ExecuteCommand_Request_1_1_EXTENT_BYTES_,
                                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                              &rx );
        if ( res < 0 )
        {
            printf( "Failed to subscribe to ExecuteCommand: %d\r\n", res );
            return -res;
        }
        printf( "ExecuteCommand port ID: %d\r\n", uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_ );
    }
    {
        static CanardRxSubscription rx;
        const int8_t res = canardRxSubscribe( &servo_state.canard,
                                              CanardTransferKindRequest,
                                              uavcan_register_Access_1_0_FIXED_PORT_ID_,
                                              uavcan_register_Access_Request_1_0_EXTENT_BYTES_,
                                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                              &rx );
        if ( res < 0 )
        {
            printf( "Failed to subscribe to register access: %d\r\n", res );
            return -res;
        }
        printf( "Register access port ID: %d\r\n", uavcan_register_Access_1_0_FIXED_PORT_ID_ );
    }
    {
        static CanardRxSubscription rx;
        const int8_t res = canardRxSubscribe( &servo_state.canard,
                                              CanardTransferKindRequest,
                                              uavcan_register_List_1_0_FIXED_PORT_ID_,
                                              uavcan_register_List_Request_1_0_EXTENT_BYTES_,
                                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                              &rx );
        if ( res < 0 )
        {
            printf( "Failed to subscribe to register list: %d\r\n", res );
            return -res;
        }
        printf( "Register list port ID: %d\r\n", uavcan_register_List_1_0_FIXED_PORT_ID_ );
    }

    // Now the node is initialized and we're ready to roll.
    servo_state.started_at = GetMonotonicMicroseconds();
    const CanardMicrosecond fast_loop_period = MEGA / 50;
    CanardMicrosecond next_fast_iter_at = servo_state.started_at + fast_loop_period;
    CanardMicrosecond next_one_hz_iter_at = servo_state.started_at + MEGA;
    CanardMicrosecond next_one_tenth_hz_iter_at = servo_state.started_at + MEGA * 10;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    do
    {
        // Run a trivial scheduler polling the loops that run the business logic.
        CanardMicrosecond monotonic_time = GetMonotonicMicroseconds();
        // printf("Monotonic time: %ld \r\n", (long)monotonic_time);

        if ( monotonic_time >= next_fast_iter_at )
        {
            next_fast_iter_at += fast_loop_period;
            // printf("Next fast loop: %ld \r\n", (long)next_fast_iter_at);
            HandleFastLoop( &servo_state, monotonic_time );
        }
        if ( monotonic_time >= next_one_hz_iter_at )
        {
            next_one_hz_iter_at += MEGA;
            // printf("Next one Hz loop: %ld \r\n", (long)next_one_hz_iter_at);
            HandleOneHzLoop( &servo_state, monotonic_time );
        }
        if ( monotonic_time >= next_one_tenth_hz_iter_at )
        {
            next_one_tenth_hz_iter_at += MEGA * 10;
            HandleOneTenthHzLoop( &servo_state, monotonic_time );
            // printf("Next onetenth Hz loop: %ld \r\n", (long)next_one_tenth_hz_iter_at);
        }

        // Manage CAN RX/TX per redundant interface.
        for ( uint8_t ifidx = 0; ifidx < CAN_REDUNDANCY_FACTOR; ifidx++ )
        {
            // Transmit pending frames from the prioritized TX queues managed by libcanard.
            CanardTxQueue* const que = &servo_state.canard_tx_queues[ifidx];
            const CanardTxQueueItem* tqi = canardTxPeek( que );  // Find the highest-priority frame.
            while ( tqi != NULL )
            {
                if ( ( tqi->tx_deadline_usec == 0 ) || ( tqi->tx_deadline_usec > GetMonotonicMicroseconds() ) )
                {
                    if ( attempt_transmit( &servo_state.canbus[ifidx], &tqi->frame ) == HAL_OK )
                    {
                        // printf( "Successfully transmitted frame\r\n" );
                    }
                    else
                    {
                        printf( "Transmit failed after retries\r\n" );
                    }
                }
                else
                {
                    printf( "Dropping frame due to timeout\r\n" );
                }
                servo_state.canard.memory_free( &servo_state.canard, canardTxPop( que, tqi ) );
                tqi = canardTxPeek( que );
            }
        }
    } while ( !g_restart_required );
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // It is recommended to postpone restart until all frames are sent though.
    (void)argc;
    puts( "RESTART " );
    NVIC_SystemReset();
    return 0;
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if ( HAL_PWREx_EnableOverDrive() != HAL_OK )
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_5 ) != HAL_OK )
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
 * Transmit a CAN frame using the STM32 HAL library.
 * @param hcan Pointer to a CAN_HandleTypeDef structure that contains
 *             the configuration information for the specified CAN.
 * @param frame Pointer to a CanardFrame structure that contains the CAN frame to send.
 * @return HAL status.
 */
HAL_StatusTypeDef hal_can_transmit( CAN_HandleTypeDef* hcan, const CanardFrame* frame )
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];
    int max_retries = 5;
    int retry_count = 0;
    HAL_StatusTypeDef result;

    // Setup CAN header based on whether an extended CAN ID is used
    if ( frame->extended_can_id > 0x7FF )
    {
        tx_header.IDE = CAN_ID_EXT;
        tx_header.ExtId = frame->extended_can_id;
    }
    else
    {
        tx_header.IDE = CAN_ID_STD;
        tx_header.StdId = frame->extended_can_id & 0x7FF;
    }

    // Configure other parameters
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = frame->payload_size;
    tx_header.TransmitGlobalTime = DISABLE;

    // Copy the payload to the transmission buffer
    memcpy( tx_data, frame->payload, frame->payload_size );

    return HAL_CAN_AddTxMessage( hcan, &tx_header, tx_data, &tx_mailbox );
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
