/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <IO/register.h>
#include <stdio.h>

#include <reg/udral/physics/dynamics/translation/LinearTs_0_1.h>
#include <reg/udral/physics/electricity/PowerTs_0_1.h>
#include <reg/udral/service/actuator/common/Feedback_0_1.h>
#include <reg/udral/service/actuator/common/Status_0_1.h>
#include <reg/udral/service/actuator/common/_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>
#include "adc.h"
#include "can.h"
#include "canard.h"
#include "foc.h"
#include "fsm.h"
#include "gpio.h"
#include "hw_config.h"
#include "position_sensor.h"
#include "spi.h"
#include "structs.h"
#include "usart.h"
#include "user_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Tail parsing Debugging
#define CANARD_TRANSFER_ID_BIT_LENGTH 5U
#define CANARD_TRANSFER_ID_MAX ( ( 1U << CANARD_TRANSFER_ID_BIT_LENGTH ) - 1U )
#define TAIL_START_OF_TRANSFER 128U
#define TAIL_END_OF_TRANSFER 64U
#define TAIL_TOGGLE 32U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void HardFault_Handler_C( uint32_t* hardfault_args );
void printError( const char* message );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SendResponse( State* const state,
                          const CanardRxTransfer* const original_request_transfer,
                          const size_t payload_size,
                          const void* const payload )
{
    CanardTransferMetadata meta = original_request_transfer->metadata;
    meta.transfer_kind = CanardTransferKindResponse;
    Send( state, original_request_transfer->timestamp_usec + MEGA, &meta, payload_size, payload );
}

static void ProcessReceivedTransfer( State* const state, const CanardRxTransfer* const transfer )
{
    printf( "Received transfer: %d bytes from %d, port-ID %d, transfer-ID %d, kind %s\r\n",
            (int)transfer->payload_size,
            (int)transfer->metadata.remote_node_id,
            (int)transfer->metadata.port_id,
            (int)transfer->metadata.transfer_id,
            KindToName( transfer->metadata.transfer_kind ) );
    if ( transfer->metadata.transfer_kind == CanardTransferKindMessage )
    {
        size_t size = transfer->payload_size;
        if ( transfer->metadata.port_id == state->port_id.sub.servo_setpoint )
        {
            reg_udral_physics_dynamics_translation_Linear_0_1 msg = { 0 };
            if ( reg_udral_physics_dynamics_translation_Linear_0_1_deserialize_( &msg, transfer->payload, &size ) >= 0 )
            {
                ProcessMessageServoSetpoint( state, &msg );
            }
        }
        else if ( transfer->metadata.port_id == state->port_id.sub.servo_readiness )
        {
            reg_udral_service_common_Readiness_0_1 msg = { 0 };
            if ( reg_udral_service_common_Readiness_0_1_deserialize_( &msg, transfer->payload, &size ) >= 0 )
            {
                ProcessMessageServiceReadiness( state, &msg, transfer->timestamp_usec );
            }
        }
        else if ( transfer->metadata.port_id == uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_ )
        {
            printf( "Received PnP node-ID allocation request\r\n" );
            uavcan_pnp_NodeIDAllocationData_1_0 msg = { 0 };
            if ( uavcan_pnp_NodeIDAllocationData_1_0_deserialize_( &msg, transfer->payload, &size ) >= 0 )
            {
                ProcessMessagePlugAndPlayNodeIDAllocation( state, &msg );
            }
        }
        else
        {
            assert( false );  // Seems like we have set up a port subscription without a handler -- bad implementation.
        }
    }
    else if ( transfer->metadata.transfer_kind == CanardTransferKindRequest )
    {
        if ( transfer->metadata.port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_ )
        {
            printf( "Received GetInfo request\r\n" );
            // The request object is empty so we don't bother deserializing it. Just send the response.
            const uavcan_node_GetInfo_Response_1_0 resp = ProcessRequestNodeGetInfo();
            uint8_t serialized[uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
            size_t serialized_size = sizeof( serialized );
            printf( "Serializing GetInfo response...\r\n" );
            const int8_t res = uavcan_node_GetInfo_Response_1_0_serialize_( &resp, &serialized[0], &serialized_size );
            if ( res >= 0 )
            {
                printf( "Sending GetInfo response...\r\n" );
                SendResponse( state, transfer, serialized_size, &serialized[0] );
            }
            else
            {
                printf( "Error serializing GetInfo response\r\n" );
                assert( false );
            }
        }
        else if ( transfer->metadata.port_id == uavcan_register_Access_1_0_FIXED_PORT_ID_ )
        {
            printf( "Received register access request\r\n" );
            uavcan_register_Access_Request_1_0 req = { 0 };
            size_t size = transfer->payload_size;
            if ( uavcan_register_Access_Request_1_0_deserialize_( &req, transfer->payload, &size ) >= 0 )
            {
                const uavcan_register_Access_Response_1_0 resp = ProcessRequestRegisterAccess( &req );
                uint8_t serialized[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
                size_t serialized_size = sizeof( serialized );
                if ( uavcan_register_Access_Response_1_0_serialize_( &resp, &serialized[0], &serialized_size ) >= 0 )
                {
                    SendResponse( state, transfer, serialized_size, &serialized[0] );
                }
            }
        }
        else if ( transfer->metadata.port_id == uavcan_register_List_1_0_FIXED_PORT_ID_ )
        {
            printf( "Received register list request\r\n" );
            uavcan_register_List_Request_1_0 req = { 0 };
            size_t size = transfer->payload_size;
            if ( uavcan_register_List_Request_1_0_deserialize_( &req, transfer->payload, &size ) >= 0 )
            {
                const uavcan_register_List_Response_1_0 resp = { .name = RegisterNameByIndex( req.index ) };
                uint8_t serialized[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
                size_t serialized_size = sizeof( serialized );
                if ( uavcan_register_List_Response_1_0_serialize_( &resp, &serialized[0], &serialized_size ) >= 0 )
                {
                    printf( "Sending register list response...\r\n" );
                    SendResponse( state, transfer, serialized_size, &serialized[0] );
                    printf( "Sent register list response\r\n" );
                }
            }
        }
        else if ( transfer->metadata.port_id == uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_ )
        {
            uavcan_node_ExecuteCommand_Request_1_1 req = { 0 };
            size_t size = transfer->payload_size;
            if ( uavcan_node_ExecuteCommand_Request_1_1_deserialize_( &req, transfer->payload, &size ) >= 0 )
            {
                const uavcan_node_ExecuteCommand_Response_1_1 resp = ProcessRequestExecuteCommand( &req );
                uint8_t serialized[uavcan_node_ExecuteCommand_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = { 0 };
                size_t serialized_size = sizeof( serialized );
                if ( uavcan_node_ExecuteCommand_Response_1_1_serialize_( &resp, &serialized[0], &serialized_size ) >=
                     0 )
                {
                    SendResponse( state, transfer, serialized_size, &serialized[0] );
                }
            }
        }
        else
        {
            assert( false );  // Seems like we have set up a port subscription without a handler -- bad implementation.
        }
    }
    else
    {
        assert( false );  // Bad implementation -- check your subscriptions.
    }
}

static void PrintCanardFrame( const CanardFrame* frame )
{
    if ( frame == NULL )
    {
        printf( "Received a NULL frame pointer.\r\n" );
        return;
    }

    printf( "CAN Frame:\r\n" );
    printf( "  Extended CAN ID: 0x%08X\r\n", frame->extended_can_id );
    printf( "  Payload Size: %u bytes\r\n", (size_t)frame->payload_size );
    printf( "  Payload: " );
    if ( frame->payload != NULL && frame->payload_size > 0 )
    {
        const uint8_t* payload_bytes = (const uint8_t*)frame->payload;
        for ( size_t i = 0; i < frame->payload_size; ++i )
        {
            printf( "%02X ", payload_bytes[i] );
        }
    }
    else
    {
        printf( "No payload" );
    }
    printf( "\r\n" );

    if ( frame->payload_size > 0 )
    {
        const uint8_t tail = ( (const uint8_t*)frame->payload )[frame->payload_size - 1];
        const uint8_t transfer_id = tail & CANARD_TRANSFER_ID_MAX;
        const bool start_of_transfer = ( tail & TAIL_START_OF_TRANSFER ) != 0;
        const bool end_of_transfer = ( tail & TAIL_END_OF_TRANSFER ) != 0;
        const bool toggle = ( tail & TAIL_TOGGLE ) != 0;

        printf( "Tail byte: 0x%02X\r\n", tail );
        printf( "Transfer ID: %u\r\n", transfer_id );
        printf( "Start of Transfer: %s\r\n", start_of_transfer ? "true" : "false" );
        printf( "End of Transfer: %s\r\n", end_of_transfer ? "true" : "false" );
        printf( "Toggle: %s\r\n", toggle ? "true" : "false" );
    }
}

static HAL_StatusTypeDef hal_can_receive( CAN_HandleTypeDef* hcan, CanardFrame* frame )
{
    if ( __HAL_CAN_GET_FLAG( hcan, CAN_FLAG_FOV0 ) != RESET )
    {
        printf( "FIFO 0 Overflow\r\n" );
        __HAL_CAN_CLEAR_FLAG( hcan, CAN_FLAG_FOV0 );
    }

    uint32_t error = HAL_CAN_GetError( hcan );
    if ( error != HAL_CAN_ERROR_NONE )
    {
        printf( "CAN error: 0x%08lx\r\n", error );
        HAL_CAN_ResetError( hcan );  // Clear the error
    }

    HAL_StatusTypeDef result;
    memset( servo_state.can_payload_buffer, 0, sizeof( servo_state.can_payload_buffer ) );
    memset( &servo_state.rx_header, 0, sizeof( servo_state.rx_header ) );

    // Attempt to receive a message from the CAN peripheral
    result = HAL_CAN_GetRxMessage( hcan, CAN_RX_FIFO0, &servo_state.rx_header, servo_state.can_payload_buffer );
    if ( result == HAL_OK )
    {
        if ( servo_state.rx_header.DLC > CANARD_MTU_CAN_CLASSIC )
        {
            printf( "Received CAN frame with invalid DLC: %u\r\n", servo_state.rx_header.DLC );
            return HAL_ERROR;
        }

        frame->extended_can_id =
            ( servo_state.rx_header.IDE == CAN_ID_EXT ) ? servo_state.rx_header.ExtId : servo_state.rx_header.StdId;
        frame->payload_size = servo_state.rx_header.DLC;
        frame->payload = servo_state.can_payload_buffer;

        // Debugging output
        printf( "Received CAN frame with ID: %lu, DLC: %u, Data:", frame->extended_can_id, frame->payload_size );
        for ( size_t i = 0; i < frame->payload_size; ++i )
        {
            printf( " %02X", servo_state.can_payload_buffer[i] );  // Correctly dereferenced
        }
        printf( "\r\n" );
    }

    return result;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler( void )
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler( void )
{
    /* USER CODE BEGIN HardFault_IRQn 0 */
    __asm volatile( "TST lr, #4 \n"     // Test bit 2 of the Link Register (LR) to determine the active stack pointer
                    "ITE EQ \n"         // If-Then-Else construct based on the result of the TST instruction
                    "MRSEQ r0, MSP \n"  // If bit 2 of LR is 0 (equal), move the Main Stack Pointer (MSP) to r0
                    "MRSNE r0, PSP \n"  // If bit 2 of LR is 1 (not equal), move the Process Stack Pointer (PSP) to r0
                    "B HardFault_Handler_C \n"  // Branch to the C function HardFault_Handler_C
    );

    /* USER CODE END HardFault_IRQn 0 */
    while ( 1 )
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler( void )
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while ( 1 )
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler( void )
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while ( 1 )
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler( void )
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while ( 1 )
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler( void )
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler( void )
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler( void )
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler( void )
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles CAN1 TX interrupt.
 */
void CAN1_TX_IRQHandler( void )
{
    /* USER CODE BEGIN CAN1_TX_IRQn 0 */

    /* USER CODE END CAN1_TX_IRQn 0 */
    HAL_CAN_IRQHandler( &hcan1 );
    /* USER CODE BEGIN CAN1_TX_IRQn 1 */

    /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
 * @brief This function handles CAN1 RX0 interrupt.
 */
void CAN1_RX0_IRQHandler( void )
{
    /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
    printf( "CAN1_RX0_IRQHandler\n\r" );
    uint8_t ifidx = 0;
    CAN_HandleTypeDef canbus = servo_state.canbus[ifidx];

    // Process received frames by feeding them from CAN to libcanard.
    // The order in which we handle the redundant interfaces doesn't matter -- libcanard can accept incoming
    // frames from any of the redundant interface in an arbitrary order.
    // The internal state machine will sort them out and remove duplicates automatically.
    CanardFrame frame = { 0 };
    const HAL_StatusTypeDef receive_result = hal_can_receive( &canbus, &frame );
    if ( receive_result != HAL_OK )  // The read operation has timed out with no frames, nothing to do here.
    {
        // no message
        // printf( "Failed to receive CAN message: %d\r\n", receive_result );
        return;
    }
    PrintCanardFrame( &frame );
    // printf( "Successfully received CAN message: %d\r\n", receive_result );
    //  The AN adapter uses the wall clock for timestamping, but we need monotonic.
    //  Wall clock can only be used for time synchronization.
    const CanardMicrosecond timestamp_usec = GetMonotonicMicroseconds();
    CanardRxTransfer transfer = { 0 };
    CanardRxSubscription* sub = NULL;
    const int8_t canard_result = canardRxAccept( &servo_state.canard, timestamp_usec, &frame, ifidx, &transfer, &sub );

    /*
    printf( "Transfer received: %d bytes, port_id=%s, transfer_id=%d, priority=%d, kind=%s, remote_node_id=%d, "
            "canardRxAccept result=%d\r\n",
            transfer.payload_size,
            PortToName( transfer.metadata.port_id ),
            transfer.metadata.transfer_id,
            transfer.metadata.priority,
            KindToName( transfer.metadata.transfer_kind ),
            transfer.metadata.remote_node_id,
            canard_result );

    if ( sub != NULL )
    {
        printf( "Matching subscription found for port ID: %u\r\n", sub->port_id );
    }
    */

    if ( canard_result > 0 )
    {
        ProcessReceivedTransfer( &servo_state, &transfer );
        servo_state.canard.memory_free( &servo_state.canard, (void*)transfer.payload );
    }
    else if ( canard_result == 0 )
    {
        if ( sub != NULL )
        {
            // printf( "Ignoring received frame, transfer not complete.\r\n" );
        }
        else
        {
            printf( "Ignoring received frame, no matching subscription found.\r\n" );
        }
    }
    else if ( canard_result == -CANARD_ERROR_OUT_OF_MEMORY )
    {
        printf( "Out of memory while processing received frame\r\n" );
        (void)0;  // The frame did not complete a transfer so there is nothing to do.
                  // OOM should never occur if the heap is sized correctly. You can track OOM errors via heap
                  // API.
    }
    else
    {
        // no other error can be possible
        assert( false );
    }

    // printf("CAN1_RX0_IRQHandler\n\r");

    /* USER CODE END CAN1_RX0_IRQn 0 */
    HAL_CAN_IRQHandler( &hcan1 );
    /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

#if 0
  HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header,
                       can_rx.data);  // Read CAN
  uint32_t TxMailbox;
  pack_reply(&can_tx, CAN_ID, comm_encoder.angle_multiturn[0] / GR,
             comm_encoder.velocity / GR, controller.i_q_filt * KT * GR,
             controller.v_bus_filt);  // Pack response
  HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data,
                       &TxMailbox);  // Send response

  /* Check for special Commands */
  if (((can_rx.data[0] == 0xFF) & (can_rx.data[1] == 0xFF) &
       (can_rx.data[2] == 0xFF) & (can_rx.data[3] == 0xFF) &
       (can_rx.data[4] == 0xFF) & (can_rx.data[5] == 0xFF) &
       (can_rx.data[6] == 0xFF) & (can_rx.data[7] == 0xFC)))
  {
    update_fsm(&state, MOTOR_CMD);
  }
  else if (((can_rx.data[0] == 0xFF) & (can_rx.data[1] == 0xFF) &
            (can_rx.data[2] == 0xFF) &
            (can_rx.data[3] == 0xFF) * (can_rx.data[4] == 0xFF) &
            (can_rx.data[5] == 0xFF) & (can_rx.data[6] == 0xFF) &
            (can_rx.data[7] == 0xFD)))
  {
    update_fsm(&state, MENU_CMD);
  }
  else if (((can_rx.data[0] == 0xFF) & (can_rx.data[1] == 0xFF) &
            (can_rx.data[2] == 0xFF) &
            (can_rx.data[3] == 0xFF) * (can_rx.data[4] == 0xFF) &
            (can_rx.data[5] == 0xFF) & (can_rx.data[6] == 0xFF) &
            (can_rx.data[7] == 0xFE)))
  {
    update_fsm(&state, ZERO_CMD);
  }
  else
  {
    unpack_cmd(can_rx, controller.commands);  // Unpack commands
    controller.timeout = 0;                   // Reset timeout counter
  }

#endif
    /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
 * @brief This function handles CAN1 RX1 interrupt.
 */
void CAN1_RX1_IRQHandler( void )
{
    /* USER CODE BEGIN CAN1_RX1_IRQn 0 */
    printf( "CAN1_RX1_IRQHandler\n\r" );

    /* USER CODE END CAN1_RX1_IRQn 0 */
    HAL_CAN_IRQHandler( &hcan1 );
    /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

    /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
 * @brief This function handles CAN1 SCE interrupt.
 */
void CAN1_SCE_IRQHandler( void )
{
    /* USER CODE BEGIN CAN1_SCE_IRQn 0 */
    printf( "CAN1_SCE_IRQHandler\n\r" );

    /* USER CODE END CAN1_SCE_IRQn 0 */
    HAL_CAN_IRQHandler( &hcan1 );
    /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

    /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
 * @brief This function handles TIM1 update interrupt and TIM10 global
 * interrupt.
 */
void TIM1_UP_TIM10_IRQHandler( void )
{
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
    // HAL_GPIO_WritePin(LED, GPIO_PIN_SET );	// Useful for timing

    /* Sample ADCs */
    // analog_sample(&controller);

    /* Sample position sensor */
    // ps_sample(&comm_encoder, DT);

    /* Run Finite State Machine */
    run_fsm( &state );

    /* Check for CAN messages */
    // can_tx_rx();

    /* increment loop count */
    controller.loop_count++;
    // HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );

    /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
    HAL_TIM_IRQHandler( &htim1 );
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
    /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler( void )
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    HAL_UART_IRQHandler( &huart2 );

    char c = Serial2RxBuffer[0];
    update_fsm( &state, c );

    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler( &huart2 );
    /* USER CODE BEGIN USART2_IRQn 1 */
    /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// Function to print error messages
void printError( const char* message )
{
    // Implement the function to send `message` to a serial console
    // e.g., using UART or ITM_SendChar for SWO
}

void HardFault_Handler_C( uint32_t* hardfault_args )
{
    uint32_t stacked_r0 = hardfault_args[0];
    uint32_t stacked_r1 = hardfault_args[1];
    uint32_t stacked_r2 = hardfault_args[2];
    uint32_t stacked_r3 = hardfault_args[3];
    uint32_t stacked_r12 = hardfault_args[4];
    uint32_t stacked_lr = hardfault_args[5];
    uint32_t stacked_pc = hardfault_args[6];
    uint32_t stacked_psr = hardfault_args[7];

    printError( "Hard Fault Handler\n" );
    printf( "R0 = 0x%08X\r\n", (unsigned int)stacked_r0 );
    printf( "R1 = 0x%08X\r\n", (unsigned int)stacked_r1 );
    printf( "R2 = 0x%08X\r\n", (unsigned int)stacked_r2 );
    printf( "R3 = 0x%08X\r\n", (unsigned int)stacked_r3 );
    printf( "R12 = 0x%08X\r\n", (unsigned int)stacked_r12 );
    printf( "LR [R14] = 0x%08X\r\n", (unsigned int)stacked_lr );
    printf( "PC [R15] = 0x%08X\r\n", (unsigned int)stacked_pc );
    printf( "PSR = 0x%08X\r\n", (unsigned int)stacked_psr );

    while ( 1 )
        ;  // Loop forever to halt the system
}
/* USER CODE END 1 */
