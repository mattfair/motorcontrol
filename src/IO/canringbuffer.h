#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "canard.h"
#include "stm32f4xx_hal.h"

#define RING_BUFFER_SIZE 32u

typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t payload[CANARD_MTU_CAN_CLASSIC];
} CAN_Message;

typedef struct
{
    CAN_Message buffer[RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t size;
} CanRingBuffer;

/**
 * @brief Initializes the ring buffer.
 * @param rb The ring buffer to initialize.
 */
void CanRingBuffer_Init( CanRingBuffer* rb );

/**
 * @brief Checks if the ring buffer is full.
 * @return true if the ring buffer is full, false otherwise.
 */
bool CanRingBuffer_IsFull( const CanRingBuffer* rb );

/**
 * @brief Checks if the ring buffer is empty.
 * @return true if the ring buffer is empty, false otherwise.
 */
bool CanRingBuffer_IsEmpty( const CanRingBuffer* rb );

/**
 * @brief Enqueues a CAN message in the ring buffer.
 * @param rb The ring buffer.
 * @param msg The CAN message to enqueue.
 * @return true if the message was enqueued successfully, false otherwise.
 */
bool CanRingBuffer_Enqueue( CanRingBuffer* rb, const CAN_Message* msg );

/**
 * @brief Dequeues a CAN message from the ring buffer.
 * @param rb The ring buffer.
 * @param msg The CAN message to dequeue.
 * @return true if the message was dequeued successfully, false otherwise.
 */
bool CanRingBuffer_Dequeue( CanRingBuffer* rb, CAN_Message* msg );

/**
 * @brief Gets the next buffer slot to write a new CAN message directly.
 * @param rb The ring buffer.
 * @return Pointer to the next buffer slot, or NULL if the buffer is full.
 */
CAN_Message* CanRingBuffer_GetNextWriteSlot( CanRingBuffer* rb );

/**
 * @brief Commits the data written to the next buffer slot.
 * @param rb The ring buffer.
 * @return true if the data was committed successfully, false otherwise.
 */
bool CanRingBuffer_CommitWriteSlot( CanRingBuffer* rb );
