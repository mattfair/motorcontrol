#include "canringbuffer.h"

void CanRingBuffer_Init(CanRingBuffer* rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->size = 0;
    memset(rb->buffer, 0, sizeof(rb->buffer));
}

bool CanRingBuffer_IsFull(const CanRingBuffer* rb)
{
    return rb->size == RING_BUFFER_SIZE;
}

bool CanRingBuffer_IsEmpty(const CanRingBuffer* rb)
{
    return rb->size == 0;
}

bool CanRingBuffer_Enqueue(CanRingBuffer* rb, const CAN_Message* msg)
{
    if (CanRingBuffer_IsFull(rb))
    {
        return false;  // Buffer is full
    }

    rb->buffer[rb->head] = *msg;  // Copy the message to the buffer
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
    rb->size++;
    return true;
}

bool CanRingBuffer_Dequeue(CanRingBuffer* rb, CAN_Message* msg)
{
    if (CanRingBuffer_IsEmpty(rb))
    {
        return false;  // Buffer is empty
    }

    *msg = rb->buffer[rb->tail];  // Copy the message from the buffer
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    rb->size--;
    return true;
}

CAN_Message* CanRingBuffer_GetNextWriteSlot(CanRingBuffer* rb)
{
    if (CanRingBuffer_IsFull(rb))
    {
        return NULL;  // Buffer is full
    }

    return &rb->buffer[rb->head];
}

bool CanRingBuffer_CommitWriteSlot(CanRingBuffer* rb)
{
    if (CanRingBuffer_IsFull(rb))
    {
        return false;  // Buffer is full
    }

    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
    rb->size++;
    return true;
}
