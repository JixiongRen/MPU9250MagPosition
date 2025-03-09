//
// Created by renji on 25-2-21.
//

#ifndef __BSP_RINGBUFFER_H
#define __BSP_RINGBUFFER_H

#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/**
 * @brief Structure representing a ring buffer.
 */
typedef struct {
    uint8_t *buffer;                    /**< Pointer to the buffer memory. */
    size_t head;                        /**< Index of the head pointer. */
    size_t tail;                        /**< Index of the tail pointer. */
    size_t maxLen;                      /**< Maximum length of the buffer. */
    SemaphoreHandle_t dataSemaphore;    /**< Semaphore for data availability notification. */
    BaseType_t useSemaphore;            /**< Flag indicating whether to use semaphore. */
} RingBuffer;

RingBuffer* createRingBuffer(size_t size, BaseType_t useSemaphore);
void destroyRingBuffer(RingBuffer *rb);
int16_t writeRingBuffer(RingBuffer *rb, const uint8_t *data, uint16_t len);
BaseType_t writeRingBufferFromISR(RingBuffer *rb, const uint8_t *data, uint16_t len, BaseType_t *pxHigherPriorityTaskWoken);
int16_t readRingBuffer(RingBuffer *rb, uint8_t *data, uint16_t len);
uint16_t availableRingBuffer(RingBuffer *rb);
uint8_t* getRingBufferWritePointer(RingBuffer *rb);
BaseType_t incWritePtrFromISR(RingBuffer *rb, size_t len, BaseType_t *pxHigherPriorityTaskWoken);

#endif // !__BSP_RINGBUFFER_H
