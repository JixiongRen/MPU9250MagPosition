//
// Created by renji on 25-2-21.
//

#include "ringbuffer.h"


/**
 * @brief   Create a ring buffer
 * @param size  Size of the buffer in bytes
 * @param useSemaphore  Whether to use a semaphore
 * @return RingBuffer*  Returns a pointer to the created ring buffer, or NULL if creation fails
 */
RingBuffer* createRingBuffer(size_t size, BaseType_t useSemaphore) {
    RingBuffer* rb = (RingBuffer *)pvPortMalloc(sizeof(RingBuffer));
    if (rb == NULL) return NULL;

    rb->buffer = (uint8_t *)pvPortMalloc(size);
    if (rb->buffer == NULL) {  // Buffer creation failed
        vPortFree(rb);
        return NULL;
    }

    rb->maxLen = size;
    rb->head = 0;
    rb->tail = 0;
    rb->useSemaphore = useSemaphore;

    if (useSemaphore) {
    rb->dataSemaphore = xSemaphoreCreateBinary(); // Create a binary semaphore
    if (rb->dataSemaphore == NULL) {  // Semaphore creation failed
            vPortFree(rb->buffer);
            vPortFree(rb);
            return NULL;
        }
    } else {
        rb->dataSemaphore = NULL;
    }
    return rb;
}


/**
 * @brief   Release the ring buffer
 * @param rb    Pointer to the ring buffer
 */
void destroyRingBuffer(RingBuffer *rb) {
    if (rb) {                                   // When the ring buffer pointer is not NULL
        vPortFree(rb->buffer);                  // Free the buffer
        vSemaphoreDelete(rb->dataSemaphore);    // Delete the semaphore
        vPortFree(rb);                          // Free the ring buffer
    }
}

/**
* @brief    Write to the ring buffer
* @param rb Pointer to the ring buffer
* @param data   Data
* @param len    Data length
* @return int16_t   Returns the length of the written data, or -1 if the buffer is full
*/
int16_t writeRingBuffer(RingBuffer *rb, const uint8_t *data, uint16_t len) {
    uint16_t freeSpace = rb->maxLen - (rb->head - rb->tail);
    if (len > freeSpace) {
        return -1; // Buffer space not enough
    }
    for (uint16_t i = 0; i < len; i++) {
        rb->buffer[rb->head % rb->maxLen] = data[i]; // Write data to the buffer
        rb->head++;
    }

    if (rb->useSemaphore) {
        // Notify that there is data to read, wake up a higher priority task
        xSemaphoreGive(rb->dataSemaphore);
    }

    return len;
}

/**
 * @brief   Write to the ring buffer from an interrupt service routine
 * @param rb    Pointer to the ring buffer
 * @param data  Data
 * @param len   Data length
 * @param pxHigherPriorityTaskWoken Whether to wake up a higher priority task
 * @return BaseType_t   Returns the result of the write operation
 */
BaseType_t writeRingBufferFromISR(RingBuffer *rb, const uint8_t *data, uint16_t len, BaseType_t *pxHigherPriorityTaskWoken) {
    size_t freeSpace = rb->maxLen - (rb->head - rb->tail); // Calculate the free space in the buffer

    if (len > freeSpace) {  // Check if there is enough space
        return pdFAIL;
    }

    for (size_t i = 0; i < len; i++) {
        rb->buffer[rb->head % rb->maxLen] = data[i];
        rb->head++;
    }

    if (rb->useSemaphore) {
        // Notify that there is data to read, wake up a higher priority task
        xSemaphoreGiveFromISR(rb->dataSemaphore, pxHigherPriorityTaskWoken);
    }

    return pdPASS; // Write operation successful
}

/**
* @brief    Get the write pointer of the ring buffer, used for DMA reception
* @param rb Pointer to the ring buffer
* @return uint8_t*  Returns the write pointer
*/
uint8_t* getRingBufferWritePointer(RingBuffer *rb) {
    if (rb == NULL) {
        return NULL;
    }
    // Check if the buffer is full
    if (rb->maxLen - (rb->head - rb->tail) == 0) {
        return NULL;
    }
    return &rb->buffer[rb->head % rb->maxLen];
}

/**
* @brief Increment the write pointer, used for DMA reception
* @param rb Pointer to the ring buffer
* @param len    Data length
* @param pxHigherPriorityTaskWoken  Whether to wake up a higher priority task
* @return BaseType_t    Returns the result of the write operation
*/
BaseType_t incWritePtrFromISR(RingBuffer *rb, size_t len, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (rb == NULL) {
        return pdFAIL;
    }

    rb->head += len;

    if (rb->useSemaphore) {
        xSemaphoreGiveFromISR(rb->dataSemaphore, pxHigherPriorityTaskWoken);
    }

    return pdPASS;
}



/**
* @brief   Read from the ring buffer
* @param rb    Pointer to the ring buffer
* @param data  Data
* @param len   Data length
* @return int16_t  Returns the length of the read data, or -1 if the read fails
*/
int16_t readRingBuffer(RingBuffer *rb, uint8_t *data, uint16_t len) {
    if (rb->useSemaphore) {
        // Wait for data to be available
        if (xSemaphoreTake(rb->dataSemaphore, portMAX_DELAY) != pdTRUE) {
            return -1;
        }
    }

    size_t availableData = rb->head - rb->tail;

    if (availableData == 0) {
        return 0; // No data available
    }

    if (len > availableData) {
        len = availableData; // Read all available data
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = rb->buffer[rb->tail % rb->maxLen];
        rb->tail++;
    }

    return len;
}

/**
* @brief   Get the length of readable data in the ring buffer
* @param rb    Pointer to the ring buffer
* @return uint16_t
*/
uint16_t availableRingBuffer(RingBuffer *rb) {
    return rb->head - rb->tail;
}