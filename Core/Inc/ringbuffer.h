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

typedef struct {
    uint8_t *buffer;                    // 缓冲区首地址
    size_t head;                        // 头指针
    size_t tail;                        // 尾指针
    size_t maxLen;                      // 缓冲区长度
    SemaphoreHandle_t dataSemaphore;    // 用于通知数据可用
    BaseType_t useSemaphore;            // 是否使用信号量
} RingBuffer;

RingBuffer* bsp_createRingBuffer(size_t size, BaseType_t useSemaphore);
void bsp_destroyRingBuffer(RingBuffer *rb);
int16_t bsp_writeRingBuffer(RingBuffer *rb, const uint8_t *data, uint16_t len);
BaseType_t bsp_writeRingBufferFromISR(RingBuffer *rb, const uint8_t *data, uint16_t len, BaseType_t *pxHigherPriorityTaskWoken);
int16_t bsp_readRingBuffer(RingBuffer *rb, uint8_t *data, uint16_t len);
uint16_t bsp_availableRingBuffer(RingBuffer *rb);
uint8_t* bsp_getRingBufferWritePointer(RingBuffer *rb);
BaseType_t bsp_incWritePtrFromISR(RingBuffer *rb, size_t len, BaseType_t *pxHigherPriorityTaskWoken);

#endif // !__BSP_RINGBUFFER_H
