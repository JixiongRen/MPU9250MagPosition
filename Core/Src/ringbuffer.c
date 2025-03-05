//
// Created by renji on 25-2-21.
//

#include "ringbuffer.h"
/**
 * @brief                       创建一个环形缓冲区
 *
 * @param size                  缓冲区大小，单位是字节
 * @param useSemaphore          是否使用信号量
 * @return RingBuffer*          返回创建的环形缓冲区指针, 如果创建失败则返回NULL
 */
RingBuffer* bsp_createRingBuffer(size_t size, BaseType_t useSemaphore) {
    RingBuffer* rb = (RingBuffer *)pvPortMalloc(sizeof(RingBuffer));
    if (rb == NULL) return NULL;

    rb->buffer = (uint8_t *)pvPortMalloc(size);
    if (rb->buffer == NULL) {  // 缓冲区创建失败
        vPortFree(rb);
        return NULL;
    }

    rb->maxLen = size;
    rb->head = 0;
    rb->tail = 0;
    rb->useSemaphore = useSemaphore;

    if (useSemaphore) {
        rb->dataSemaphore = xSemaphoreCreateBinary(); // 创建二值信号量
        if (rb->dataSemaphore == NULL) {  // 信号量创建失败
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
 * @brief               释放环形缓冲区
 *
 * @param rb            环形缓冲区指针
 */
void bsp_destroyRingBuffer(RingBuffer *rb) {
    if (rb) {                                   // 当环形缓冲区指针不为空时
        vPortFree(rb->buffer);                  // 释放缓冲区
        vSemaphoreDelete(rb->dataSemaphore);    // 删除信号量
        vPortFree(rb);                          // 释放环形缓冲区
    }
}

/**
 * @brief               写入环形缓冲区
 *
 * @param rb            环形缓冲区指针
 * @param data          数据
 * @param len           数据长度
 * @return int16_t      返回写入的数据长度, 如果返回-1则表示缓冲区空间不足
 */
int16_t bsp_writeRingBuffer(RingBuffer *rb, const uint8_t *data, uint16_t len) {
    uint16_t freeSpace = rb->maxLen - (rb->head - rb->tail);
    if (len > freeSpace) {
        return -1; // 缓冲区空间不足
    }
    for (uint16_t i = 0; i < len; i++) {
        rb->buffer[rb->head % rb->maxLen] = data[i]; // 环形写入数据，求模防止溢出
        rb->head++;
    }

    if (rb->useSemaphore) {
        // 通知有数据可读
        xSemaphoreGive(rb->dataSemaphore);
    }

    return len;
}

/**
 * @brief                               从中断服务函数中写入环形缓冲区
 *
 * @param rb                            环形缓冲区指针
 * @param data                          数据
 * @param len                           数据长度
 * @param pxHigherPriorityTaskWoken     是否唤醒更高优先级任务
 * @return BaseType_t                   返回写入结果
 */
BaseType_t bsp_writeRingBufferFromISR(RingBuffer *rb, const uint8_t *data, uint16_t len, BaseType_t *pxHigherPriorityTaskWoken) {
    size_t freeSpace = rb->maxLen - (rb->head - rb->tail); // 计算剩余空间

    if (len > freeSpace) {  // 要写入的长度大于空间长度，写入失败
        return pdFAIL;
    }

    for (size_t i = 0; i < len; i++) {
        rb->buffer[rb->head % rb->maxLen] = data[i];
        rb->head++;
    }

    if (rb->useSemaphore) {
        // 通知有数据可读，唤醒更高优先级任务
        xSemaphoreGiveFromISR(rb->dataSemaphore, pxHigherPriorityTaskWoken); // TODO:学一下这种写法
    }

    return pdPASS; // 写入成功
}

/**
 * @brief                               获取环形缓冲区写指针,dma接收时使用
 *
 * @param rb                            环形缓冲区指针
 * @return uint8_t*                     返回写指针
 */
uint8_t* bsp_getRingBufferWritePointer(RingBuffer *rb) {
    if (rb == NULL) {
        return NULL;
    }
    // 检查是否有足够的空间
    if (rb->maxLen - (rb->head - rb->tail) == 0) {
        return NULL;
    }
    return &rb->buffer[rb->head % rb->maxLen];
}

/**
 * @brief
 *
 * @param rb                            递增写指针，dma接收时使用
 * @param len                           数据长度
 * @param pxHigherPriorityTaskWoken     是否唤醒更高优先级任务
 * @return BaseType_t                   返回写入结果
 */
BaseType_t bsp_incWritePtrFromISR(RingBuffer *rb, size_t len, BaseType_t *pxHigherPriorityTaskWoken)
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
 * @brief                   读取环形缓冲区
 *
 * @param rb                环形缓冲区指针
 * @param data              数据
 * @param len               数据长度
 * @return int16_t          返回读取的数据长度, 如果返回-1则表示读取失败
 */
int16_t bsp_readRingBuffer(RingBuffer *rb, uint8_t *data, uint16_t len) {
    if (rb->useSemaphore) {
        // 等待信号量通知有数据可读
        if (xSemaphoreTake(rb->dataSemaphore, portMAX_DELAY) != pdTRUE) {
            return -1;
        }
    }

    size_t availableData = rb->head - rb->tail;

    if (availableData == 0) {
        return 0; // 安全检查，确保有数据可读
    }

    if (len > availableData) {
        len = availableData; // 调整读取长度
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = rb->buffer[rb->tail % rb->maxLen];
        rb->tail++;
    }

    return len;
}

/**
 * @brief               获取环形缓冲区中可读取的数据长度
 *
 * @param rb            环形缓冲区指针
 * @return uint16_t
 */
uint16_t bsp_availableRingBuffer(RingBuffer *rb) {
    return rb->head - rb->tail;
}