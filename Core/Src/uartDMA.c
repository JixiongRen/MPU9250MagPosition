//
// Created by renji on 25-2-21.
//
#include "uartDMA.h"
#include "usart.h"

/**
 * @brief               初始化DMA
 *
 * @param uartdma       dma发送结构体指针
 * @param huart         发送所使用的UART句柄
 * @param buffer        指向发送缓冲区的指针
 * @param bufferSize    发送缓冲区大小
 */

void bsp_uartdma_tx_init(UartDmaTxBuffer *uartdma, UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t bufferSize) {
    uartdma->data = buffer;             // 指向数据缓冲区的指针
    uartdma->length = 0;                // 当前缓冲区的数据长度
    uartdma->isSending = false;         // 是否正在进行 DMA 发送的标志
    uartdma->huart = huart;             // 发送所使用的 UART 句柄
    uartdma->bufferSize = bufferSize;   // 发送缓冲区大小
}

/**
 * @brief               写入数据到DMA缓冲区
 *
 * @param uartdma       dma发送结构体指针
 * @param data          要写入的数据
 * @param length        数据长度
 * @return bool         写入成功返回 true，失败返回 false
 */
bool bsp_uartdma_tx_write(UartDmaTxBuffer *uartdma, uint8_t *data, uint16_t length) {
    // 检查数据长度是否大于缓冲区长度
    if (length > uartdma->bufferSize) {
        return false; // 待写入的数据长度大于缓冲区长度，写入失败
    }

    // 检查是否正在发送
    if (uartdma->isSending) {
        return false; // 正在发送中，写入失败
    }

    // 复制数据到缓冲区
    memcpy(uartdma->data, data, length);
    uartdma->length = length;

    // 启动 DMA 发送
    if (HAL_UART_Transmit_DMA(uartdma->huart, uartdma->data, uartdma->length) != HAL_OK) {
        return false; // DMA 启动失败
    }

    // 设定正在发送中
    uartdma->isSending = true;
    return true; // 写入成功
}

/**
 * @brief DMA 发送完成回调函数
 *
 * @param uartdma       dma发送结构体指针
 */
void bsp_uartdma_tx_callback(UartDmaTxBuffer *uartdma) {
    uartdma->isSending = false;
    // 发送完成后对优先级进行切换，保证高优先级任务能及时执行
    //BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    //portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief               查询DMA是否发送清空
 *
 * @param uartdma       dma发送结构体指针
 */
bool bsp_uartdma_tx_isEmpty(UartDmaTxBuffer *uartdma) {
    return !uartdma->isSending; // 如果正在发送中（True），说明发送尚未清空（False）
}



/**
 * @brief               初始化DMA接收缓冲区
 *
 * @param uartdma       接收所使用的DMA接收缓冲区结构体指针
 * @param huart         接收所使用的UART句柄
 * @param bufferSize    接收缓冲区大小
 * @param dmaSize       DMA每次接收的数据大小
 * @param useSemaphore  是否使用信号量
 *
 * @return bool         初始化成功返回 true，失败返回 false
 */
bool bsp_uartdma_rx_init(UartDmaRxBuffer *uartdma, UART_HandleTypeDef *huart, uint16_t bufferSize, uint16_t dmaSize, bool useSemaphore) {
    uartdma->ringBuffer = bsp_createRingBuffer(bufferSize, useSemaphore);
    if (uartdma->ringBuffer == NULL) {
        return false;  // 创建环形缓冲区失败
    }
    uartdma->huart = huart;
    uartdma->dmaSize = dmaSize;
    //DEBUG_PRINTF(LEVEL_DEBUG, "dmaSize: %d\r\n", dmaSize);
    uint8_t *writePointer = bsp_getRingBufferWritePointer(uartdma->ringBuffer); // 获取环形缓冲区写指针
    if (writePointer == NULL) {
        return false; // 若没有足够的空间或缓冲区为空，返回失败
    }
    HAL_UART_Receive_DMA(huart, writePointer, dmaSize); // 启动DMA接收
    return true;
}

/**
 * @brief               DMA接收完成回调函数， 只能在中断中调用。
 *
 * @param uartdma       DMA接收缓冲区结构体指针
 */
void bsp_uartdma_rx_callback(UartDmaRxBuffer *uartdma) {
    // 通知有数据可读
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // xHigherPriorityTaskWoken 是一个标志位，用于指示是否需要进行任务切换
    bsp_incWritePtrFromISR(uartdma->ringBuffer, uartdma->dmaSize, &xHigherPriorityTaskWoken);

    // 重新启动DMA接收
    uint8_t *writePointer = bsp_getRingBufferWritePointer(uartdma->ringBuffer);
    if (writePointer != NULL) {
        // DEBUG_PRINTF(LEVEL_DEBUG, "DMA RX Start!\r\n");
        // HAL_UART_DMAStop(uartdma->huart);
        HAL_UART_Receive_DMA(uartdma->huart, writePointer, uartdma->dmaSize);
    }else{
        //DEBUG_PRINTF(LEVEL_ERROR, "DMA RX Buffer is full!\r\n");
    }
    // 切换上下文
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


RingBuffer *g_UART1_RxRingBuffer;
uint8_t g_rxBuffer1[1];
UartDmaRxBuffer g_UART2_RxDmaBuffer;
/**
 * @brief           串口接收中断回调函数
 *
 * @param huart     串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (huart->Instance == USART1){
    if (bsp_writeRingBufferFromISR(g_UART1_RxRingBuffer, g_rxBuffer1, 1, &xHigherPriorityTaskWoken) != pdPASS){
      //DEBUG_PRINTF(LEVEL_ERROR, "Write UART1 Rx RingBuffer failed!\r\n");
    }
    HAL_UART_Receive_IT(&huart1, g_rxBuffer1, 1);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  if (huart->Instance == USART2){
    bsp_uartdma_rx_callback(&g_UART2_RxDmaBuffer);          // DMA接收完成回调
  }
}


UartDmaTxBuffer  g_UART2_TxDmaBuffer;
/**
 * @brief           串口发送完成回调函数
 *
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2){
        bsp_uartdma_tx_callback(&g_UART2_TxDmaBuffer);
    }
}