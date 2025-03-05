//
// Created by renji on 25-2-21.
//

#ifndef BSP_UARTDMA_H
#define BSP_UARTDMA_H

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include "ringbuffer.h"

typedef struct {
    uint8_t *data;              // 指向数据缓冲区的指针
    uint16_t length;            // 当前缓冲区中数据的长度
    bool isSending;             // 表示是否正在进行 DMA 发送的标志
    UART_HandleTypeDef *huart;  // 指向 UART 句柄的指针
    uint16_t bufferSize;        // 缓冲区的总大小
} UartDmaTxBuffer;

typedef struct {
    RingBuffer *ringBuffer;  // 指向环形缓冲区的指针
    UART_HandleTypeDef *huart; // 指向 UART 句柄的指针
    uint16_t dmaSize;         // DMA 每次接收的数据大小
} UartDmaRxBuffer;

extern RingBuffer *g_UART1_RxRingBuffer;
extern uint8_t g_rxBuffer1[1];
extern UartDmaRxBuffer g_UART2_RxDmaBuffer;
extern UartDmaTxBuffer  g_UART2_TxDmaBuffer;

// DMA 发送初始化函数
void bsp_uartdma_tx_init(UartDmaTxBuffer *uartdma, UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t bufferSize);
// 写入数据到 DMA 缓冲区函数
bool bsp_uartdma_tx_write(UartDmaTxBuffer *uartdma, uint8_t *data, uint16_t length);
// DMA 发送完成回调函数
void bsp_uartdma_tx_callback(UartDmaTxBuffer *uartdma);
// 查询 DMA 是否发送清空函数
bool bsp_uartdma_tx_isEmpty(UartDmaTxBuffer *uartdma);
// DMA 接收初始化函数
bool bsp_uartdma_rx_init(UartDmaRxBuffer *uartdma, UART_HandleTypeDef *huart, uint16_t bufferSize, uint16_t dmaSize, bool useSemaphore);
// DMA 接收完成回调函数
void bsp_uartdma_rx_callback(UartDmaRxBuffer *uartdma);
// 串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
// 串口发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif //BSP_UARTDMA_H
