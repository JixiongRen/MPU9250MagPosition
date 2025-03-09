//
// Created by renji on 25-2-21.
//

#ifndef BSP_UARTDMA_H
#define BSP_UARTDMA_H

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include "ringbuffer.h"

/**
 * @brief Structure representing a UART DMA transmit buffer.
 */
typedef struct {
    uint8_t *data;              ///< Pointer to the data buffer.
    uint16_t length;            ///< Length of the current data in the buffer.
    bool isSending;             ///< Flag indicating if DMA transmission is in progress.
    UART_HandleTypeDef *huart;  ///< Pointer to the UART handle.
    uint16_t bufferSize;        ///< Total size of the buffer.
} UartDmaTxBuffer;

/**
 * @brief Structure representing a UART DMA receive buffer.
 */
typedef struct {
    RingBuffer *ringBuffer;  ///< Pointer to the ring buffer.
    UART_HandleTypeDef *huart; ///< Pointer to the UART handle.
    uint16_t dmaSize;         ///< Size of data received by DMA each time.
} UartDmaRxBuffer;

extern RingBuffer *g_UART1_RxRingBuffer;
extern uint8_t g_rxBuffer1[1];
extern UartDmaRxBuffer g_UART2_RxDmaBuffer;
extern UartDmaTxBuffer  g_UART2_TxDmaBuffer;

void uartdma_tx_init(UartDmaTxBuffer *uartdma, UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t bufferSize);
bool uartdma_tx_write(UartDmaTxBuffer *uartdma, uint8_t *data, uint16_t length);
void uartdma_tx_callback(UartDmaTxBuffer *uartdma);
bool uartdma_tx_isEmpty(UartDmaTxBuffer *uartdma);
bool uartdma_rx_init(UartDmaRxBuffer *uartdma, UART_HandleTypeDef *huart, uint16_t bufferSize, uint16_t dmaSize, bool useSemaphore);
void uartdma_rx_callback(UartDmaRxBuffer *uartdma);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif //BSP_UARTDMA_H
