//
// Created by renji on 25-2-21.
//
#include "uartDMA.h"
#include "usart.h"

/**
 * @brief  Initialize the DMA send buffer
 *
 * @param uartdma DMA send buffer structure pointer
 * @param huart  UART handle used for sending
 * @param buffer  Data buffer
 * @param bufferSize Buffer size
 */

void uartdma_tx_init(UartDmaTxBuffer *uartdma, UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t bufferSize) {
    uartdma->data = buffer;             // Data buffer to be sent
    uartdma->length = 0;                // Data length
    uartdma->isSending = false;         // Whether it is sending
    uartdma->huart = huart;             // UART handle
    uartdma->bufferSize = bufferSize;   // Buffer size
}

/**
 * @brief Write data to the DMA send buffer
 *
 * @param uartdma DMA send buffer structure pointer
 * @param data  Data to be written
 * @param length Data length
 * @return bool  Write success returns true, failure returns false
 */
bool uartdma_tx_write(UartDmaTxBuffer *uartdma, uint8_t *data, uint16_t length) {
    // Check if the data length to be written is greater than the buffer length
    if (length > uartdma->bufferSize) {
        return false; // Write failed
    }

    // Check if the data is being sent
    if (uartdma->isSending) {
        return false; // Write failed
    }

    // Copy the data to the buffer
    memcpy(uartdma->data, data, length);
    uartdma->length = length;

    // Start DMA transmission
    if (HAL_UART_Transmit_DMA(uartdma->huart, uartdma->data, uartdma->length) != HAL_OK) {
        return false; // Write failed
    }

    // Set the sending flag
    uartdma->isSending = true;
    return true; // Write successful
}

/**
* @brief DMA send completion callback function
* @param uartdma DMA send buffer structure pointer
*/
void uartdma_tx_callback(UartDmaTxBuffer *uartdma) {
    uartdma->isSending = false;
}

/**
 * @brief Check if the DMA send buffer is empty
 * @param uartdma DMA send buffer structure pointer
 * @return bool  If it is empty, return true, if it is not empty, return false
 */
bool uartdma_tx_isEmpty(UartDmaTxBuffer *uartdma) {
    return !uartdma->isSending; // If it is not sending, it is empty
}

/**
 * @brief Initialize the DMA receive buffer
 * @param uartdma DMA receive buffer structure pointer
 * @param huart  UART handle used for receiving
 * @param bufferSize  Buffer size
 * @param dmaSize  DMA buffer size
 * @param useSemaphore  Whether to use a semaphore
 * @return bool  Initialization success returns true, failure returns false
 */
bool uartdma_rx_init(UartDmaRxBuffer *uartdma, UART_HandleTypeDef *huart, uint16_t bufferSize, uint16_t dmaSize, bool useSemaphore) {
    uartdma->ringBuffer = createRingBuffer(bufferSize, useSemaphore);
    if (uartdma->ringBuffer == NULL) {
        return false;  // Initialization failed
    }
    uartdma->huart = huart;
    uartdma->dmaSize = dmaSize;
    //DEBUG_PRINTF(LEVEL_DEBUG, "dmaSize: %d\r\n", dmaSize);
    uint8_t *writePointer = getRingBufferWritePointer(uartdma->ringBuffer); // Get the write pointer of the ring buffer
    if (writePointer == NULL) {
        return false; // Initialization failed
    }
    HAL_UART_Receive_DMA(huart, writePointer, dmaSize); // Start DMA reception
    return true;
}

/**
 * @brief DMA receive completion callback function
 * @param uartdma DMA receive buffer structure pointer
 */
void uartdma_rx_callback(UartDmaRxBuffer *uartdma) {
    // Notify that there is data to read, wake up a higher priority task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Whether to wake up a higher priority task
    incWritePtrFromISR(uartdma->ringBuffer, uartdma->dmaSize, &xHigherPriorityTaskWoken);
    // Get the write pointer of the ring buffer, used for DMA reception
    uint8_t *writePointer = getRingBufferWritePointer(uartdma->ringBuffer);
    if (writePointer != NULL) {
        // DEBUG_PRINTF(LEVEL_DEBUG, "DMA RX Start!\r\n");
        // HAL_UART_DMAStop(uartdma->huart);
        HAL_UART_Receive_DMA(uartdma->huart, writePointer, uartdma->dmaSize);
    }else{
        //DEBUG_PRINTF(LEVEL_ERROR, "DMA RX Buffer is full!\r\n");
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


RingBuffer *g_UART1_RxRingBuffer;
uint8_t g_rxBuffer1[1];
UartDmaRxBuffer g_UART2_RxDmaBuffer;

/**
 * @brief Initialize the UART1 and UART2 DMA receive buffer
 * @param huart1 UART1 handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (huart->Instance == USART1){
    if (writeRingBufferFromISR(g_UART1_RxRingBuffer, g_rxBuffer1, 1, &xHigherPriorityTaskWoken) != pdPASS){
      //DEBUG_PRINTF(LEVEL_ERROR, "Write UART1 Rx RingBuffer failed!\r\n");
    }
    HAL_UART_Receive_IT(&huart1, g_rxBuffer1, 1);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  if (huart->Instance == USART2){
    uartdma_rx_callback(&g_UART2_RxDmaBuffer);
  }
}


UartDmaTxBuffer  g_UART2_TxDmaBuffer;

/**
 * @brief UART2 DMA send completion callback function\
 * @param huart UART handle
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2){
        uartdma_tx_callback(&g_UART2_TxDmaBuffer);
    }
}