//
// Created by renji on 25-2-17.
//
#include "S_SPI_driver.h"

/**
 * @brief Set the Slave Select (SS) pin.
 * @param val Value to set the SS pin (1 for high, 0 for low).
 */
void S_SPI_SetSS(uint8_t val)
{
    HAL_GPIO_WritePin(S_SPI_SS_GPIO_GPIO_Port, S_SPI_SS_GPIO_Pin, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Set the Serial Clock (SCK) pin.
 * @param val Value to set the SCK pin (1 for high, 0 for low).
 */
void S_SPI_SetSCK(uint8_t val)
{
    HAL_GPIO_WritePin(S_SPI_SCK_GPIO_GPIO_Port, S_SPI_SCK_GPIO_Pin, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Set the Master Out Slave In (MOSI) pin.
 * @param val Value to set the MOSI pin (1 for high, 0 for low).
 */
void S_SPI_SetMOSI(uint8_t val)
{
    HAL_GPIO_WritePin(S_SPI_MOSI_GPIO_GPIO_Port, S_SPI_MOSI_GPIO_Pin, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Get the value of the Master In Slave Out (MISO) pin.
 * @return uint8_t Value of the MISO pin (1 for high, 0 for low).
 */
uint8_t S_SPI_GetMISO(void)
{
    return HAL_GPIO_ReadPin(S_SPI_MISO_GPIO_GPIO_Port, S_SPI_MISO_GPIO_Pin);
}

/**
 * @brief Initialize the SPI GPIO pins.
 */
void S_Init_SPI(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure SS, SCK, MOSI as push-pull output
    GPIO_InitStruct.Pin = S_SPI_SS_GPIO_Pin | S_SPI_MISO_GPIO_Pin | S_SPI_MOSI_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(S_SPI_MISO_GPIO_GPIO_Port, &GPIO_InitStruct);

    // Configure MISO as pull-up input
    GPIO_InitStruct.Pin = S_SPI_MISO_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(S_SPI_MISO_GPIO_GPIO_Port, &GPIO_InitStruct);

    // Initialize default levels
    S_SPI_SetSS(1);     // SS default high
    S_SPI_SetSCK(0);    // SCK default low
}

/**
 * @brief Start SPI communication by selecting the slave device.
 */
void S_SPI_Start(void)
{
    S_SPI_SetSS(0);    // Select the slave device
}

/**
 * @brief End SPI communication by releasing the slave device.
 */
void S_SPI_End(void)
{
    S_SPI_SetSS(1);    // Release the slave device
}

/**
 * @brief Swap a byte over SPI.
 * @param data Byte to be sent.
 * @return uint8_t Byte received.
 */
uint8_t S_SPI_SwapByte(uint8_t data)
{
    uint8_t receive = 0x00;

    for(uint8_t i = 0; i < 8; i++)
    {
        // Set MOSI
        S_SPI_SetMOSI((data & (0x80 >> i)) ? 1 : 0);

        // Rising edge
        S_SPI_SetSCK(1);

        // Read MISO
        if(S_SPI_GetMISO())
        {
            receive |= (0x80 >> i);
        }

        // Falling edge
        S_SPI_SetSCK(0);
    }

    return receive;
}