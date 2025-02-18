//
// Created by renji on 25-2-17.
//
#include "S_SPI_driver.h"

void S_SPI_SetSS(uint8_t val)
{
    HAL_GPIO_WritePin(S_SPI_SS_GPIO_GPIO_Port, S_SPI_SS_GPIO_Pin, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void S_SPI_SetSCK(uint8_t val)
{
    HAL_GPIO_WritePin(S_SPI_SCK_GPIO_GPIO_Port, S_SPI_SCK_GPIO_Pin, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void S_SPI_SetMOSI(uint8_t val)
{
    HAL_GPIO_WritePin(S_SPI_MOSI_GPIO_GPIO_Port, S_SPI_MOSI_GPIO_Pin, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t S_SPI_GetMISO(void)
{
    return HAL_GPIO_ReadPin(S_SPI_MISO_GPIO_GPIO_Port, S_SPI_MISO_GPIO_Pin);
}

void S_Init_SPI(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置SS, SCK, MOSI为推挽输出
    GPIO_InitStruct.Pin = S_SPI_SS_GPIO_Pin | S_SPI_MISO_GPIO_Pin | S_SPI_MOSI_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(S_SPI_MISO_GPIO_GPIO_Port, &GPIO_InitStruct);

    // 配置MISO为上拉输入
    GPIO_InitStruct.Pin = S_SPI_MISO_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(S_SPI_MISO_GPIO_GPIO_Port, &GPIO_InitStruct);

    // 初始化默认电平
    S_SPI_SetSS(1);     // SS默认高电平
    S_SPI_SetSCK(0);    // SCK默认低电平
}

void S_SPI_Start(void)
{
    S_SPI_SetSS(0);    // 选中从设备
}

void S_SPI_End(void)
{
    S_SPI_SetSS(1);    // 释放从设备
}

uint8_t S_SPI_SwapByte(uint8_t data)
{
    uint8_t receive = 0x00;

    for(uint8_t i = 0; i < 8; i++)
    {
        // 设置MOSI
        S_SPI_SetMOSI((data & (0x80 >> i)) ? 1 : 0);

        // 上升沿
        S_SPI_SetSCK(1);

        // 读取MISO
        if(S_SPI_GetMISO())
        {
            receive |= (0x80 >> i);
        }

        // 下降沿
        S_SPI_SetSCK(0);
    }

    return receive;
}

