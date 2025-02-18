//
// Created by renji on 25-2-17.
//

#ifndef S_SPI_DRIVER_H
#define S_SPI_DRIVER_H

#include "main.h"
#include "stm32f4xx_hal.h"

void S_SPI_SetSS(uint8_t val);
void S_SPI_SetSCK(uint8_t val);
void S_SPI_SetMOSI(uint8_t val);
uint8_t S_SPI_GetMISO(void);
void S_Init_SPI(void);
void S_SPI_Start(void);
void S_SPI_End(void);
uint8_t S_SPI_SwapByte(uint8_t data);


#endif //S_SPI_DRIVER_H
