//
// Created by renji on 25-2-6.
//

#ifndef FUNC_DELAY_H
#define FUNC_DELAY_H

#include "stm32f4xx_hal.h"

extern uint8_t rtos_init_flag;

void SYS_Delay(uint8_t rtos_init_flag, uint8_t ms);

#endif //FUNC_DELAY_H
