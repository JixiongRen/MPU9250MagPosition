//
// Created by renji on 25-2-6.
//
#include "func_delay.h"
#include "cmsis_os.h"

uint8_t rtos_init_flag = 0;

void SYS_Delay(uint8_t rtos_init_flag, uint8_t ms) {
    if (rtos_init_flag == 0) {
        HAL_Delay(ms);
    } else {
        osDelay(ms);
    }
}