//
// Created by renji on 25-2-6.
//
#include "func_delay.h"
#include "cmsis_os.h"

uint8_t rtos_init_flag = 0;

/**
 * @brief 可以根据 rtos_init_flag 的值选择使用 HAL_Delay 还是 osDelay
 * @param rtos_init_flag 判断是否使用 RTOS 的标志
 * @param ms 延时时长, ms
 */
void SYS_Delay(uint8_t rtos_init_flag, uint8_t ms) {
    if (rtos_init_flag == 0) {
        HAL_Delay(ms);
    } else {
        osDelay(ms);
    }
}