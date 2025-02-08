//
// Created by renji on 25-2-7.
//
#include "func_task.h"

/**
 * @brief FreeRTOS 读取加速度数据任务
 * @param argument
 */
void StartGetAccelDataTask(void *argument) {
    for (;;) {
        MPU9250_ReadAccel(mpu_1);
        MPU9250_ReadAccel(mpu_2);
        osDelay(1);
    }
}

/**
 * @brief FreeRTOS 读取陀螺仪数据任务
 * @param argument
 */
void StartGetGyroDataTask(void *argument) {
    for (;;) {
        MPU9250_ReadGyro(mpu_1);
        MPU9250_ReadGyro(mpu_2);
        osDelay(1);
    }
}

/**
 * @brief FreeRTOS 读取磁力计数据任务
 * @param argument
 */
void StartGetMagnDataTask(void *argument) {
    uint32_t start_time, end_time, duration;
    for (;;) {
        start_time = HAL_GetTick();
        MPU9250_ReadMag(mpu_1);
        end_time = HAL_GetTick();
        duration = end_time - start_time;
        if (end_time < start_time) {
            duration = (0xFFFFFFFF - start_time) + end_time;
        }
        MPU9250_ReadMag(mpu_2);
        osDelay(1);
    }
}

/**
 * @brief 串口传输任务
 * @param argument
 */
void StartUsartTransTask(void *argument) {
    for (;;) {
    //
    }
}