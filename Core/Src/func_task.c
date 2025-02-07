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
    for (;;) {
        MPU9250_ReadMag(mpu_1);
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

    }
}