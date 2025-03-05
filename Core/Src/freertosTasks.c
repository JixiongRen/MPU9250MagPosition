//
// Created by renji on 25-2-20.
//
#include "uartDMA.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOSVariables.h"
#include "usart.h"
#include <stdio.h>

float MagData[36] = {0.0}; // 全局变量
char buffer[256] = {0};     // 全局变量
uint8_t g_txBuffer[2048] = {0};

// 定义汇总数据的结构体
typedef struct {
    SPI_SensorsGroup group1;
    SPI_SensorsGroup group2;
    SPI_SensorsGroup group3;
} SensorDataSummary_t;


void StartrSpi01MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi01MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask01Handle, portMAX_DELAY) == pdTRUE) {
            // Simulate the time required to complete the SPI transaction
            SensorGroup_ReadMag(spi_sensorsgroup_1);
        }
        if (xQueueOverwrite(sensorGroupQueue01Handle, &spi_sensorsgroup_1) == pdPASS) {
            xEventGroupSetBits(xSensorEventGroup, SENSOR_GROUP1_BIT);
        } else {
            //printf("send failed\r\n");
        }
        //vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void StartrSpi02MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi02MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask02Handle, portMAX_DELAY) == pdTRUE) {
            SensorGroup_ReadMag(spi_sensorsgroup_2);
        }
        if (xQueueOverwrite(sensorGroupQueue02Handle, &spi_sensorsgroup_2) == pdPASS) {
            //printf("send success\r\n");
            xEventGroupSetBits(xSensorEventGroup, SENSOR_GROUP2_BIT);
        } else {
            //printf("send failed\r\n");
        }
        //vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void StartrSpi03MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi02MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask03Handle, portMAX_DELAY) == pdTRUE) {
            SensorGroup_ReadMag(spi_sensorsgroup_3);
        }
        if (xQueueOverwrite(sensorGroupQueue03Handle, &spi_sensorsgroup_3) == pdPASS) {
            //printf("send success\r\n");
            xEventGroupSetBits(xSensorEventGroup, SENSOR_GROUP3_BIT);
        } else {
            //printf("send failed\r\n");
        }
        //vTaskDelay(pdMS_TO_TICKS(1));
    }
}


// void StartUsartTransTask(void *argument) {
//     SensorDataSummary_t summary;
//     uint8_t txBuffer[2048] = {0}; // 发送缓冲区
//     uint16_t txLength;      // 发送数据长度
//
//     /* Infinite loop */
//     for (;;) {
//         //printf("waiting for sensor data...\r\n");
//         //等待所有三个事件位就绪
//           xEventGroupWaitBits(
//               xSensorEventGroup,
//               SENSOR_GROUP1_BIT | SENSOR_GROUP2_BIT | SENSOR_GROUP3_BIT,
//               pdTRUE,
//               pdTRUE,
//               portMAX_DELAY
//           );
//
//         SPI_SensorsGroup *group1 = NULL;
//         SPI_SensorsGroup *group2 = NULL;
//         SPI_SensorsGroup *group3 = NULL;
//
//         // 读取最新数据（队列长度1，直接读取）
//         xQueueReceive(sensorGroupQueue01Handle, &group1, 0);
//         xQueueReceive(sensorGroupQueue02Handle, &group2, 0);
//         xQueueReceive(sensorGroupQueue03Handle, &group3, 0);
//
//         // 检查指针有效性
//         if (group1 && group2 && group3) {
//             // 填充汇总数据结构
//             summary.group1 = *group1;
//             summary.group2 = *group2;
//             summary.group3 = *group3;
//
//             // 将数据拷贝到发送缓冲区
//             txLength = sizeof(summary);
//             memcpy(txBuffer, &summary, txLength);
//
//             // 使用 HAL_UART_Transmit 直接发送数据
//             if (HAL_UART_Transmit(&huart2, txBuffer, txLength, 1000) == HAL_OK) {
//                 printf("UART send success\r\n");
//             } else {
//                 printf("UART send failed\r\n");
//             }
//         } else {
//             printf("Error: Invalid sensor data pointers!\r\n");
//         }
//         vTaskDelay(10);
//     }
// }

 void StartUsartTransTask(void *argument) {
     SensorDataSummary_t summary;
     UBaseType_t stackHighWaterMark;
     bsp_uartdma_tx_init(&g_UART2_TxDmaBuffer, &huart2, g_txBuffer, 2048);

     for (;;) {
         // 等待所有三个事件位就绪
         xEventGroupWaitBits(xSensorEventGroup,
             SENSOR_GROUP1_BIT | SENSOR_GROUP2_BIT | SENSOR_GROUP3_BIT,
             pdTRUE, pdTRUE, portMAX_DELAY);

         SPI_SensorsGroup *group1, *group2, *group3;


         // 读取最新数据（队列长度1，直接读取）
         xQueueReceive(sensorGroupQueue01Handle, &group1, portMAX_DELAY);
         xQueueReceive(sensorGroupQueue02Handle, &group2, portMAX_DELAY);
         xQueueReceive(sensorGroupQueue03Handle, &group3, portMAX_DELAY);

         // 使用 memcpy 直接拷贝数据
         memcpy(&MagData[0],  group1->mpuSensor1.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[3],  group1->mpuSensor2.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[6],  group1->mpuSensor3.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[9],  group1->mpuSensor4.mpu_value.Mag, 3 * sizeof(float));

         memcpy(&MagData[12], group2->mpuSensor1.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[15], group2->mpuSensor2.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[18], group2->mpuSensor3.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[21], group2->mpuSensor4.mpu_value.Mag, 3 * sizeof(float));

         memcpy(&MagData[24], group3->mpuSensor1.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[27], group3->mpuSensor2.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[30], group3->mpuSensor3.mpu_value.Mag, 3 * sizeof(float));
         memcpy(&MagData[33], group3->mpuSensor4.mpu_value.Mag, 3 * sizeof(float));


         //snprintf(buffer, sizeof(buffer), "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t\r\n",
         //       MagData[7], MagData[8], MagData[9], MagData[10], MagData[11], MagData[12], MagData[13], MagData[14], MagData[15], MagData[16], MagData[17], MagData[18], MagData[19], MagData[20], MagData[21], MagData[22], MagData[23], MagData[24], MagData[25], MagData[26], MagData[27], MagData[28], MagData[29], MagData[30], MagData[31], MagData[32], MagData[33], MagData[34], MagData[35]);

         // 发送数据
         bsp_uartdma_tx_write(&g_UART2_TxDmaBuffer, (uint8_t*)MagData, sizeof(MagData));
         //bsp_uartdma_tx_write(&g_UART2_TxDmaBuffer, (uint8_t*)buffer, strlen(buffer));
         //清空buffer
         //memset(buffer, 0, sizeof(buffer));
         //stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
         //printf("Stack High Water Mark: %lu\r\n", stackHighWaterMark);
     }
}