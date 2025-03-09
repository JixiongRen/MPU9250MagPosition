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

float MagData[36] = {0.0};
char buffer[256] = {0};
uint8_t g_txBuffer[2048] = {0};

// Define the structure of the sensor data summary
typedef struct {
    SPI_SensorsGroup group1;
    SPI_SensorsGroup group2;
    SPI_SensorsGroup group3;
} SensorDataSummary_t;

/**
 * @brief Function implementing the rSpi01MagTask thread.
 * @param argument Not used
 */
void StartrSpi01MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi01MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask01Handle, portMAX_DELAY) == pdTRUE) {
            SensorGroup_ReadMag(spi_sensorsgroup_1);
        }
        if (xQueueOverwrite(sensorGroupQueue01Handle, &spi_sensorsgroup_1) == pdPASS) {
            xEventGroupSetBits(xSensorEventGroup, SENSOR_GROUP1_BIT);
        } else {
        }
    }
}

/**
 * @brief Function implementing the rSpi02MagTask thread.
 * @param argument Not used
 */
void StartrSpi02MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi02MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask02Handle, portMAX_DELAY) == pdTRUE) {
            SensorGroup_ReadMag(spi_sensorsgroup_2);
        }
        if (xQueueOverwrite(sensorGroupQueue02Handle, &spi_sensorsgroup_2) == pdPASS) {
            xEventGroupSetBits(xSensorEventGroup, SENSOR_GROUP2_BIT);
        } else {
        }
    }
}

/**
 * @brief Function implementing the rSpi03MagTask thread.
 * @param argument Not used
 */
void StartrSpi03MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi02MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask03Handle, portMAX_DELAY) == pdTRUE) {
            SensorGroup_ReadMag(spi_sensorsgroup_3);
        }
        if (xQueueOverwrite(sensorGroupQueue03Handle, &spi_sensorsgroup_3) == pdPASS) {
            xEventGroupSetBits(xSensorEventGroup, SENSOR_GROUP3_BIT);
        } else {
        }
    }
}

/**
 * @brief Function implementing the UsartTransTask thread.
 * @param argument Not used
 */
void StartUsartTransTask(void *argument) {
   SensorDataSummary_t summary;
   UBaseType_t stackHighWaterMark;
   uartdma_tx_init(&g_UART2_TxDmaBuffer, &huart2, g_txBuffer, 2048);

   for (;;) {
       // Wait for all sensor data to be ready
       xEventGroupWaitBits(xSensorEventGroup,
           SENSOR_GROUP1_BIT | SENSOR_GROUP2_BIT | SENSOR_GROUP3_BIT,
           pdTRUE, pdTRUE, portMAX_DELAY);

       SPI_SensorsGroup *group1, *group2, *group3;


       // Receive data from queue
       xQueueReceive(sensorGroupQueue01Handle, &group1, portMAX_DELAY);
       xQueueReceive(sensorGroupQueue02Handle, &group2, portMAX_DELAY);
       xQueueReceive(sensorGroupQueue03Handle, &group3, portMAX_DELAY);

       // Copy the data to the summary structure
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

       // Send data via UART with DMA
       uartdma_tx_write(&g_UART2_TxDmaBuffer, (uint8_t*)MagData, sizeof(MagData));
   }
}