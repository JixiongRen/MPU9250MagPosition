//
// Created by renji on 25-2-20.
//
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOSVariables.h"


void StartrSpi01MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi01MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask01Handle, portMAX_DELAY) == pdTRUE) {
            // Simulate the time required to complete the SPI transaction
            SensorGroup_ReadMag(spi_sensorsgroup_1);
        }
    }
}


void StartrSpi02MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi02MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask02Handle, portMAX_DELAY) == pdTRUE) {
            // Simulate the time required to complete the SPI transaction
            SensorGroup_ReadMag(spi_sensorsgroup_2);
        }
    }
}

void StartrSpi03MagTask(void *argument) {
    /* USER CODE BEGIN StartrSpi02MagTask */
    /* Infinite loop */
    for(;;)
    {
        if (xSemaphoreTake(samplingStartTask03Handle, portMAX_DELAY) == pdTRUE) {
            // Simulate the time required to complete the SPI transaction
            SensorGroup_ReadMag(spi_sensorsgroup_3);
        }
    }
}