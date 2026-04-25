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
#include "frameProtocol.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

uint8_t g_txBuffer[2048] = {0};
uint8_t g_frameBuffer[512] = {0};
float MagData[36] = {0};

enum {
    SENSOR_COUNT = 12,
    MAG_FLOAT_COUNT = 36,
    MAG_AUTO_ZERO_SAMPLE_COUNT = 150,
    DIAG_PAYLOAD_LEN = (MAG_FLOAT_COUNT * (int) sizeof(float))
            + (3 * (int) sizeof(uint32_t))
            + (13 * SENSOR_COUNT * (int) sizeof(uint8_t))
            + (2 * SENSOR_COUNT * (int) sizeof(uint32_t))
};

static uint8_t g_payloadBuffer[DIAG_PAYLOAD_LEN] = {0};
static uint32_t g_group_sample_count[3] = {0};

typedef struct {
    uint8_t state;
    uint16_t collected_frames;
    float accum[MAG_FLOAT_COUNT];
    float offset[MAG_FLOAT_COUNT];
} MagAutoZeroState;

enum {
    MAG_AUTO_ZERO_WAIT_VALID = 0,
    MAG_AUTO_ZERO_COLLECTING = 1,
    MAG_AUTO_ZERO_READY = 2,
};

static MagAutoZeroState g_magAutoZero = {0};

static MPU9250 *GetSensorByIndex(SPI_SensorsGroup *group1,
                                 SPI_SensorsGroup *group2,
                                 SPI_SensorsGroup *group3,
                                 uint8_t sensor_index) {
    if (sensor_index < 4U) {
        switch (sensor_index) {
            case 0: return &group1->mpuSensor1;
            case 1: return &group1->mpuSensor2;
            case 2: return &group1->mpuSensor3;
            default: return &group1->mpuSensor4;
        }
    }

    if (sensor_index < 8U) {
        switch (sensor_index - 4U) {
            case 0: return &group2->mpuSensor1;
            case 1: return &group2->mpuSensor2;
            case 2: return &group2->mpuSensor3;
            default: return &group2->mpuSensor4;
        }
    }

    switch (sensor_index - 8U) {
        case 0: return &group3->mpuSensor1;
        case 1: return &group3->mpuSensor2;
        case 2: return &group3->mpuSensor3;
        default: return &group3->mpuSensor4;
    }
}

static void LoadSensorArray(SPI_SensorsGroup *group1,
                             SPI_SensorsGroup *group2,
                             SPI_SensorsGroup *group3,
                             MPU9250 *sensors[SENSOR_COUNT]) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        sensors[i] = GetSensorByIndex(group1, group2, group3, i);
    }
}

static uint8_t SensorHasValidMagOutput(const MPU9250 *sensor) {
    return (sensor != NULL
            && sensor->diag_init_error == 0U
            && sensor->diag_last_mag_status == 0U
            && sensor->diag_mag_read_success > 0U) ? 1U : 0U;
}

static void MagAutoZeroReset(uint8_t next_state) {
    g_magAutoZero.state = next_state;
    g_magAutoZero.collected_frames = 0U;
    memset(g_magAutoZero.accum, 0, sizeof(g_magAutoZero.accum));
}

static void UpdateAppLedStateFromAutoZero(void) {
    if (g_magAutoZero.state == MAG_AUTO_ZERO_READY) {
        AppLed_SetState(APP_LED_STATE_READY_ON);
        return;
    }

    if (g_magAutoZero.state == MAG_AUTO_ZERO_COLLECTING) {
        AppLed_SetState(APP_LED_STATE_AUTO_ZERO_BLINK);
        return;
    }

    AppLed_SetState(APP_LED_STATE_INIT_BLINK);
}

static void MagAutoZeroUpdate(MPU9250 *sensors[SENSOR_COUNT]) {
    for (uint8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {
        if (SensorHasValidMagOutput(sensors[sensor_index]) == 0U) {
            if (g_magAutoZero.state != MAG_AUTO_ZERO_READY) {
                MagAutoZeroReset(MAG_AUTO_ZERO_WAIT_VALID);
                UpdateAppLedStateFromAutoZero();
            }
            return;
        }
    }

    if (g_magAutoZero.state == MAG_AUTO_ZERO_READY) {
        UpdateAppLedStateFromAutoZero();
        return;
    }

    if (g_magAutoZero.state == MAG_AUTO_ZERO_WAIT_VALID) {
        MagAutoZeroReset(MAG_AUTO_ZERO_COLLECTING);
        UpdateAppLedStateFromAutoZero();
    }

    for (uint8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {
        for (uint8_t axis = 0; axis < 3U; axis++) {
            uint8_t channel_index = (uint8_t) (sensor_index * 3U + axis);
            g_magAutoZero.accum[channel_index] += sensors[sensor_index]->mpu_value.Mag[axis];
        }
    }

    g_magAutoZero.collected_frames++;
    if (g_magAutoZero.collected_frames < MAG_AUTO_ZERO_SAMPLE_COUNT) {
        return;
    }

    for (uint8_t channel_index = 0; channel_index < MAG_FLOAT_COUNT; channel_index++) {
        g_magAutoZero.offset[channel_index] = g_magAutoZero.accum[channel_index] / (float) g_magAutoZero.collected_frames;
    }
    g_magAutoZero.state = MAG_AUTO_ZERO_READY;
    UpdateAppLedStateFromAutoZero();
}

static float GetOutputMagValue(const MPU9250 *sensor, uint8_t sensor_index, uint8_t axis) {
    uint8_t channel_index = (uint8_t) (sensor_index * 3U + axis);
    float value = sensor->mpu_value.Mag[axis];
    if (g_magAutoZero.state == MAG_AUTO_ZERO_READY) {
        value -= g_magAutoZero.offset[channel_index];
    }
    return value;
}

static uint16_t BuildTelemetryPayload(SPI_SensorsGroup *group1,
                                      SPI_SensorsGroup *group2,
                                      SPI_SensorsGroup *group3,
                                      uint8_t *payload) {
    size_t offset = 0;
    MPU9250 *sensors[SENSOR_COUNT];

    LoadSensorArray(group1, group2, group3, sensors);

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        for (uint8_t axis = 0; axis < 3U; axis++) {
            MagData[i * 3U + axis] = GetOutputMagValue(sensors[i], i, axis);
        }
        memcpy(&payload[offset], &MagData[i * 3U], 3U * sizeof(float));
        offset += 3U * sizeof(float);
    }

    memcpy(&payload[offset], g_group_sample_count, sizeof(g_group_sample_count));
    offset += sizeof(g_group_sample_count);

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_mpu_whoami;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_ak8963_whoami;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_init_error;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_mag_status;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_i2c_mst_status;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_aux_data;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_user_ctrl;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_i2c_mst_ctrl;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_aux_op;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_slv4_ctrl;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_slv4_addr;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_slv4_reg;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        payload[offset++] = sensors[i]->diag_last_slv4_do;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        memcpy(&payload[offset], &sensors[i]->diag_mag_read_success, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        memcpy(&payload[offset], &sensors[i]->diag_mag_read_attempts, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }

    return (uint16_t) offset;
}

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
   uartdma_tx_init(&g_UART2_TxDmaBuffer, &huart2, g_txBuffer, 2048);
   HAL_TIM_Base_Start_IT(&htim1);

   for (;;) {
       // Wait for all sensor data to be ready
       xEventGroupWaitBits(xSensorEventGroup,
           SENSOR_GROUP1_BIT | SENSOR_GROUP2_BIT | SENSOR_GROUP3_BIT,
           pdTRUE, pdTRUE, portMAX_DELAY);

       SPI_SensorsGroup *group1, *group2, *group3;
       MPU9250 *sensors[SENSOR_COUNT];

       // Receive data from queue
       xQueueReceive(sensorGroupQueue01Handle, &group1, portMAX_DELAY);
       xQueueReceive(sensorGroupQueue02Handle, &group2, portMAX_DELAY);
       xQueueReceive(sensorGroupQueue03Handle, &group3, portMAX_DELAY);

       LoadSensorArray(group1, group2, group3, sensors);
       MagAutoZeroUpdate(sensors);

       if (g_magAutoZero.state != MAG_AUTO_ZERO_READY) {
           continue;
       }

       uint16_t payload_len = BuildTelemetryPayload(group1, group2, group3, g_payloadBuffer);

       // Build frame with header and CRC
       uint16_t frame_len = buildFrame(g_payloadBuffer, payload_len, g_frameBuffer);

       // Send data via UART with DMA
       while (!uartdma_tx_write(&g_UART2_TxDmaBuffer, g_frameBuffer, frame_len)) {
           osDelay(1);
       }
   }
}
