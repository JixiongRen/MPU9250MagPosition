//
// Created by renji on 25-1-31.
//

#include "main.h"
#include "MPU9250.h"
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "magCalibration.h"

#define MPU9250_RESET_DELAY_MS 100U
#define MPU9250_I2C_POLL_DELAY_US 100U
#define MPU9250_I2C_POLL_RETRIES 50U
#define AK8963_MODE_CHANGE_DELAY_MS 20U
#define AK8963_FIRST_SAMPLE_RETRIES 20U
#define AK8963_FIRST_SAMPLE_DELAY_MS 2U
#define AK8963_SHADOW_READ_LEN 8U
#define AK8963_SHADOW_READ_RETRIES 3U
#define AK8963_SHADOW_READ_DELAY_MS 1U
#define MPU9250_I2C_MST_CTRL_EXPERIMENT (MPU9250_I2C_MST_CLK_400KHZ | 0x10U | 0x40U)

#define MPU9250_DIAG_OK                0U
#define MPU9250_DIAG_ERR_MPU_WHOAMI    1U
#define MPU9250_DIAG_ERR_AK_WHOAMI     2U
#define MPU9250_DIAG_ERR_FIRST_SAMPLE  3U
#define MPU9250_DIAG_ERR_MAG_EMPTY     4U
#define MPU9250_DIAG_ERR_MAG_OVERFLOW  5U

static void MPU9250_DelayMs(uint32_t delay_ms) {
    if (osKernelGetState() == osKernelRunning) {
        osDelay(delay_ms);
    } else {
        HAL_Delay(delay_ms);
    }
}

static void MPU9250_DelayUs(uint32_t delay_us) {
    static uint8_t dwt_initialized = 0;

    if (dwt_initialized == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        dwt_initialized = 1;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * delay_us;
    while ((DWT->CYCCNT - start) < cycles) { }
}

static MPU9250 *SensorGroup_GetSensor(SPI_SensorsGroup *spi_sensorsgroup, uint8_t index) {
    switch (index) {
        case 0:
            return &(spi_sensorsgroup->mpuSensor1);
        case 1:
            return &(spi_sensorsgroup->mpuSensor2);
        case 2:
            return &(spi_sensorsgroup->mpuSensor3);
        case 3:
            return &(spi_sensorsgroup->mpuSensor4);
        default:
            return NULL;
    }
}

static void SensorGroup_DeselectAll(SPI_SensorsGroup *spi_sensorsgroup) {
    for (uint8_t i = 0; i < spi_sensorsgroup->mpuSensorNum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu != NULL) {
            MPU9250_DISENABLE(mpu);
        }
    }
}

static float MPU9250_GetMagScale(const MPU9250 *mpu, uint8_t axis) {
    uint8_t adjust = mpu->mag_adjust[axis];
    if (adjust == 0x00 || adjust == 0xFF) {
        adjust = 128;
    }
    return ((((float) adjust) - 128.0f) / 256.0f + 1.0f) * 0.15f;
}

static float MPU9250_Median3(float a, float b, float c) {
    if (a > b) {
        float temp = a;
        a = b;
        b = temp;
    }
    if (b > c) {
        float temp = b;
        b = c;
        c = temp;
    }
    if (a > b) {
        float temp = a;
        a = b;
        b = temp;
    }
    return b;
}

static void MPU9250_ApplyMagDespike(MPU9250 *mpu, float *mag) {
    if (mpu->mag_despike_count < 2U) {
        memcpy(mpu->mag_despike_history[mpu->mag_despike_count], mag, 3U * sizeof(float));
        mpu->mag_despike_count++;
        return;
    }

    for (uint8_t axis = 0; axis < 3U; axis++) {
        float current = mag[axis];
        mag[axis] = MPU9250_Median3(
            mpu->mag_despike_history[0][axis],
            mpu->mag_despike_history[1][axis],
            current
        );
        mpu->mag_despike_history[0][axis] = mpu->mag_despike_history[1][axis];
        mpu->mag_despike_history[1][axis] = current;
    }
}

static uint8_t MPU9250_IsWhoAmIValid(uint8_t whoami) {
    return (whoami == WHO_AM_I_ANS || whoami == WHO_AM_I_ANS_ALT) ? 1U : 0U;
}

static void MPU9250_ResetAuxI2CMaster(MPU9250 *mpu) {
    mpu_w_reg(USER_CTRL, USER_CTRL_I2C_IF_DIS | USER_CTRL_I2C_MST_EN | USER_CTRL_I2C_MST_RST, mpu);
    MPU9250_DelayMs(1U);
    mpu_w_reg(USER_CTRL, USER_CTRL_I2C_IF_DIS | USER_CTRL_I2C_MST_EN, mpu);
}

static void MPU9250_UpdateAuxConfigDiag(MPU9250 *mpu) {
    mpu_r_reg(USER_CTRL, 1, mpu, &mpu->diag_user_ctrl);
    mpu_r_reg(I2C_MST_CTRL, 1, mpu, &mpu->diag_i2c_mst_ctrl);
    mpu_r_reg(I2C_SLV4_CTRL, 1, mpu, &mpu->diag_last_slv4_ctrl);
    mpu_r_reg(I2C_SLV4_ADDR, 1, mpu, &mpu->diag_last_slv4_addr);
    mpu_r_reg(I2C_SLV4_REG, 1, mpu, &mpu->diag_last_slv4_reg);
    mpu_r_reg(I2C_SLV4_DO, 1, mpu, &mpu->diag_last_slv4_do);
}

static uint8_t MPU9250_WaitI2CSlave4Done(MPU9250 *mpu) {
    uint8_t status = 0;

    for (uint8_t retry = 0; retry < MPU9250_I2C_POLL_RETRIES; retry++) {
        mpu_r_reg(I2C_MST_STATUS, 1, mpu, &status);
        mpu->diag_last_i2c_mst_status = status;
        if ((status & I2C_MST_STATUS_SLV4_DONE) != 0U) {
            return ((status & I2C_MST_STATUS_SLV4_NACK) == 0U) ? 1U : 0U;
        }
        MPU9250_DelayUs(MPU9250_I2C_POLL_DELAY_US);
    }

    return 0;
}

static void MPU9250_DisableMagShadowRead(MPU9250 *mpu) {
    mpu_w_reg(I2C_SLV0_CTRL, 0x00, mpu);
}

static void MPU9250_EnableMagShadowRead(MPU9250 *mpu) {
    mpu_w_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80U, mpu);
    mpu_w_reg(I2C_SLV0_REG, AK8963_ST1_REG, mpu);
    mpu_w_reg(I2C_SLV0_CTRL, I2C_SLV0_EN | AK8963_SHADOW_READ_LEN, mpu);
    mpu_w_reg(I2C_MST_DELAY_CTRL, I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW, mpu);
}

static uint8_t MPU9250_BufferHasValidMagData(const uint8_t *buffer, uint8_t require_drdy) {
    if ((buffer[AK8963_SHADOW_READ_LEN - 1U] & AK8963_ST2_HOFL) != 0U) {
        return 0;
    }

    if (require_drdy != 0U && (buffer[0] & AK8963_ST1_DRDY) == 0U) {
        return 0;
    }

    if ((buffer[1] | buffer[2] | buffer[3] | buffer[4] | buffer[5] | buffer[6]) == 0U) {
        return 0;
    }

    return 1;
}

static uint8_t MPU9250_ReadMagRegisters(MPU9250 *mpu, uint8_t *buffer, uint8_t require_drdy) {
    for (uint8_t retry = 0; retry < AK8963_SHADOW_READ_RETRIES; retry++) {
        if (ak8963_r_reg(AK8963_ST1_REG, AK8963_SHADOW_READ_LEN, mpu, buffer) != 0U) {
            mpu->diag_last_aux_data = buffer[0];
            if (MPU9250_BufferHasValidMagData(buffer, require_drdy) != 0U) {
                return 1U;
            }
            if ((buffer[AK8963_SHADOW_READ_LEN - 1U] & AK8963_ST2_HOFL) != 0U) {
                mpu->diag_last_mag_status = MPU9250_DIAG_ERR_MAG_OVERFLOW;
            }
        }
        MPU9250_DelayMs(AK8963_SHADOW_READ_DELAY_MS);
    }

    memset(buffer, 0, AK8963_SHADOW_READ_LEN);
    return 0U;
}

static uint8_t MPU9250_WaitFirstMagSample(MPU9250 *mpu) {
    uint8_t mag_shadow[AK8963_SHADOW_READ_LEN] = { 0 };

    for (uint8_t retry = 0; retry < AK8963_FIRST_SAMPLE_RETRIES; retry++) {
        if (MPU9250_ReadMagRegisters(mpu, mag_shadow, 1U) != 0U) {
            return 1;
        }
        MPU9250_DelayMs(AK8963_FIRST_SAMPLE_DELAY_MS);
    }

    return 0;
}

static uint8_t MPU9250_Slv4WriteByte(MPU9250 *mpu, uint8_t reg, uint8_t byte) {
    uint8_t status = 0;

    mpu->diag_last_aux_op = 0x11U;
    mpu_r_reg(I2C_MST_STATUS, 1, mpu, &status);
    mpu->diag_last_i2c_mst_status = status;

    mpu_w_reg(I2C_SLV4_ADDR, AK8963_I2C_ADDR, mpu);
    mpu_w_reg(I2C_SLV4_REG, reg, mpu);
    mpu_w_reg(I2C_SLV4_DO, byte, mpu);
    mpu_w_reg(I2C_SLV4_CTRL, I2C_SLV4_EN, mpu);
    mpu_r_reg(I2C_SLV4_ADDR, 1, mpu, &mpu->diag_last_slv4_addr);
    mpu_r_reg(I2C_SLV4_REG, 1, mpu, &mpu->diag_last_slv4_reg);
    mpu_r_reg(I2C_SLV4_DO, 1, mpu, &mpu->diag_last_slv4_do);
    mpu_r_reg(I2C_SLV4_CTRL, 1, mpu, &mpu->diag_last_slv4_ctrl);

    if (MPU9250_WaitI2CSlave4Done(mpu) == 0U) {
        mpu->diag_last_aux_op = 0x1FU;
        mpu_w_reg(I2C_SLV4_CTRL, 0x00U, mpu);
        return 0U;
    }

    mpu->diag_last_aux_op = 0x12U;
    mpu_w_reg(I2C_SLV4_CTRL, 0x00U, mpu);
    return 1U;
}

static uint8_t MPU9250_Slv4ReadByte(MPU9250 *mpu, uint8_t reg, uint8_t *byte) {
    uint8_t status = 0;
    uint8_t data = 0;

    mpu->diag_last_aux_op = 0x21U;
    mpu_r_reg(I2C_MST_STATUS, 1, mpu, &status);
    mpu->diag_last_i2c_mst_status = status;

    mpu_w_reg(I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80U, mpu);
    mpu_w_reg(I2C_SLV4_REG, reg, mpu);
    mpu_w_reg(I2C_SLV4_CTRL, I2C_SLV4_EN, mpu);
    mpu_r_reg(I2C_SLV4_ADDR, 1, mpu, &mpu->diag_last_slv4_addr);
    mpu_r_reg(I2C_SLV4_REG, 1, mpu, &mpu->diag_last_slv4_reg);
    mpu_r_reg(I2C_SLV4_DO, 1, mpu, &mpu->diag_last_slv4_do);
    mpu_r_reg(I2C_SLV4_CTRL, 1, mpu, &mpu->diag_last_slv4_ctrl);

    if (MPU9250_WaitI2CSlave4Done(mpu) == 0U) {
        mpu->diag_last_aux_op = 0x2FU;
        mpu_w_reg(I2C_SLV4_CTRL, 0x00U, mpu);
        *byte = 0U;
        return 0U;
    }

    mpu_r_reg(I2C_SLV4_DI, 1, mpu, &data);
    mpu->diag_last_aux_data = data;
    *byte = data;

    mpu->diag_last_aux_op = 0x22U;
    mpu_w_reg(I2C_SLV4_CTRL, 0x00U, mpu);
    return 1U;
}

/**
 * @brief Enable MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ENABLE(MPU9250 *mpu) {
    HAL_GPIO_WritePin(mpu->mpu9250_cfg.CS_Port, mpu->mpu9250_cfg.CS_Pin, GPIO_PIN_RESET);
}


/**
 * @brief Disable MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_DISENABLE(MPU9250 *mpu) {
    HAL_GPIO_WritePin(mpu->mpu9250_cfg.CS_Port, mpu->mpu9250_cfg.CS_Pin, GPIO_PIN_SET);
}


/**
 * @brief Send a byte using SPI
 * @param hspix SPI handle
 * @param byte Byte to be sent
 * @return Returns the received byte
 */
uint8_t spi_w_byte(SPI_HandleTypeDef *hspix, uint8_t byte) {
    uint8_t feedback = 0;
    while (HAL_SPI_GetState(hspix) != HAL_SPI_STATE_READY) { }
    if (HAL_SPI_TransmitReceive(hspix, &byte, &feedback, 1, 0x01f4) != HAL_OK) {
        return 0xff;
    }
    return feedback;
}

/**
 * @brief Write multiple bytes using SPI
 * @param reg Target register address to write
 * @param mpu Pointer to MPU9250 structure
 * @param bytes Multiple bytes to be written
 * @param num Number of bytes
 */
void spi_w_bytes(uint8_t reg, MPU9250 *mpu, uint8_t *bytes, uint16_t num)
{
    SPI_HandleTypeDef *hspix = mpu->mpu9250_cfg.hspix;

    MPU9250_ENABLE(mpu);
    spi_w_byte(hspix, reg);
    for (uint16_t index = 0; index < num; index++) {
        spi_w_byte(hspix, bytes[index]);
    }
    MPU9250_DISENABLE(mpu);
}


/**
 * @brief Read multiple bytes using SPI
 * @param reg Target register address to read
 * @param mpu Pointer to MPU9250 structure
 * @param buffer Buffer to store the read data
 * @param num Number of bytes to read using SPI
 */
void spi_r_bytes(uint8_t reg, MPU9250 *mpu, uint8_t *buffer, uint8_t num)
{
    SPI_HandleTypeDef *hspix = mpu->mpu9250_cfg.hspix;
    reg |= 0x80;
    MPU9250_ENABLE(mpu);

    if (HAL_SPI_Transmit(hspix, &reg, 1, 0x01f4) != HAL_OK) {
        memset(buffer, 0, num);
    } else if (HAL_SPI_Receive(hspix, buffer, num, 0x01f4) != HAL_OK) {
        memset(buffer, 0, num);
    }

    MPU9250_DISENABLE(mpu);
}


/**
 * @brief Write data to MPU9250 register
 * @param reg Target register address
 * @param byte Data to be written
 * @param mpu Pointer to MPU9250 structure
 */
void mpu_w_reg(uint8_t reg, uint8_t byte, MPU9250 *mpu) {
    spi_w_bytes(reg, mpu, &byte, 1);
}


/**
 * @brief Read data from MPU9250 register
 * @param reg Target register address
 * @param num Number of bytes to read
 * @param mpu Pointer to MPU9250 structure
 * @param buffer Buffer to store the read data
 */
void mpu_r_reg(uint8_t reg, uint8_t num, MPU9250 *mpu, uint8_t *buffer) {
    spi_r_bytes(reg, mpu, buffer, num);
}


/**
 * @brief Write data to AK8963 register
 * @param reg Target register address
 * @param byte Data to be written
 * @param mpu Pointer to MPU9250 structure
 */
uint8_t ak8963_w_reg(uint8_t reg, uint8_t byte, MPU9250 *mpu) {
    mpu->diag_last_aux_op = 0x14U;
    mpu_w_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR, mpu);
    mpu_w_reg(I2C_SLV0_REG, reg, mpu);
    mpu_w_reg(I2C_SLV0_DO, byte, mpu);
    mpu_w_reg(I2C_SLV0_CTRL, 0x87U, mpu);
    MPU9250_DelayMs(1U);
    mpu_r_reg(I2C_MST_STATUS, 1, mpu, &mpu->diag_last_i2c_mst_status);
    return ((mpu->diag_last_i2c_mst_status & I2C_MST_STATUS_SLV0_NACK) == 0U) ? 1U : 0U;
}


/**
 * @brief Read data from AK8963 register
 * @param reg Register address
 * @param num Number of bytes to read
 * @param mpu Pointer to MPU9250 structure
 * @param buffer Buffer to store the read data
 */
uint8_t ak8963_r_reg(uint8_t reg, uint8_t num, MPU9250 *mpu, uint8_t *buffer) {
    if (num == 0U) {
        return 0U;
    }

    if (num == 1U) {
        return MPU9250_Slv4ReadByte(mpu, reg, buffer);
    }

    mpu->diag_last_aux_op = 0x31U;
    mpu_w_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80U, mpu);
    mpu_w_reg(I2C_SLV0_REG, reg, mpu);
    mpu_w_reg(I2C_SLV0_CTRL, I2C_SLV0_EN | (num & 0x0FU), mpu);
    memset(buffer, 0, num);

    for (uint8_t retry = 0; retry < 5U; retry++) {
        MPU9250_DelayMs(1U);
        mpu_r_reg(EXT_SENS_DATA_00, num, mpu, buffer);
        mpu_r_reg(I2C_MST_STATUS, 1, mpu, &mpu->diag_last_i2c_mst_status);
        mpu->diag_last_aux_data = buffer[0];

        if ((mpu->diag_last_i2c_mst_status & I2C_MST_STATUS_SLV0_NACK) != 0U) {
            break;
        }

        for (uint8_t index = 0; index < num; index++) {
            if (buffer[index] != 0U) {
                mpu_w_reg(I2C_SLV0_CTRL, 0x00U, mpu);
                mpu->diag_last_aux_op = 0x32U;
                return 1U;
            }
        }
    }

    mpu_w_reg(I2C_SLV0_CTRL, 0x00U, mpu);

    if ((mpu->diag_last_i2c_mst_status & I2C_MST_STATUS_SLV0_NACK) != 0U) {
        mpu->diag_last_aux_op = 0x3FU;
        memset(buffer, 0, num);
        return 0U;
    }

    mpu->diag_last_aux_op = 0x33U;
    memset(buffer, 0, num);
    return 0U;
}

/**
 * @brief Confirm the device ID of MPU9250
 * @param mpu Pointer to MPU9250 structure
 * @return uint8_t Returns the value of the WHO_AM_I register of MPU9250
 */
uint8_t mpu_r_WhoAmI(MPU9250 *mpu) {
    uint8_t buffer[1] = { 0 };
    mpu_r_reg(WHO_AM_I, 1, mpu, buffer);
    return buffer[0];
}


/**
 * @brief Confirm the device ID of AK8963
 * @param mpu Pointer to MPU9250 structure
 * @return uint8_t Returns the value of the WHO_AM_I register of AK8963
 */
uint8_t mpu_r_ak8963_WhoAmI(MPU9250 *mpu) {
    uint8_t buffer[2] = { 0 };
    (void) ak8963_r_reg(AK8963_WHOAMI_REG, 2, mpu, buffer);
    return buffer[0];
}

/**
 * @brief Initialize MPU9250 structure
 * @param mpu Pointer to MPU9250 structure
 * @param hspix SPI handle
 * @param cs_port GPIO port number connected to MPU chip select pin
 * @param cs_pin GPIO pin number connected to MPU chip select pin
 */
void MPU9250_StructInit(MPU9250 *mpu, SPI_HandleTypeDef *hspix, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    mpu->mpu9250_cfg.hspix = hspix;
    mpu->mpu9250_cfg.CS_Port = cs_port;
    mpu->mpu9250_cfg.CS_Pin = cs_pin;

    for (uint8_t i = 0; i < 3; i++) {
        mpu->mpu_value.Accel[i] = 0.0f;
        mpu->mpu_value.Gyro[i] = 0.0f;
        mpu->mpu_value.Mag[i] = 0.0f;

        mpu->mpu_value.Accel_row[i] = 0;
        mpu->mpu_value.Gyro_row[i] = 0;
        mpu->mpu_value.Mag_row[i] = 0;
        mpu->mag_adjust[i] = 128;
        mpu->mag_despike_history[0][i] = 0.0f;
        mpu->mag_despike_history[1][i] = 0.0f;
    }

    mpu->mag_despike_count = 0U;

    mpu->diag_mpu_whoami = 0U;
    mpu->diag_ak8963_whoami = 0U;
    mpu->diag_init_error = MPU9250_DIAG_ERR_FIRST_SAMPLE;
    mpu->diag_last_mag_status = MPU9250_DIAG_ERR_MAG_EMPTY;
    mpu->diag_last_i2c_mst_status = 0U;
    mpu->diag_last_aux_data = 0U;
    mpu->diag_user_ctrl = 0U;
    mpu->diag_i2c_mst_ctrl = 0U;
    mpu->diag_last_aux_op = 0U;
    mpu->diag_last_slv4_ctrl = 0U;
    mpu->diag_last_slv4_addr = 0U;
    mpu->diag_last_slv4_reg = 0U;
    mpu->diag_last_slv4_do = 0U;
    mpu->diag_mag_read_attempts = 0U;
    mpu->diag_mag_read_success = 0U;

    MPU9250_DISENABLE(mpu);
}

/**
 * @brief Initialize SPI_SensorsGroup structure
 * @param spi_sensorsgroup Pointer to SPI_SensorsGroup structure
 * @param sensornum Number of MPU9250 sensors in the group
 * @param ghspix Array of SPI handles for each MPU9250 sensor
 * @param gcs_port Array of GPIO ports for each MPU9250 sensor's chip select pin
 * @param gcs_pin Array of GPIO pins for each MPU9250 sensor's chip select pin
 */
void SensorGroup_StructInit(SPI_SensorsGroup* spi_sensorsgroup, uint8_t sensornum, SPI_HandleTypeDef *ghspix[], GPIO_TypeDef *gcs_port[], uint16_t gcs_pin[]) {
    spi_sensorsgroup->mpuSensorNum = sensornum;
    spi_sensorsgroup->base_sensor_id = 0;

    for (uint8_t i = 0; i < sensornum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu == NULL) {
            return;
        }
        MPU9250_StructInit(mpu, ghspix[i], gcs_port[i], gcs_pin[i]);
    }
}

/**
 * @brief Initialize MPU9250
 * @param mpu Pointer to MPU9250 structure
 * @return uint8_t Returns 0x00 indicating successful initialization
 */
uint8_t MPU9250_Init(MPU9250 *mpu) {
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        uint8_t buffer[1] = { 0 };
        mpu->diag_init_error = MPU9250_DIAG_ERR_FIRST_SAMPLE;
        mpu->diag_mpu_whoami = 0U;
        mpu->diag_ak8963_whoami = 0U;
        mpu->diag_last_i2c_mst_status = 0U;
        mpu->diag_last_aux_data = 0U;
        mpu->diag_user_ctrl = 0U;
        mpu->diag_i2c_mst_ctrl = 0U;
        mpu->diag_last_aux_op = 0U;
        mpu->diag_last_slv4_ctrl = 0U;
        mpu->diag_last_slv4_addr = 0U;
        mpu->diag_last_slv4_reg = 0U;
        mpu->diag_last_slv4_do = 0U;

        for (uint8_t axis = 0; axis < 3; axis++) {
            mpu->mag_adjust[axis] = 128;
            mpu->mag_despike_history[0][axis] = 0.0f;
            mpu->mag_despike_history[1][axis] = 0.0f;
        }
        mpu->mag_despike_count = 0U;

        MPU9250_DisableMagShadowRead(mpu);
        mpu_w_reg(PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET, mpu);
        MPU9250_DelayMs(10U);
        mpu_w_reg(INT_PIN_CFG, 0x00U, mpu);
        mpu_w_reg(USER_CTRL, USER_CTRL_I2C_IF_DIS | USER_CTRL_I2C_MST_EN, mpu);
        MPU9250_ResetAuxI2CMaster(mpu);
        mpu_w_reg(I2C_MST_CTRL, MPU9250_I2C_MST_CTRL_EXPERIMENT, mpu);
        (void) ak8963_w_reg(AK8963_CNTL1_REG, AK8963_CNTL1_POWER_DOWN, mpu);

        mpu_w_reg(PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET, mpu);
        MPU9250_DelayMs(10U);
        (void) ak8963_w_reg(AK8963_CNTL2_REG, AK8963_CNTL2_SRST, mpu);
        mpu_w_reg(PWR_MGMT_1, MPU9250_PWR1_CLKSEL_PLL, mpu);
        mpu_w_reg(PWR_MGMT_2, (uint8_t) 0x00, mpu);

        mpu_w_reg(SMPLRT_DIV, (uint8_t) 0x00, mpu);
        mpu_w_reg(GYRO_CONFIG, (uint8_t) MPU9250_Gyro_Range_2000dps, mpu);
        mpu_w_reg(ACCEL_CONFIG, (uint8_t) MPU9250_Accel_Range_16G, mpu);
        mpu_w_reg(ACCEL_CONFIG_2, (uint8_t) MPU9250_Accel_DLPFBandwidth_460, mpu);
        mpu_w_reg(CONFIG, (uint8_t) MPU9250_Gyro_DLPFBandwidth_250, mpu);

        mpu_w_reg(INT_PIN_CFG, 0x00U, mpu);
        mpu_w_reg(USER_CTRL, USER_CTRL_I2C_IF_DIS | USER_CTRL_I2C_MST_EN, mpu);
        MPU9250_ResetAuxI2CMaster(mpu);
        mpu_w_reg(I2C_MST_CTRL, MPU9250_I2C_MST_CTRL_EXPERIMENT, mpu);
        MPU9250_UpdateAuxConfigDiag(mpu);

        mpu->diag_mpu_whoami = mpu_r_WhoAmI(mpu);
        if (MPU9250_IsWhoAmIValid(mpu->diag_mpu_whoami) == 0U) {
            mpu->diag_init_error = MPU9250_DIAG_ERR_MPU_WHOAMI;
            continue;
        }

        (void) ak8963_w_reg(AK8963_CNTL1_REG, AK8963_CNTL1_POWER_DOWN, mpu);
        MPU9250_DelayMs(100U);
        mpu->diag_ak8963_whoami = mpu_r_ak8963_WhoAmI(mpu);
        if (mpu->diag_ak8963_whoami != AK8963_WHOAMI_ID) {
            mpu->diag_init_error = MPU9250_DIAG_ERR_AK_WHOAMI;
            continue;
        }

        if (ak8963_w_reg(AK8963_CNTL1_REG, AK8963_CNTL1_FUSE_ROM_ACCESS, mpu) == 0U) {
            mpu->diag_init_error = MPU9250_DIAG_ERR_AK_WHOAMI;
            continue;
        }
        MPU9250_DelayMs(100U);
        if (ak8963_r_reg(AK8963_ASAX, 3, mpu, mpu->mag_adjust) == 0U) {
            mpu->diag_init_error = MPU9250_DIAG_ERR_AK_WHOAMI;
            continue;
        }

        (void) ak8963_w_reg(AK8963_CNTL1_REG, AK8963_CNTL1_POWER_DOWN, mpu);
        MPU9250_DelayMs(100U);
        if (ak8963_w_reg(AK8963_CNTL1_REG, AK8963_CNTL1_CONT_MEAS_2_16B, mpu) == 0U) {
            mpu->diag_init_error = MPU9250_DIAG_ERR_AK_WHOAMI;
            continue;
        }
        MPU9250_DelayMs(100U);

        mpu_w_reg(PWR_MGMT_1, MPU9250_PWR1_CLKSEL_PLL, mpu);
        (void) ak8963_r_reg(MAG_XOUT_L, 1, mpu, buffer);
        (void) ak8963_r_reg(AK8963_ST2_REG, 1, mpu, buffer);
        if ((buffer[0] & AK8963_ST2_HOFL) != 0U) {
            mpu->diag_last_mag_status = MPU9250_DIAG_ERR_MAG_OVERFLOW;
        } else {
            mpu->diag_last_mag_status = MPU9250_DIAG_OK;
        }
        mpu->diag_init_error = MPU9250_DIAG_OK;
        return 0x00;
    }

    return 0x01;
}

/**
 * @brief Initialize all MPU9250 sensors in the SPI_SensorsGroup
 * @param spi_sensorsgroup Pointer to SPI_SensorsGroup structure
 * @return uint8_t Returns 0x00 indicating successful initialization of all sensors
 */
uint8_t SensorGroup_Init(SPI_SensorsGroup* spi_sensorsgroup) {
    uint8_t init_status = 0x00;
    uint8_t sensorNum = spi_sensorsgroup->mpuSensorNum;

    SensorGroup_DeselectAll(spi_sensorsgroup);
    MPU9250_DelayMs(1);

    for (uint8_t i = 0; i < sensorNum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu == NULL) {
            continue;
        }

        SensorGroup_DeselectAll(spi_sensorsgroup);
        if (MPU9250_Init(mpu) != 0x00) {
            init_status = 0x01;
        }
    }

    SensorGroup_DeselectAll(spi_sensorsgroup);
    return init_status;
}

/**
 * @brief Read accelerometer data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ReadAccel(MPU9250 *mpu) {
    uint8_t buffer[6] = { 0 };
    mpu_r_reg(ACCEL_XOUT_H, 6, mpu, buffer);

    mpu->mpu_value.Accel_row[0] = ((int16_t)buffer[0] << 8) | buffer[1];
    mpu->mpu_value.Accel[0] = (float) mpu->mpu_value.Accel_row[0] / 208.980f;

    mpu->mpu_value.Accel_row[1] = ((int16_t)buffer[2] << 8) | buffer[3];
    mpu->mpu_value.Accel[1] = (float) mpu->mpu_value.Accel_row[1] / 208.980f;

    mpu->mpu_value.Accel_row[2] = ((int16_t)buffer[4] << 8) | buffer[5];
    mpu->mpu_value.Accel[2] = (float) mpu->mpu_value.Accel_row[2] / 208.980f;
}

/**
 * @brief Read gyroscope data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ReadGyro(MPU9250 *mpu) {
    uint8_t buffer[6] = { 0 };
    mpu_r_reg(GYRO_XOUT_H, 6, mpu, buffer);

    mpu->mpu_value.Gyro_row[0] = ((int16_t)buffer[0] << 8) | buffer[1];
    mpu->mpu_value.Gyro[0] = mpu->mpu_value.Gyro_row[0] / 16.384f;

    mpu->mpu_value.Gyro_row[1] = ((int16_t)buffer[2] << 8) | buffer[3];
    mpu->mpu_value.Gyro[1] = mpu->mpu_value.Gyro_row[1] / 16.384f;

    mpu->mpu_value.Gyro_row[2] = ((int16_t)buffer[4] << 8) | buffer[5];
    mpu->mpu_value.Gyro[2] = mpu->mpu_value.Gyro_row[2] / 16.384f;
}

/**
 * @brief Read magnetometer data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 * @return uint8_t Returns 1 when new valid data is latched
 */
uint8_t MPU9250_ReadMag(MPU9250 *mpu) {
    uint8_t mag_adjust[3] = { 0 };
    uint8_t mag_buffer[7] = { 0 };
    mpu->diag_mag_read_attempts++;

    (void) ak8963_r_reg(AK8963_ASAX, 3, mpu, mag_adjust);
    if ((mag_adjust[0] | mag_adjust[1] | mag_adjust[2]) != 0U) {
        memcpy(mpu->mag_adjust, mag_adjust, sizeof(mag_adjust));
    }

    (void) ak8963_r_reg(MAG_XOUT_L, 7, mpu, mag_buffer);
    if ((mag_buffer[6] & AK8963_ST2_HOFL) != 0U) {
        mpu->diag_last_mag_status = MPU9250_DIAG_ERR_MAG_OVERFLOW;
        return 0;
    }

    if ((mag_buffer[0] | mag_buffer[1] | mag_buffer[2] | mag_buffer[3] | mag_buffer[4] | mag_buffer[5]) == 0U) {
        mpu->diag_last_mag_status = MPU9250_DIAG_ERR_MAG_EMPTY;
        return 0;
    }

    mpu->mpu_value.Mag_row[0] = ((int16_t) mag_buffer[1] << 8) | mag_buffer[0];
    mpu->mpu_value.Mag_row[1] = ((int16_t) mag_buffer[3] << 8) | mag_buffer[2];
    mpu->mpu_value.Mag_row[2] = ((int16_t) mag_buffer[5] << 8) | mag_buffer[4];

    mpu->mpu_value.Mag[0] = (float) mpu->mpu_value.Mag_row[0] * MPU9250_GetMagScale(mpu, 0);
    mpu->mpu_value.Mag[1] = (float) mpu->mpu_value.Mag_row[1] * MPU9250_GetMagScale(mpu, 1);
    mpu->mpu_value.Mag[2] = (float) mpu->mpu_value.Mag_row[2] * MPU9250_GetMagScale(mpu, 2);
    mpu->diag_last_mag_status = MPU9250_DIAG_OK;
    mpu->diag_mag_read_success++;

    return 1;
}

/**
 * @brief Read magnetometer data from all MPU9250 sensors in the SPI_SensorsGroup
 * @param spi_sensorsgroup Pointer to SPI_SensorsGroup structure
 */
void SensorGroup_ReadMag(SPI_SensorsGroup* spi_sensorsgroup) {
    uint8_t sensorNum = spi_sensorsgroup->mpuSensorNum;

    for (uint8_t i = 0; i < sensorNum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu == NULL) {
            continue;
        }

        if (MPU9250_ReadMag(mpu) == 0) {
            continue;
        }

        uint8_t sensor_id = (uint8_t)(spi_sensorsgroup->base_sensor_id + i);
        if (sensor_id < 12) {
            float mag_cal[3];
            applyMagCalibration(sensor_id, mpu->mpu_value.Mag, mag_cal);
            mpu->mpu_value.Mag[0] = mag_cal[0];
            mpu->mpu_value.Mag[1] = mag_cal[1];
            mpu->mpu_value.Mag[2] = mag_cal[2];
        }

        MPU9250_ApplyMagDespike(mpu, mpu->mpu_value.Mag);
    }
}

/**
 * @brief Read accelerate data from all MPU9250 sensors in the SPI_SensorsGroup
 * @param spi_sensorsgroup Pointer to SPI_SensorsGroup structure
 */
void SensorGroup_ReadAccel(SPI_SensorsGroup* spi_sensorsgroup) {
    uint8_t sensorNum = spi_sensorsgroup->mpuSensorNum;
    for (uint8_t i = 0; i < sensorNum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu == NULL) {
            continue;
        }
        MPU9250_ReadAccel(mpu);
    }
}

/**
 * @brief Read gyroscope data from all MPU9250 sensors in the SPI_SensorsGroup
 * @param spi_sensorsgroup
 */
void SensorGroup_ReadGyro(SPI_SensorsGroup* spi_sensorsgroup) {
    uint8_t sensorNum = spi_sensorsgroup->mpuSensorNum;
    for (uint8_t i = 0; i < sensorNum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu == NULL) {
            continue;
        }
        MPU9250_ReadGyro(mpu);
    }
}

/**
 * @brief Read all data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ReadData(MPU9250 *mpu) {
    MPU9250_ReadAccel(mpu);
    MPU9250_ReadGyro(mpu);
    (void) MPU9250_ReadMag(mpu);
}

void SensorsGroup_ReadData(SPI_SensorsGroup* spi_sensorsgroup) {
    uint8_t sensorNum = spi_sensorsgroup->mpuSensorNum;
    for (uint8_t i = 0; i < sensorNum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu == NULL) {
            continue;
        }
        MPU9250_ReadData(mpu);
    }
}

void SensorGroup_DebugPrint(const char *tag, SPI_SensorsGroup* spi_sensorsgroup) {
    printf("[%s]\r\n", tag);
    for (uint8_t i = 0; i < spi_sensorsgroup->mpuSensorNum; i++) {
        MPU9250 *mpu = SensorGroup_GetSensor(spi_sensorsgroup, i);
        if (mpu == NULL) {
            continue;
        }

        printf("S%u mpu=0x%02X ak=0x%02X init=%u mag=%u aux=0x%02X data=0x%02X uc=0x%02X mc=0x%02X op=0x%02X s4=0x%02X a4=0x%02X r4=0x%02X d4=0x%02X ok=%lu/%lu\r\n",
               (unsigned int)(spi_sensorsgroup->base_sensor_id + i + 1U),
               (unsigned int)mpu->diag_mpu_whoami,
               (unsigned int)mpu->diag_ak8963_whoami,
               (unsigned int)mpu->diag_init_error,
               (unsigned int)mpu->diag_last_mag_status,
               (unsigned int)mpu->diag_last_i2c_mst_status,
               (unsigned int)mpu->diag_last_aux_data,
               (unsigned int)mpu->diag_user_ctrl,
               (unsigned int)mpu->diag_i2c_mst_ctrl,
               (unsigned int)mpu->diag_last_aux_op,
               (unsigned int)mpu->diag_last_slv4_ctrl,
               (unsigned int)mpu->diag_last_slv4_addr,
               (unsigned int)mpu->diag_last_slv4_reg,
               (unsigned int)mpu->diag_last_slv4_do,
               (unsigned long)mpu->diag_mag_read_success,
               (unsigned long)mpu->diag_mag_read_attempts);
    }
}
