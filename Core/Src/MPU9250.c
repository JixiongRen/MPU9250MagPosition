//
// Created by renji on 25-1-31.
//

#include "main.h"
#include "MPU9250.h"
#include <sys/types.h>
#include "stm32f4xx_hal.h"  // Modify according to your specific STM32 series

#define DATABUF_SIZE    64

static uint8_t dataBuf[DATABUF_SIZE] = { 0 };  // Buffer that stores data which has been read
static uint16_t i = 0; // loop control
static uint16_t I = 0;


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
uint8_t spi_w_byte(SPI_HandleTypeDef hspix, uint8_t byte) {
    uint8_t feedback = 0;
    while (HAL_SPI_GetState(&hspix) == HAL_SPI_STATE_BUSY_TX_RX);
    if (HAL_SPI_TransmitReceive(&hspix, &byte, &feedback, 1, 0x01f4) != HAL_OK) {
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
    MPU9250_ENABLE(mpu);
    SPI_HandleTypeDef hspix = mpu->mpu9250_cfg.hspix;
    spi_w_byte(hspix, reg);
    for (i=0; i<num; i++) {
        spi_w_byte(hspix, bytes[i]);
    }
    MPU9250_DISENABLE(mpu);
}


/**
 * @brief Read multiple bytes using SPI
 * @param reg Target register address to read
 * @param mpu Pointer to MPU9250 structure
 * @param num Number of bytes to read using SPI
 */
void spi_r_bytes(uint8_t reg, MPU9250 *mpu, uint8_t num)
{
    SPI_HandleTypeDef hspix = mpu->mpu9250_cfg.hspix;
    reg |= 0x80;  // The highest bit is 1 when reading the register
    MPU9250_ENABLE(mpu);

    HAL_SPI_Transmit(&hspix, &reg, 1, 0x01f4);
    // The returned data is stored in dataBuf
    HAL_SPI_Receive(&hspix, dataBuf, num, 0x01f4);

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
 */
void mpu_r_reg(uint8_t reg, uint8_t num, MPU9250 *mpu) {
    spi_r_bytes(reg, mpu, num);
}


/**
 * @brief Write data to AK8963 register
 * @param reg Target register address
 * @param byte Data to be written
 * @param mpu Pointer to MPU9250 structure
 */
void ak8963_w_reg(uint8_t reg, uint8_t byte, MPU9250 *mpu) {
    mpu_w_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR, mpu);
    mpu_w_reg(I2C_SLV0_REG, reg, mpu);
    mpu_w_reg(I2C_SLV0_DO, byte, mpu);
    //mpu_w_reg(I2C_SLV0_CTRL, 0x81, mpu);
    mpu_w_reg(I2C_SLV0_CTRL, 0x87, mpu);
}


/**
 * @brief Read data from AK8963 register
 * @param reg Register address
 * @param num Number of bytes to read
 * @param mpu Pointer to MPU9250 structure
 */
void ak8963_r_reg(uint8_t reg, uint8_t num, MPU9250 *mpu) {
    mpu_w_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80, mpu);
    mpu_w_reg(I2C_SLV0_REG, reg, mpu);
    mpu_w_reg(I2C_SLV0_DO, num | 0x80, mpu);
    // mpu_w_reg(I2C_SLV0_DO, num, mpu);
    HAL_Delay(1);
    mpu_r_reg(EXT_SENS_DATA_00, num, mpu);
}

/**
 * @brief Confirm the device ID of MPU9250
 * @param mpu Pointer to MPU9250 structure
 * @return uint8_t Returns the value of the WHO_AM_I register of MPU9250
 */
uint8_t mpu_r_WhoAmI(MPU9250 *mpu) {
    mpu_r_reg(WHO_AM_I, 1, mpu);
    return dataBuf[0];
}


/**
 * @brief Confirm the device ID of AK8963
 * @param mpu Pointer to MPU9250 structure
 * @return uint8_t Returns the value of the WHO_AM_I register of AK8963
 */
uint8_t mpu_r_ak8963_WhoAmI(MPU9250 *mpu) {
    ak8963_r_reg(AK8963_WHOAMI_REG, 1, mpu);
    return dataBuf[0];
}


/**
 * @brief Initialize MPU9250 structure
 * @param mpu Pointer to MPU9250 structure
 * @param hspix SPI handle
 * @param cs_port GPIO port number connected to MPU chip select pin
 * @param cs_pin GPIO pin number connected to MPU chip select pin
 */
void MPU9250_StructInit(MPU9250 *mpu, SPI_HandleTypeDef hspix, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    mpu->mpu9250_cfg.hspix = hspix;
    mpu->mpu9250_cfg.CS_Port = cs_port;
    mpu->mpu9250_cfg.CS_Pin = cs_pin;

    for (uint8_t i = 0; i < 3; i++) {
        mpu->mpu_value.Accel[i] = 0;
        mpu->mpu_value.Gyro[i] = 0;
        mpu->mpu_value.Mag[i] = 0;

        mpu->mpu_value.Accel_row[i] = 0;
        mpu->mpu_value.Gyro_row[i] = 0;
        mpu->mpu_value.Mag_row[i] = 0.0;
    }
}

/**
 * @brief Initialize SPI_SensorsGroup structure
 * @param spi_sensorsgroup Pointer to SPI_SensorsGroup structure
 * @param sensornum Number of MPU9250 sensors in the group
 * @param ghspix Array of SPI handles for each MPU9250 sensor
 * @param gcs_port Array of GPIO ports for each MPU9250 sensor's chip select pin
 * @param gcs_pin Array of GPIO pins for each MPU9250 sensor's chip select pin
 */
void SensorGroup_StructInit(SPI_SensorsGroup* spi_sensorsgroup, uint8_t sensornum, SPI_HandleTypeDef ghspix[], GPIO_TypeDef *gcs_port[], uint16_t gcs_pin[]) {
    // 记录传感器数量
    spi_sensorsgroup->mpuSensorNum = sensornum;
    // 遍历每个传感器进行初始化
    for (uint8_t i = 0; i < sensornum; i++) {
        MPU9250 *mpu;
        switch (i) {
            case 0:
                mpu = &(spi_sensorsgroup->mpuSensor1);
            break;
            case 1:
                mpu = &(spi_sensorsgroup->mpuSensor2);
            break;
            case 2:
                mpu = &(spi_sensorsgroup->mpuSensor3);
            break;
            case 3:
                mpu = &(spi_sensorsgroup->mpuSensor4);
            break;
            default:
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
    // MPU9250_Value_StructInit(mpu);
    mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x80, mpu); // reset MPU9250, reg107
    HAL_Delay(10);
    mpu_w_reg(USER_CTRL, (uint8_t) 0x20, mpu); // enable I2C master mode, reg106
    mpu_w_reg(I2C_MST_CTRL, (uint8_t) 0x0D, mpu); // set I2C clock speed to 400kHz, reg36
    ak8963_w_reg(AK8963_CNTL1_REG, (uint8_t) 0x00, mpu); // set AK8963 to power down
    mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x80, mpu); // reset MPU9250, Bit[7] will auto clear
    HAL_Delay(10);
    ak8963_w_reg(AK8963_CNTL2_REG, AK8963_CNTL2_SRST, mpu); // reset AK8963
    mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x01, mpu); // select clock source
    mpu_w_reg(PWR_MGMT_2, (uint8_t) 0x00, mpu); // enable accel and gyro// enable data ready interrupt
    /* init GYRO and ACCEL */
    mpu_w_reg(SMPLRT_DIV, (uint8_t) 0x00, mpu); // SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV), Internal_Sample_Rate==8K
    mpu_w_reg(GYRO_CONFIG, (uint8_t) MPU9250_Gyro_Range_2000dps, mpu); // gyro full scale select
    mpu_w_reg(ACCEL_CONFIG, (uint8_t) MPU9250_Accel_Range_16G, mpu); // accel full scale select
    mpu_w_reg(ACCEL_CONFIG_2, (uint8_t) MPU9250_Accel_DLPFBandwidth_460, mpu);
    mpu_w_reg(CONFIG, (uint8_t) MPU9250_Gyro_DLPFBandwidth_250, mpu);
    /* init MAG */
    mpu_w_reg(USER_CTRL, (uint8_t) 0x20, mpu); // enable I2C master mode
    mpu_w_reg(I2C_MST_CTRL, (uint8_t) 0x0D, mpu); // set I2C clock speed to 400kHz, reg36
    ak8963_w_reg(AK8963_CNTL1_REG, (uint8_t) 0x00, mpu); // set AK8963 to power down
    HAL_Delay(100);
    ak8963_w_reg(AK8963_CNTL1_REG, (uint8_t) 0x0f, mpu); // set AK8963 to Fuse ROM access mode
    HAL_Delay(100);
    ak8963_w_reg(AK8963_CNTL1_REG, (uint8_t) 0x00, mpu); // set AK8963 to power down
    HAL_Delay(100);
    ak8963_w_reg(AK8963_CNTL1_REG, (uint8_t) 0x16, mpu); // AK8963 working on Continuous measurement mode 2 & 16-bit output
    HAL_Delay(100);
    mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x01, mpu); // select clock source
    ak8963_r_reg(MAG_XOUT_L, 1, mpu);
    ak8963_r_reg(AK8963_ST2_REG, 1, mpu);
    if (dataBuf[0] & AK8963_ST2_HOFL) { }
    return 0x00;
}

/**
 * @brief Initialize all MPU9250 sensors in the SPI_SensorsGroup
 * @param spi_sensorsgroup Pointer to SPI_SensorsGroup structure
 * @return uint8_t Returns 0x00 indicating successful initialization of all sensors
 */
uint8_t SensorGroup_Init(SPI_SensorsGroup* spi_sensorsgroup) {
    uint8_t sensorNum = spi_sensorsgroup->mpuSensorNum;
    for (uint8_t i = 0; i < sensorNum; i++) {
        MPU9250 *mpu;
        switch (i) {
            case 0:
                mpu = &(spi_sensorsgroup->mpuSensor1);
            break;
            case 1:
                mpu = &(spi_sensorsgroup->mpuSensor2);
            break;
            case 2:
                mpu = &(spi_sensorsgroup->mpuSensor3);
            break;
            case 3:
                mpu = &(spi_sensorsgroup->mpuSensor4);
            break;
            default:
                // 最多支持 4 个传感器，超出范围则跳过
                    continue;
        }
        // 调用 MPU9250_Init 函数初始化单个传感器
        MPU9250_Init(mpu);
    }
    return 0x00;
}

/**
 * @brief Read accelerometer data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ReadAccel(MPU9250 *mpu) {
    // m/s
    mpu_r_reg(ACCEL_XOUT_H, 6, mpu);
    // calculate x axis
    mpu->mpu_value.Accel_row[0] = ((int16_t)dataBuf[0] << 8) | dataBuf[1];
    mpu->mpu_value.Accel[0] = (float) mpu->mpu_value.Accel_row[0] / 208.980;

    // calculate y axis
    mpu->mpu_value.Accel_row[1] = ((int16_t)dataBuf[2] << 8) | dataBuf[3];
    mpu->mpu_value.Accel[1] = (float) mpu->mpu_value.Accel_row[1] / 208.980;

    // calculate z axis
    mpu->mpu_value.Accel_row[2] = ((int16_t)dataBuf[4] << 8) | dataBuf[5];
    mpu->mpu_value.Accel[2] = (float) mpu->mpu_value.Accel_row[2] / 208.980;
}


/**
 * @brief Read gyroscope data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ReadGyro(MPU9250 *mpu) {
    // d/s
    mpu_r_reg(GYRO_XOUT_H, 6, mpu);
    // calculate x axis
    mpu->mpu_value.Gyro_row[0] = ((int16_t)dataBuf[0] << 8) | dataBuf[1];
    mpu->mpu_value.Gyro[0] = mpu->mpu_value.Gyro_row[0] / 16.384;

    // calculate y axis
    mpu->mpu_value.Gyro_row[1] = ((int16_t)dataBuf[2] << 8) | dataBuf[3];
    mpu->mpu_value.Gyro[1] = mpu->mpu_value.Gyro_row[1] / 16.384;

    // calculate z axis
    mpu->mpu_value.Gyro_row[2] = ((int16_t)dataBuf[4] << 8) | dataBuf[5];
    mpu->mpu_value.Gyro[2] = mpu->mpu_value.Gyro_row[2] / 16.384;
}


/**
 * @brief Read magnetometer data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ReadMag(MPU9250 *mpu) {
    uint8_t mag_adjust[3] = { 0 };
    uint8_t mag_buffer[6] = { 0 };

    ak8963_r_reg(AK8963_ASAX, 3, mpu);
    mag_adjust[0] = dataBuf[0];
    mag_adjust[1] = dataBuf[1];
    mag_adjust[2] = dataBuf[2];

    // Read data from MAG_XOUT_L to MAG_ZOUT_H and ST2 register at once
    ak8963_r_reg(MAG_XOUT_L, 7, mpu);  // Read 6 bytes: MAG_XOUT_L, MAG_XOUT_H, MAG_YOUT_L, MAG_YOUT_H, MAG_ZOUT_L, MAG_ZOUT_H
    for (int i = 0; i < 6; i++) {
        mag_buffer[i] = dataBuf[i];
    }

    if (dataBuf[6] & AK8963_ST2_HOFL) { }

    mpu->mpu_value.Mag_row[0] = ((int16_t)mag_buffer[1] << 8) | mag_buffer[0];
    mpu->mpu_value.Mag_row[1] = ((int16_t)mag_buffer[3] << 8) | mag_buffer[2];
    mpu->mpu_value.Mag_row[2] = ((int16_t)mag_buffer[5] << 8) | mag_buffer[4];

    // calculate real value, check page53
    mpu->mpu_value.Mag[0] = (float) mpu->mpu_value.Mag_row[0]
            * (((mag_adjust[0] - 128) / 256.0) + 1);
    mpu->mpu_value.Mag[0] = mpu->mpu_value.Mag_row[0] * 0.15;

    mpu->mpu_value.Mag[1] = (float) mpu->mpu_value.Mag_row[1]
            * (((mag_adjust[1] - 128) / 256.0) + 1);
    mpu->mpu_value.Mag[1] = mpu->mpu_value.Mag_row[1] * 0.15;

    mpu->mpu_value.Mag[2] = (float) mpu->mpu_value.Mag_row[2]
            * (((mag_adjust[2] - 128) / 256.0) + 1);
    mpu->mpu_value.Mag[2] = mpu->mpu_value.Mag_row[2] * 0.15;
}


/**
 * @brief Read magnetometer data from all MPU9250 sensors in the SPI_SensorsGroup
 * @param spi_sensorsgroup Pointer to SPI_SensorsGroup structure
 */
void SensorGroup_ReadMag(SPI_SensorsGroup* spi_sensorsgroup) {
    uint8_t sensorNum = spi_sensorsgroup->mpuSensorNum;
    for (uint8_t i = 0; i < sensorNum; i++) {
        MPU9250 *mpu;
        switch (i) {
            case 0:
                mpu = &(spi_sensorsgroup->mpuSensor1);
            break;
            case 1:
                mpu = &(spi_sensorsgroup->mpuSensor2);
            break;
            case 2:
                mpu = &(spi_sensorsgroup->mpuSensor3);
            break;
            case 3:
                mpu = &(spi_sensorsgroup->mpuSensor4);
            break;
            default:
                    continue;
        }
        MPU9250_ReadMag(mpu);
    }
}


/**
 * @brief Read all data from MPU9250
 * @param mpu Pointer to MPU9250 structure
 */
void MPU9250_ReadData(MPU9250 *mpu) {
    MPU9250_ReadAccel(mpu);
    MPU9250_ReadGyro(mpu);
    MPU9250_ReadMag(mpu);
}
