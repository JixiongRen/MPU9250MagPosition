//
// Created by renji on 25-1-31.
//

#ifndef MPU9250_H
#define MPU9250_H

#include  <stdint.h>
#include  <math.h>
#include  "main.h"
#include "stm32f4xx_hal_spi.h"
#include <stdlib.h>

//----------------------config register--------------------------/
#define SMPLRT_DIV           0x19   // Sample rate divider
#define CONFIG               0x1A   // Configuration
#define GYRO_CONFIG          0x1B   // Gyroscope configuration
#define ACCEL_CONFIG         0x1C   // Accelerometer configuration
#define ACCEL_CONFIG_2       0x1D   // Accelerometer configuration 2

#define INT_ENABLE           0x38   // Register: Interrupt Enable
#define INT_PIN_CFG          0x37
#define INT_STATUS           0x3A
#define USER_CTRL            0x6a
#define I2C_MST_CTRL         0x24
#define I2C_MST_DELAY_CTRL   0x67

//--------------------------i2c slv0---------------------------------//
#define I2C_SLV0_ADDR        0x25   // I2C slave 0 address
#define I2C_SLV0_REG         0x26   // I2C slave 0 register
#define I2C_SLV0_CTRL        0x27   // I2C slave 0 control
#define I2C_SLV0_DO          0x63   // I2C slave 0 data out register

//-----------------------AK8963 reg addr-----------------------------//
#define AK8963_I2C_ADDR      0x0C  //AKM addr
#define AK8963_WHOAMI_REG    0x00  //AKM ID addr
#define AK8963_WHOAMI_ID     0x48  //ID
#define AK8963_ST1_REG       0x02  //Data Status1
#define AK8963_ST2_REG       0x09  //Data reading end register & check Magnetic sensor overflow occurred
#define AK8963_ST1_DOR       0x02
#define AK8963_ST1_DRDY      0x01  //Data Ready
#define AK8963_ST2_BITM      0x10
#define AK8963_ST2_HOFL      0x08  // Magnetic sensor overflow
#define AK8963_CNTL1_REG     0x0A
#define AK8963_CNTL2_REG     0x0B
#define AK8963_CNTL2_SRST    0x01  //soft Reset
#define AK8963_ASAX          0x10  //X-axis sensitivity adjustment value
#define AK8963_ASAY          0x11  //Y-axis sensitivity adjustment value
#define AK8963_ASAZ          0x12  //Z-axis sensitivity adjustment value

//-----------------------9-axis reg addr-----------------------------//
#define ACCEL_XOUT_H         0x3B   // Accelerometer X-axis high byte
#define ACCEL_XOUT_L         0x3C   // Accelerometer X-axis low byte
#define ACCEL_YOUT_H         0x3D   // Accelerometer Y-axis high byte
#define ACCEL_YOUT_L         0x3E   // Accelerometer Y-axis low byte
#define ACCEL_ZOUT_H         0x3F   // Accelerometer Z-axis high byte
#define ACCEL_ZOUT_L         0x40   // Accelerometer Z-axis low byte

#define TEMP_OUT_H           0x41   // Temp high byte
#define TEMP_OUT_L           0x42   // Temp low byte

#define GYRO_XOUT_H          0x43   // Gyroscope X-axis high byte
#define GYRO_XOUT_L          0x44   // Gyroscope X-axis low byte
#define GYRO_YOUT_H          0x45   // Gyroscope Y-axis high byte
#define GYRO_YOUT_L          0x46   // Gyroscope Y-axis low byte
#define GYRO_ZOUT_H          0x47   // Gyroscope Z-axis high byte
#define GYRO_ZOUT_L          0x48   // Gyroscope Z-axis low byte

#define MAG_XOUT_L           0x03   // Magnetometer X-axis low byte
#define MAG_XOUT_H           0x04   // Magnetometer X-axis high byte
#define MAG_YOUT_L           0x05   // Magnetometer Y-axis low byte
#define MAG_YOUT_H           0x06   // Magnetometer Y-axis high byte
#define MAG_ZOUT_L           0x07   // Magnetometer Z-axis low byte
#define MAG_ZOUT_H           0x08   // Magnetometer Z-axis high byte

//------------------------other addr-------------------------------//
#define PWR_MGMT_1           0x6B   // Power management 1
#define PWR_MGMT_2           0x6C   // Power management 2

#define WHO_AM_I             0x75   // Device ID, Should return 0x71
#define WHO_AM_I_ANS         0x71   // Device ID answer

#define EXT_SENS_DATA_00     0x49   // External sensor data 00
#define EXT_SENS_DATA_01     0x4A   // External sensor data 01
#define EXT_SENS_DATA_02     0x4B   // External sensor data 02
#define EXT_SENS_DATA_03     0x4C   // External sensor data 03

//------------------------SPI CS-------------------------------//
// #define MPU_9250_DISENABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
// #define MPU_9250_ENABLE()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)

//-----------------------config Structure---------------------------//
typedef enum __MPU9250_AccelRange {
    MPU9250_Accel_Range_2G = 0x00,
    MPU9250_Accel_Range_4G = 0x08,
    MPU9250_Accel_Range_8G = 0x10,
    MPU9250_Accel_Range_16G = 0x18
/*
 * 0b00000000 => 0x00 for +-2G
 * 0b00001000 => 0x08 for +-4G
 * 0b00010000 => 0x10 for +-8G
 * 0b00011000 => 0x18 for +-16G
 * */
} MPU9250_AccelRange;

typedef enum __MPU9250_GyroRange {
    MPU9250_Gyro_Range_250dps = 0x00,
    MPU9250_Gyro_Range_500dps = 0x08,
    MPU9250_Gyro_Range_1000dps = 0x10,
    MPU9250_Gyro_Range_2000dps = 0x18
/*
 * 0b00000000 for 250dps
 * 0b00001000 for 500dps
 * 0b00010000 for 1000dps
 * 0b00011000 for 2000dps
 * */
} MPU9250_GyroRange;

typedef enum __MPU9250_Accel_DLPFBandwidth {
    // set reg29 Bit[3] & Bit[2:0]
    // Hz
    MPU9250_Accel_DLPFBandwidth_460 = 0x00,    // delay 1.94ms
    MPU9250_Accel_DLPFBandwidth_184,           // delay 5.80ms
    MPU9250_Accel_DLPFBandwidth_92,            // delay 7.80ms
    MPU9250_Accel_DLPFBandwidth_41,            // delay 11.80ms
    MPU9250_Accel_DLPFBandwidth_20,            // delay 19.80ms
    MPU9250_Accel_DLPFBandwidth_10,            // delay 35.70ms
    MPU9250_Accel_DLPFBandwidth_5,             // delay 66.96ms
    MPU9250_Accel_DLPFBandwidth_460_2,         // delay 1.94ms
    MPU9250_Accel_DLPFBandwidth_1_13k = 0x08,  // delay 0.75ms
} MPU9250_Accel_DLPFBandwidth;

typedef enum __MPU9250_Accel_SampleRateDivider {
    // check reg30, when use lower power mode
    // Hz
    // Bandwidth 1.1kHz, Delay 1ms
    MPU9250_LP_ACCEL_ODR_0_24HZ = 0x00,
    MPU9250_LP_ACCEL_ODR_0_49HZ,
    MPU9250_LP_ACCEL_ODR_0_98HZ,
    MPU9250_LP_ACCEL_ODR_1_95HZ,
    MPU9250_LP_ACCEL_ODR_3_91HZ,
    MPU9250_LP_ACCEL_ODR_7_81HZ,
    MPU9250_LP_ACCEL_ODR_15_63HZ,
    MPU9250_LP_ACCEL_ODR_31_25HZ,
    MPU9250_LP_ACCEL_ODR_62_50HZ,
    MPU9250_LP_ACCEL_ODR_125HZ,
    MPU9250_LP_ACCEL_ODR_250HZ,
    MPU9250_LP_ACCEL_ODR_500HZ
} MPU9250_Accel_SampleRateDivider;

typedef enum __MPU9250_Gyro_DLPFBandwidth {
    // to use following 2 options, reg27 Bit[1:0](fchoice_b) must be set
    // these to options are not related to reg26 Bit[2:0](DLPF_CFG), but reg27 Bit[1:0](fchoice_b)
    // notice: this options can also affect temperature sensor
    // Hz
    MPU9250_Gyro_DLPFBandwidth_8800_x = 0x00,    // delay 0.064ms
    MPU9250_Gyro_DLPFBandwidth_3600_x = 0x00,    // delay 0.11ms
    // follow options can be set to reg26 Bit[2:0](DLPF_CFG) to set DLPFBandwidth
    MPU9250_Gyro_DLPFBandwidth_250 = 0x00,     // delay 0.97ms
    MPU9250_Gyro_DLPFBandwidth_184,            // delay 2.9ms
    MPU9250_Gyro_DLPFBandwidth_92,             // delay 3.9ms
    MPU9250_Gyro_DLPFBandwidth_41,             // delay 5.9ms
    MPU9250_Gyro_DLPFBandwidth_20,             // delay 9.9ms
    MPU9250_Gyro_DLPFBandwidth_10,             // delay 17.85ms
    MPU9250_Gyro_DLPFBandwidth_5,              // delay 33.48ms
    MPU9250_Gyro_DLPFBandwidth_3600,           // delay 0.17ms
} MPU9250_Gyro_DLPFBandwidth;

typedef struct __MPU9250_Value{
    // row data
    volatile int16_t Accel_row[3];
    volatile int16_t Gyro_row[3];
    volatile int16_t Mag_row[3];
    float Accel[3]; // Accel X,Y,Z
    float Gyro[3];  // Gyro X,Y,Z
    float Mag[3];   // Mag X,Y,Z
} MPU_Value;

typedef struct __MPU9250_CFG {
    SPI_HandleTypeDef hspix;
    GPIO_TypeDef* CS_Port;
    uint16_t CS_Pin;
} MPU9250_CFG;

typedef struct __MPU9250 {
    MPU9250_CFG mpu9250_cfg;
    MPU_Value mpu_value;
} MPU9250;

typedef struct __SPI_SensorsGroup {
    uint8_t mpuSensorNum;
    MPU9250 mpuSensor1;
    MPU9250 mpuSensor2;
    MPU9250 mpuSensor3;
    MPU9250 mpuSensor4;
} SPI_SensorsGroup;

//extern MPU_Value mpu_value;

void MPU9250_ENABLE(MPU9250 *mpu);
void MPU9250_DISENABLE(MPU9250 *mpu);
void delay_10us(void);
// 以下7个均为static函数
uint8_t spi_w_byte(SPI_HandleTypeDef hspix, uint8_t byte);
void spi_w_bytes(uint8_t reg, MPU9250 *mpu, uint8_t *bytes, uint16_t num);
void spi_r_bytes(uint8_t reg, MPU9250 *mpu, uint8_t num);
void mpu_w_reg(uint8_t reg, uint8_t byte, MPU9250 *mpu);
void mpu_r_reg(uint8_t reg, uint8_t num, MPU9250 *mpu);
void ak8963_w_reg(uint8_t reg, uint8_t byte, MPU9250 *mpu);
void ak8963_r_reg(uint8_t reg, uint8_t num, MPU9250 *mpu);
uint8_t mpu_r_ak8963_WhoAmI(MPU9250 *mpu);
uint8_t mpu_r_WhoAmI(MPU9250 *mpu);
void MPU9250_StructInit(MPU9250 *mpu, SPI_HandleTypeDef hspix, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void SensorGroup_StructInit(SPI_SensorsGroup* spi_sensorsgroup, uint8_t sensornum, SPI_HandleTypeDef ghspix[], GPIO_TypeDef *gcs_port[], uint16_t gcs_pin[]);
uint8_t MPU9250_Init(MPU9250 *mpu);
uint8_t SensorGroup_Init(SPI_SensorsGroup* spi_sensorsgroup);
void MPU9250_ReadAccel(MPU9250 *mpu);
void MPU9250_ReadGyro(MPU9250 *mpu);
void MPU9250_ReadMag(MPU9250 *mpu);
void SensorGroup_ReadMag(SPI_SensorsGroup* spi_sensorsgroup);
void SensorGroup_ReadAccel(SPI_SensorsGroup* spi_sensorsgroup);
void SensorGroup_ReadGyro(SPI_SensorsGroup* spi_sensorsgroup);
void MPU9250_ReadData(MPU9250 *mpu);

#endif //MPU9250_H
