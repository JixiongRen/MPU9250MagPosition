//
// Created by renji on 25-1-31.
//

#ifndef MPU9250_H
#define MPU9250_H

# include  <stdint.h>
# include  <math.h>
# include  "main.h"

//--------------------------i2c slv0---------------------------------//
# define I2C_SLV0_ADDR       0x25   // I2C slave 0 address
# define I2C_SLV0_REG        0x26   // I2C slave 0 register
# define I2C_SLV0_CTRL       0x27   // I2C slave 0 control
# define I2C_SLV0_DO         0x63   // I2C slave 0 data out register

//-----------------------AK8963 reg addr-----------------------------//
# define MPU9250_AK8963_ADDR 0x0C   // AK8963 address
# define AK8963_WHOAMI_REG   0x00   // Device ID, Should return 0x48
# define AK8963_WHOAMI_ANS   0x48   // Device ID answer
# define AK8963_ST1_REG      0x02   // Data Status 1
# define AK8963_ST2_REG      0x09   // Data Status 2
# define AK8963_ST1_DOR      0x00   // Data overflow
#define AK8963_ST1_DRDY      0x01   // Data ready
#define AK8963_ST2_BITM      0x08   // Bit M
#define AK8963_ST2_HOFL      0x08   // Magnetic sensor overflow
#define AK8963_CNTL1_REG     0x0A   // Control 1
#define AK8963_CNTL2_REG     0x0B   // Control 2
#define AK8963_CNTL2_SRST    0x01   // Soft reset
#define AK8963_ASAX          0x10   // X-axis sensitivity adjustment value
#define AK8963_ASAY          0x11   // Y-axis sensitivity adjustment value
#define AK8963_ASAZ          0x12   // Z-axis sensitivity adjustment value

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

#define SMPLRT_DIV           0x19   // Sample rate divider
#define CONFIG               0x1A   // Configuration
#define GYRO_CONFIG          0x1B   // Gyroscope configuration
#define ACCEL_CONFIG         0x1C   // Accelerometer configuration
#define ACCEL_CONFIG_2       0x1D   // Accelerometer configuration 2

#define INT_PIN_CFG          0x37   //中断配置
#define USER_CTRL            0x6a
#define I2C_MST_CTRL         0x24
#define I2C_MST_DELAY_CTRL   0x67

//------------------------SPI CS-------------------------------//
#define MPU_9250_DISENABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define MPU_9250_ENABLE()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)


uint8_t MPU9250_Write_Reg(uint8_t reg, uint8_t value, SPI_HandleTypeDef hspix);
uint8_t MPU9250_Read_Reg(uint8_t reg, SPI_HandleTypeDef hspix);
static void i2c_Mag_write(uint8_t reg, uint8_t value, SPI_HandleTypeDef hspix);
static uint8_t i2c_Mag_read(uint8_t reg, SPI_HandleTypeDef hspix);
void Init_MPU9250(SPI_HandleTypeDef hspix);
void READ_MPU9250_ACCEL(SPI_HandleTypeDef hspix);
void READ_MPU9250_GYRO(SPI_HandleTypeDef hspix);
void READ_MPU9250_MAG(SPI_HandleTypeDef hspix);

#endif //MPU9250_H
