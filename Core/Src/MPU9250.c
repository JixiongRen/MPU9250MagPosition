//
// Created by renji on 25-1-31.
//

#include "main.h"
#include "MPU9250.h"
#include "stm32f4xx_hal.h"  // 根据你的具体 STM32 系列修改

typedef struct {
    short Accel[3]; // Accel X,Y,Z
    short Gyro[3];  // Gyro X,Y,Z
    short Mag[3];   // Mag X,Y,Z
} MPU_Value;

MPU_Value mpu_value;

unsigned char BUF[6];


/**
 * @brief 向 MPU9250 写入寄存器
 * @param reg 寄存器地址
 * @param value 写入的值
 * @param hspix SPI 句柄
 * @retval 读取的值
 */
uint8_t MPU9250_Write_Reg(uint8_t reg, uint8_t value, SPI_HandleTypeDef hspix)
{
    uint8_t status;
    reg &= 0x7F;  // 写入寄存器时最高位为 0
    MPU_9250_ENABLE();
    status = HAL_SPI_TransmitReceive(&hspix, &reg, &reg, 1, 100);  // 发送寄存器地址
    if (status != HAL_OK)
    {
        return status;
    } else {
        HAL_SPI_TransmitReceive(&hspix, &value, &value, 1, 100);  // 发送数据
        MPU_9250_DISENABLE();
        return status;
    }
}

/**
 * @brief 读取 MPU9250 寄存器
 * @param reg 寄存器地址
 * @param hspix SPI 句柄
 * @retval 读取的值
 */
uint8_t MPU9250_Read_Reg(uint8_t reg, SPI_HandleTypeDef hspix)
{
    uint8_t status;
    uint8_t value;
    reg |= 0x80;  // 读取寄存器时最高位为 1
    MPU_9250_ENABLE();
    status = HAL_SPI_TransmitReceive(&hspix, &reg, &reg, 1, 100);  // 发送寄存器地址
    HAL_SPI_TransmitReceive(&hspix, &value, &value, 1, 100);  // 读取数据
    MPU_9250_DISENABLE();
    return value;
}


/**
 * @brief 通过 MPU 内部 I2C 总线写入 AK8963 寄存器
 * @param reg 寄存器地址
 * @param value 写入的值
 * @param hspix SPI 句柄
 */
static void i2c_Mag_write(uint8_t reg, uint8_t value, SPI_HandleTypeDef hspix)
{
    uint16_t j = 500;
    MPU9250_Write_Reg(I2C_SLV0_ADDR, MPU9250_AK8963_ADDR, hspix);  // 设计从机，即 AK8963 的地址
    MPU9250_Write_Reg(I2C_SLV0_REG, reg, hspix);  // 设置从机寄存器地址
    MPU9250_Write_Reg(I2C_SLV0_DO, value, hspix);  // 写入数据
    while(j--);  // 延时, 等待数据写入
}

/**
 * @brief 通过 MPU 内部 I2C 总线读取 AK8963 寄存器
 * @param reg 寄存器地址
 * @param hspix SPI 句柄
 * @retval 读取的值
 */
static uint8_t i2c_Mag_read(uint8_t reg, SPI_HandleTypeDef hspix)
{
    uint16_t j = 500;
    MPU9250_Write_Reg(I2C_SLV0_ADDR, MPU9250_AK8963_ADDR, hspix);  // 设计从机，即 AK8963 的地址
    MPU9250_Write_Reg(I2C_SLV0_REG, reg, hspix);  // 设置从机寄存器地址
    while(j--);  // 延时, 等待数据写入
    return MPU9250_Read_Reg(I2C_SLV0_DO, hspix);  // 读取数据
}

/**
 * @brief 初始化 MPU9250
 * @param hspix SPI 句柄
 * @note 该函数初始化 MPU9250 的寄存器，使其进入正常工作状态
 */
void Init_MPU9250(SPI_HandleTypeDef hspix)
{
    // 解除 MPU9250 的睡眠模式
    MPU9250_Write_Reg(PWR_MGMT_1, 0x00, hspix);
    // 低通滤波频率，设置典型值为 3600 Hz，该寄存器决定 Internal_Sample_Rate==8K
    MPU9250_Write_Reg(CONFIG, 0x07, hspix);
    /**********************Init SLV0 i2c**********************************/
    // 使用 SPI 总线读取从设备 0
    MPU9250_Write_Reg(INT_PIN_CFG, 0x30, hspix);  // INT Pin / Bypass Enable Configuration
    MPU9250_Write_Reg(I2C_MST_CTRL, 0x4d, hspix);  // I2C 主模式和速度 400 kHz
    MPU9250_Write_Reg(USER_CTRL, 0x20, hspix);  // I2C_MST _EN
    MPU9250_Write_Reg(I2C_MST_DELAY_CTRL, 0x01, hspix);  // 延时启用 I2C_SLV0 _DLY_
    MPU9250_Write_Reg(I2C_SLV0_CTRL, 0x81, hspix);  // 启用 IIC 并且 EXT_SENS_DATA==1 Byte

    /*******************Init GYRO and ACCEL******************************/
    MPU9250_Write_Reg(SMPLRT_DIV, 0x07, hspix);  // 陀螺仪采样频率，典型值：0x07 (1kHz) (SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV) )
    MPU9250_Write_Reg(GYRO_CONFIG, 0x18, hspix);  // 陀螺仪自检及测量范围，典型值：0x18 (不自检，2000deg/s)
    MPU9250_Write_Reg(ACCEL_CONFIG, 0x18, hspix);  // 加速度计自检、测量范围及高通滤波频率，典型值：0x18 (不自检，16G)
    MPU9250_Write_Reg(ACCEL_CONFIG_2, 0x08, hspix);  // 加速度计高通滤波频率，典型值：0x08 (1.13kHz)

    /**********************Init MAG **********************************/
    i2c_Mag_write(AK8963_CNTL2_REG, AK8963_CNTL2_SRST, hspix);  // 复位 AK8963
    i2c_Mag_write(AK8963_CNTL1_REG, 0x12, hspix);  // 使用 i2c 设置 AK8963 工作在连续测量模式 1 及 16 位输出
}

/**
 * @brief 读取 MPU9250 加速度计数据
 * @param hspix SPI 句柄
 * @note 该函数读取 MPU9250 加速度计数据，并将其转换为实际值
 */
void READ_MPU9250_ACCEL(SPI_HandleTypeDef hspix)
{
    // 定义加速度计寄存器地址数组
    uint8_t accel_reg[3][2] = {
        {ACCEL_XOUT_L, ACCEL_XOUT_H},
        {ACCEL_YOUT_L, ACCEL_YOUT_H},
        {ACCEL_ZOUT_L, ACCEL_ZOUT_H}
    };

    // 遍历每个轴，读取数据并进行转换
    for (int i = 0; i < 3; i++) {
        BUF[2 * i] = MPU9250_Read_Reg(accel_reg[i][0], hspix);   // 读取低字节
        BUF[2 * i + 1] = MPU9250_Read_Reg(accel_reg[i][1], hspix); // 读取高字节
        mpu_value.Accel[i] = (BUF[2 * i + 1] << 8) | BUF[2 * i];  // 组合高低字节
        mpu_value.Accel[i] /= 164;  // 转换数据
    }
}

/**
 * @brief 读取 MPU9250 陀螺仪数据
 * @param hspix SPI 句柄
 * @note 该函数读取 MPU9250 陀螺仪数据，并将其转换为实际值
 */
void READ_MPU9250_GYRO(SPI_HandleTypeDef hspix)
{
    // 定义重力计寄存器地址数组
    uint8_t gyro_reg[3][2] = {
        {GYRO_XOUT_L, GYRO_CONFIG},
        {GYRO_YOUT_L, GYRO_YOUT_H},
        {GYRO_ZOUT_L, GYRO_ZOUT_H}
    };

    // 遍历每个轴，读取数据并进行转换
    for (int i = 0; i < 3; i++) {
        BUF[2 * i] = MPU9250_Read_Reg(gyro_reg[i][0], hspix);   // 读取低字节
        BUF[2 * i + 1] = MPU9250_Read_Reg(gyro_reg[i][1], hspix); // 读取高字节
        mpu_value.Gyro[i] = (BUF[2 * i + 1] << 8) | BUF[2 * i];  // 组合高低字节
        mpu_value.Gyro[i] /= 16.4;  // 转换数据
    }
}

/**
 * @brief 读取 MPU9250 磁力计数据
 * @param hspix SPI 句柄
 * @note 该函数读取 MPU9250 磁力计数据，并将其转换为实际值
 */
void READ_MPU9250_MAG(SPI_HandleTypeDef hspix)
{
    // 读取磁力计三轴灵敏度值
    uint8_t mag_sens[3];
    mag_sens[0] = i2c_Mag_read(AK8963_ASAX, hspix);
    mag_sens[1] = i2c_Mag_read(AK8963_ASAY, hspix);
    mag_sens[2] = i2c_Mag_read(AK8963_ASAZ, hspix);

    // 检查数据是否准备好
    if (i2c_Mag_read(AK8963_ST1_REG, hspix) && (AK8963_ST1_DOR | AK8963_ST1_DRDY))
    {
        for (int i = 0; i < 3; i++)
        {
            uint8_t low_byte = i2c_Mag_read(MAG_XOUT_L + i * 2, hspix);
            if ((i2c_Mag_read(AK8963_ST2_REG, hspix) & AK8963_ST2_HOFL) == 0)
            {
                uint8_t high_byte = i2c_Mag_read(MAG_XOUT_H + i * 2, hspix);
                mpu_value.Mag[i] = (high_byte << 8) | low_byte;
                mpu_value.Mag[i] = mpu_value.Mag[i] * (((mag_sens[i] - 128) >> 8) + 1);
            }
        }
    }
}

// void READ_MPU9250_MAG(SPI_HandleTypeDef hspix)
// {
//     // 读取磁力计三轴灵敏度值
//     uint8_t x_axis, y_axis, z_axis;
//     x_axis = i2c_Mag_read(AK8963_ASAX, hspix);
//     y_axis = i2c_Mag_read(AK8963_ASAY, hspix);
//     z_axis = i2c_Mag_read(AK8963_ASAZ, hspix);

//     // 检查数据是否准备好
//     if (i2c_Mag_read(AK8963_ST1_REG, hspix) && (AK8963_ST1_DOR | AK8963_ST1_DRDY))
//     // DOR: 数据溢出，AK8963_ST1_REG 的 D1，0-Normal；1-Overrun
//     // DRDY: 数据准备好，AK8963_ST1_REG 的 D0，0-未准备好；1-准备好
//     {
//         // 读取 X 轴数据
//         BUF[0] = i2c_Mag_read(MAG_XOUT_L, hspix);  // 读取 X 轴低字节
//         if ((i2c_Mag_read(AK8963_ST2_REG, hspix) & AK8963_ST2_HOFL) == 0)
//         {
//             BUF[1] = i2c_Mag_read(MAG_XOUT_H, hspix);  // 读取 X 轴高字节
//         }
//         mpu_value.Mag[0] = (BUF[1] << 8) | BUF[0];  // 组合高低字节
//         mpu_value.Mag[0] = mpu_value.Mag[0] * (((x_axis - 128) >> 8) + 1);  // 转换数据

//         // 读取 Y 轴数据
//         BUF[2] = i2c_Mag_read(MAG_YOUT_L, hspix);  // 读取 Y 轴低字节
//         if ((i2c_Mag_read(AK8963_ST2_REG, hspix) & AK8963_ST2_HOFL) == 0)
//         {
//             BUF[3] = i2c_Mag_read(MAG_YOUT_H, hspix);  // 读取 Y 轴高字节
//         }
//         mpu_value.Mag[1] = (BUF[3] << 8) | BUF[2];  // 组合高低字节
//         mpu_value.Mag[1] = mpu_value.Mag[1] * (((y_axis - 128) >> 8) + 1);  // 转换数据

//         // 读取 Z 轴数据
//         BUF[4] = i2c_Mag_read(MAG_ZOUT_L, hspix);  // 读取 Z 轴低字节
//         if ((i2c_Mag_read(AK8963_ST2_REG, hspix) & AK8963_ST2_HOFL) == 0)
//         {
//             BUF[5] = i2c_Mag_read(MAG_ZOUT_H, hspix);  // 读取 Z 轴高字节
//         }
//         mpu_value.Mag[2] = (BUF[5] << 8) | BUF[4];  // 组合高低字节
//         mpu_value.Mag[2] = mpu_value.Mag[2] * (((z_axis - 128) >> 8) + 1);  // 转换数据
//     }
// }