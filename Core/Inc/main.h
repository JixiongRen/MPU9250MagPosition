/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "MPU9250.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "usart.h"
#include "cmsis_os2.h"
#include "freertos_os2.h"
#include "FreeRTOSVariables.h"



/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  extern uint8_t ak8963_WhoAmI;
  extern uint8_t mpu9250_WhoAmI;

  extern MPU9250 mpu_instance_1;
  extern MPU9250 *mpu_1;

  extern MPU9250 mpu_instance_2;
  extern MPU9250 *mpu_2;

  extern SPI_SensorsGroup spi_sensorsgroup_instance_1;
  extern SPI_SensorsGroup *spi_sensorsgroup_1;

  extern SPI_SensorsGroup spi_sensorsgroup_instance_2;
  extern SPI_SensorsGroup *spi_sensorsgroup_2;

  extern SPI_SensorsGroup spi_sensorsgroup_instance_3;
  extern SPI_SensorsGroup *spi_sensorsgroup_3;

  extern uint8_t sensorsnum;

  extern SPI_HandleTypeDef *ghspix_1;
  extern GPIO_TypeDef **gcs_port_1;
  extern uint16_t *gcs_pin_1;

  extern SPI_HandleTypeDef *ghspix_2;
  extern GPIO_TypeDef **gcs_port_2;
  extern uint16_t *gcs_pin_2;

  extern SPI_HandleTypeDef *ghspix_3;
  extern GPIO_TypeDef **gcs_port_3;
  extern uint16_t *gcs_pin_3;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MPU9250_CHIP_G3S3_Pin GPIO_PIN_6
#define MPU9250_CHIP_G3S3_GPIO_Port GPIOF
#define MPU9250_CHIP_G3S4_Pin GPIO_PIN_7
#define MPU9250_CHIP_G3S4_GPIO_Port GPIOF
#define MPU9250_CHIP_G2S3_Pin GPIO_PIN_0
#define MPU9250_CHIP_G2S3_GPIO_Port GPIOC
#define MPU9250_CHIP_G2S4_Pin GPIO_PIN_1
#define MPU9250_CHIP_G2S4_GPIO_Port GPIOC
#define MPU9250_CHIP_G3S1_Pin GPIO_PIN_4
#define MPU9250_CHIP_G3S1_GPIO_Port GPIOC
#define MPU9250_CHIP_G3S2_Pin GPIO_PIN_5
#define MPU9250_CHIP_G3S2_GPIO_Port GPIOC
#define MPU9250_CHIP_G1S1_Pin GPIO_PIN_0
#define MPU9250_CHIP_G1S1_GPIO_Port GPIOB
#define MPU9250_CHIP_G1S2_Pin GPIO_PIN_1
#define MPU9250_CHIP_G1S2_GPIO_Port GPIOB
#define MPU9250_CHIP_G1S3_Pin GPIO_PIN_2
#define MPU9250_CHIP_G1S3_GPIO_Port GPIOB
#define S_SPI_SS_GPIO_Pin GPIO_PIN_6
#define S_SPI_SS_GPIO_GPIO_Port GPIOG
#define S_SPI_SCK_GPIO_Pin GPIO_PIN_7
#define S_SPI_SCK_GPIO_GPIO_Port GPIOG
#define S_SPI_MOSI_GPIO_Pin GPIO_PIN_8
#define S_SPI_MOSI_GPIO_GPIO_Port GPIOG
#define S_SPI_MISO_GPIO_Pin GPIO_PIN_11
#define S_SPI_MISO_GPIO_GPIO_Port GPIOG
#define LCD_RES_Pin GPIO_PIN_13
#define LCD_RES_GPIO_Port GPIOG
#define LCD_DC_Pin GPIO_PIN_14
#define LCD_DC_GPIO_Port GPIOG
#define MPU9250_CHIP_G1S4_Pin GPIO_PIN_3
#define MPU9250_CHIP_G1S4_GPIO_Port GPIOB
#define MPU9250_CHIP_G2S1_Pin GPIO_PIN_4
#define MPU9250_CHIP_G2S1_GPIO_Port GPIOB
#define MPU9250_CHIP_G2S2_Pin GPIO_PIN_5
#define MPU9250_CHIP_G2S2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
