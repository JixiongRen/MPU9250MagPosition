/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
  *NOTE: Write the variables that need to be called in other .c files into
  *      "FreeRTOSVariables.h" and prefix them with the extern flag.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOSVariables.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for rSpi01MagTask */
osThreadId_t rSpi01MagTaskHandle;
const osThreadAttr_t rSpi01MagTask_attributes = {
  .name = "rSpi01MagTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rSpi02MagTask */
osThreadId_t rSpi02MagTaskHandle;
const osThreadAttr_t rSpi02MagTask_attributes = {
  .name = "rSpi02MagTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rSpi03MagTask */
osThreadId_t rSpi03MagTaskHandle;
const osThreadAttr_t rSpi03MagTask_attributes = {
  .name = "rSpi03MagTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sGetDataStart */
osSemaphoreId_t sGetDataStartHandle;
const osSemaphoreAttr_t sGetDataStart_attributes = {
  .name = "sGetDataStart"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartrSpi01MagTask(void *argument);
void StartrSpi02MagTask(void *argument);
void StartrSpi03MagTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sGetDataStart */
  sGetDataStartHandle = osSemaphoreNew(1, 1, &sGetDataStart_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of rSpi01MagTask */
  rSpi01MagTaskHandle = osThreadNew(StartrSpi01MagTask, NULL, &rSpi01MagTask_attributes);

  /* creation of rSpi02MagTask */
  rSpi02MagTaskHandle = osThreadNew(StartrSpi02MagTask, NULL, &rSpi02MagTask_attributes);

  /* creation of rSpi03MagTask */
  rSpi03MagTaskHandle = osThreadNew(StartrSpi03MagTask, NULL, &rSpi03MagTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartrSpi01MagTask */
/**
  * @brief  Function implementing the rSpi01MagTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartrSpi01MagTask */
__weak void StartrSpi01MagTask(void *argument)
{
  /* USER CODE BEGIN StartrSpi01MagTask */
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(sGetDataStartHandle, portMAX_DELAY) == pdTRUE) {
      // Simulate the time required to complete the SPI transaction
      SensorGroup_ReadMag(spi_sensorsgroup_1);
    }
  }
  /* USER CODE END StartrSpi01MagTask */
}

/* USER CODE BEGIN Header_StartrSpi02MagTask */
/**
* @brief Function implementing the rSpi02MagTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartrSpi02MagTask */
__weak void StartrSpi02MagTask(void *argument)
{
  /* USER CODE BEGIN StartrSpi02MagTask */
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(sGetDataStartHandle, portMAX_DELAY) == pdTRUE) {
      // Simulate the time required to complete the SPI transaction
      HAL_Delay(9);
    }
  }
  /* USER CODE END StartrSpi02MagTask */
}

/* USER CODE BEGIN Header_StartrSpi03MagTask */
/**
* @brief Function implementing the rSpi03MagTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartrSpi03MagTask */
__weak void StartrSpi03MagTask(void *argument)
{
  /* USER CODE BEGIN StartrSpi03MagTask */
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(sGetDataStartHandle, portMAX_DELAY) == pdTRUE) {
      // Simulate the time required to complete the SPI transaction
      HAL_Delay(9);
    }
  }
  /* USER CODE END StartrSpi03MagTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

