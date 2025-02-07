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
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for GetAccDataTas */
osThreadId_t GetAccDataTasHandle;
uint32_t GetAccelDataTaskBuffer[ 128 ];
osStaticThreadDef_t GetAccelDataTaskControlBlock;
const osThreadAttr_t GetAccDataTas_attributes = {
  .name = "GetAccDataTas",
  .cb_mem = &GetAccelDataTaskControlBlock,
  .cb_size = sizeof(GetAccelDataTaskControlBlock),
  .stack_mem = &GetAccelDataTaskBuffer[0],
  .stack_size = sizeof(GetAccelDataTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GetGyroDataTask */
osThreadId_t GetGyroDataTaskHandle;
uint32_t GetGyroDataTaskBuffer[ 128 ];
osStaticThreadDef_t GetGyroDataTaskControlBlock;
const osThreadAttr_t GetGyroDataTask_attributes = {
  .name = "GetGyroDataTask",
  .cb_mem = &GetGyroDataTaskControlBlock,
  .cb_size = sizeof(GetGyroDataTaskControlBlock),
  .stack_mem = &GetGyroDataTaskBuffer[0],
  .stack_size = sizeof(GetGyroDataTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GetMagnDataTask */
osThreadId_t GetMagnDataTaskHandle;
uint32_t GetMagnDataTaskBuffer[ 128 ];
osStaticThreadDef_t GetMagnDataTaskControlBlock;
const osThreadAttr_t GetMagnDataTask_attributes = {
  .name = "GetMagnDataTask",
  .cb_mem = &GetMagnDataTaskControlBlock,
  .cb_size = sizeof(GetMagnDataTaskControlBlock),
  .stack_mem = &GetMagnDataTaskBuffer[0],
  .stack_size = sizeof(GetMagnDataTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UsartTransTask */
osThreadId_t UsartTransTaskHandle;
uint32_t UsartTransTaskBuffer[ 128 ];
osStaticThreadDef_t UsartTransTaskControlBlock;
const osThreadAttr_t UsartTransTask_attributes = {
  .name = "UsartTransTask",
  .cb_mem = &UsartTransTaskControlBlock,
  .cb_size = sizeof(UsartTransTaskControlBlock),
  .stack_mem = &UsartTransTaskBuffer[0],
  .stack_size = sizeof(UsartTransTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartGetAccelDataTask(void *argument);
void StartGetGyroDataTask(void *argument);
void StartGetMagnDataTask(void *argument);
void StartUsartTransTask(void *argument);

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
  /* creation of GetAccDataTas */
  GetAccDataTasHandle = osThreadNew(StartGetAccelDataTask, NULL, &GetAccDataTas_attributes);

  /* creation of GetGyroDataTask */
  GetGyroDataTaskHandle = osThreadNew(StartGetGyroDataTask, NULL, &GetGyroDataTask_attributes);

  /* creation of GetMagnDataTask */
  GetMagnDataTaskHandle = osThreadNew(StartGetMagnDataTask, NULL, &GetMagnDataTask_attributes);

  /* creation of UsartTransTask */
  UsartTransTaskHandle = osThreadNew(StartUsartTransTask, NULL, &UsartTransTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartGetAccelDataTask */
/**
  * @brief  Function implementing the GetAccelDataTas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGetAccelDataTask */
__weak void StartGetAccelDataTask(void *argument)
{
  /* USER CODE BEGIN StartGetAccelDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGetAccelDataTask */
}

/* USER CODE BEGIN Header_StartGetGyroDataTask */
/**
* @brief Function implementing the GetGyroDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetGyroDataTask */
__weak void StartGetGyroDataTask(void *argument)
{
  /* USER CODE BEGIN StartGetGyroDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGetGyroDataTask */
}

/* USER CODE BEGIN Header_StartGetMagnDataTask */
/**
* @brief Function implementing the GetMagnDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetMagnDataTask */
__weak void StartGetMagnDataTask(void *argument)
{
  /* USER CODE BEGIN StartGetMagnDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGetMagnDataTask */
}

/* USER CODE BEGIN Header_StartUsartTransTask */
/**
* @brief Function implementing the UsartTransTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTransTask */
__weak void StartUsartTransTask(void *argument)
{
  /* USER CODE BEGIN StartUsartTransTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsartTransTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

