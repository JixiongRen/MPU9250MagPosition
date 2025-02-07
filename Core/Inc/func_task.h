//
// Created by renji on 25-2-7.
//

#ifndef FUNC_TASK_H
#define FUNC_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void StartGetAccelDataTask(void *argument);
void StartGetGyroDataTask(void *argument);
void StartGetMagnDataTask(void *argument);
void StartUsartTransTask(void *argument);

#endif //FUNC_TASK_H
