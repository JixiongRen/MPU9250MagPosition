//
// Created by renji on 25-2-12.
//

#ifndef FREERTOSVARIABLES_H
#define FREERTOSVARIABLES_H
#include "semphr.h"
#include "event_groups.h"

extern osSemaphoreId_t sGetDataStartHandle;

extern osSemaphoreId_t samplingStartTask01Handle;
extern osSemaphoreId_t samplingStartTask02Handle;
extern osSemaphoreId_t samplingStartTask03Handle;

extern osMessageQueueId_t sensorGroupQueue01Handle;
extern osMessageQueueId_t sensorGroupQueue02Handle;
extern osMessageQueueId_t sensorGroupQueue03Handle;

extern osEventFlagsId_t sensorSampleEventHandle;

extern EventGroupHandle_t xSensorEventGroup;

#define SENSOR_GROUP1_BIT (1 << 0)
#define SENSOR_GROUP2_BIT (1 << 1)
#define SENSOR_GROUP3_BIT (1 << 2)

#endif //FREERTOSVARIABLES_H
