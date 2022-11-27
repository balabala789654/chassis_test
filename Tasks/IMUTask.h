#ifndef _IMUTASK_H_
#define _IMUTASK_H_
#include "MPU6050.h"
#include "IMU.h"


extern IMU_Data_t IMU_Data;
void IMU_task(void *pvParameters);

#endif

