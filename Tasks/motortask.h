#ifndef __MOTORTASK_H
#define __MOTORTASK_H

#include "canrecive.h"
#include "can.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "FreeRTOSConfig.h"
#include "math.h"
#include "angle_compute.h"
#include "main.h"                  // Device header

#define translate 1
#define spin 2
#define spin_translate 3

extern void task1_task(void *pvParameters);
void chassis_speed(RC_ctrl_t* RC);
void angle_mapping_direction(RC_ctrl_t* RC);
void motor_pid_compute(void);

#endif


