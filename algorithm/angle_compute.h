#ifndef __ANGLE_COMPUTE_H
#define __ANGLE_COMPUTE_H

#include "Remote_Control.h"
#include "math.h"
#define PI 3.1415926f
float angle_com(RC_ctrl_t* RC);
float mini_deviation(float now,float last);

#endif

