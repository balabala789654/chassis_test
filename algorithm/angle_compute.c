#include "angle_compute.h"

float angle = 0;

float angle_com(RC_ctrl_t* RC)
{
	float x = RC->rc.ch[3];
	float y = -RC->rc.ch[2];
	angle = (float)atan2(y,x);
	if(angle<0) angle = 2*PI + angle;
	return angle;
}


float mini_deviation(float now,float last)
{
	static float deviation;

	if((now-last)<=PI&&(now-last)>=(-PI))
		deviation = now - last;
	else if((now-last)>PI)
		deviation = -(2*PI-(now-last));
	else if((now-last)<(-PI))
		deviation = 2*PI+(now-last);
	return deviation;
}

