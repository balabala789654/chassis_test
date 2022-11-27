#include "motortask.h"

extern M3508 M3508_control[4];
extern M2006 M2006_control[4];

float new_angle=0;
float last_angle=0;
void task1_task(void *pvParameters)
{

	while(1)
	{
		angle_mapping_direction(&rc_ctrl);
		chassis_speed(&rc_ctrl);
	
		motor_pid_compute();
	
		moter_send_3508(M3508_control[0].M3508_PID[speed_loop].out,
						M3508_control[1].M3508_PID[speed_loop].out,
						M3508_control[2].M3508_PID[speed_loop].out,
						M3508_control[3].M3508_PID[speed_loop].out);

		motor_send_2006(M2006_control[0].M2006_PID[speed_loop].out,
						M2006_control[1].M2006_PID[speed_loop].out,
						M2006_control[2].M2006_PID[speed_loop].out,
						M2006_control[3].M2006_PID[speed_loop].out);
		vTaskDelay(1);
		}
}



void chassis_speed(RC_ctrl_t* RC)
{
	float x;
	float y;
	
	if(RC->rc.ch[2]==0&&RC->rc.ch[3]==0)
	{
		M3508_control[0].pid_set_speed_3508 = 0;
		M3508_control[1].pid_set_speed_3508 = 0;
		M3508_control[2].pid_set_speed_3508 = 0;
		M3508_control[3].pid_set_speed_3508 = 0;
	}
	else 
	{
		x = RC->rc.ch[2]*3000/660;
		y = RC->rc.ch[3]*3000/660;

		M3508_control[0].pid_set_speed_3508 = sqrt(x*x+y*y);
		M3508_control[1].pid_set_speed_3508 = sqrt(x*x+y*y);
		M3508_control[2].pid_set_speed_3508 = sqrt(x*x+y*y);
		M3508_control[3].pid_set_speed_3508 = sqrt(x*x+y*y);

	}
}

void angle_mapping_direction(RC_ctrl_t* RC)
{

	if(RC->rc.ch[2]==0&&RC->rc.ch[3]==0)
	{
		M2006_control[0].pid_set_angle_2006 += mini_deviation(0,new_angle)*1769256/(2*PI);
		M2006_control[1].pid_set_angle_2006 += mini_deviation(0,new_angle)*1769256/(2*PI);
		M2006_control[2].pid_set_angle_2006 += mini_deviation(0,new_angle)*1769256/(2*PI);
		M2006_control[3].pid_set_angle_2006 += mini_deviation(0,new_angle)*1769256/(2*PI);
		new_angle=0;
		last_angle=0;

	}

	else 
	{
		last_angle = new_angle;
		new_angle = angle_com(&rc_ctrl);
		M2006_control[0].pid_set_angle_2006 += mini_deviation(new_angle,last_angle)*1769256/(2*PI);
		M2006_control[1].pid_set_angle_2006 += mini_deviation(new_angle,last_angle)*1769256/(2*PI);
		M2006_control[2].pid_set_angle_2006 += mini_deviation(new_angle,last_angle)*1769256/(2*PI);
		M2006_control[3].pid_set_angle_2006 += mini_deviation(new_angle,last_angle)*1769256/(2*PI);
			

	}
}
	
void motor_pid_compute(void)
{
	PID_calc(&M3508_control[0].M3508_PID[speed_loop], M3508_control[0].M3508.speed_rpm, M3508_control[0].pid_set_speed_3508);
	PID_calc(&M3508_control[1].M3508_PID[speed_loop], M3508_control[1].M3508.speed_rpm, M3508_control[1].pid_set_speed_3508);
	PID_calc(&M3508_control[2].M3508_PID[speed_loop], M3508_control[2].M3508.speed_rpm, M3508_control[2].pid_set_speed_3508);
	PID_calc(&M3508_control[3].M3508_PID[speed_loop], M3508_control[3].M3508.speed_rpm, M3508_control[3].pid_set_speed_3508);

	PID_calc(&M2006_control[0].M2006_PID[angle_loop], M2006_control[0].M2006.all_ecd, M2006_control[0].pid_set_angle_2006);
	PID_calc(&M2006_control[0].M2006_PID[speed_loop], M2006_control[0].M2006.speed_rpm, M2006_control[0].M2006_PID[angle_loop].out);

	PID_calc(&M2006_control[1].M2006_PID[angle_loop], M2006_control[1].M2006.all_ecd, M2006_control[1].pid_set_angle_2006);
	PID_calc(&M2006_control[1].M2006_PID[speed_loop], M2006_control[1].M2006.speed_rpm, M2006_control[1].M2006_PID[angle_loop].out);

	PID_calc(&M2006_control[2].M2006_PID[angle_loop], M2006_control[2].M2006.all_ecd, M2006_control[2].pid_set_angle_2006);
	PID_calc(&M2006_control[2].M2006_PID[speed_loop], M2006_control[2].M2006.speed_rpm, M2006_control[2].M2006_PID[angle_loop].out);

	PID_calc(&M2006_control[3].M2006_PID[angle_loop], M2006_control[3].M2006.all_ecd, M2006_control[3].pid_set_angle_2006);
	PID_calc(&M2006_control[3].M2006_PID[speed_loop], M2006_control[3].M2006.speed_rpm, M2006_control[3].M2006_PID[angle_loop].out);

}

