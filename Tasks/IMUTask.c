#include "main.h"

IMU_Data_t IMU_Data;
float yaw = 0;
float pitch =0;
float roll = 0;

void IMU_task(void  *pvParameters)
{
	IMU_Get_Accel_Cali_Data(&IMU_Data);
	while(1)
	{
		
		IMU_Get_Data(&IMU_Data,&pitch,&roll,&yaw);
		vTaskDelay(1);
	}
	


}


