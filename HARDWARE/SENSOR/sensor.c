#include "sys.h"
#include "sensor.h"
void init_sensor(SENSOR *sensor){//初始化传感器结构体
  sensor->if_open_sensor=0xf0;//初始开
	sensor->sensor_state=1;//初始开
}

