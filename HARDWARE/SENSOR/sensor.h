#include "sys.h"
//传感器控制结构体
typedef struct sensor {
	u8 if_open_sensor;
  u8 sensor_state;
}SENSOR;
void init_sensor(SENSOR *sensor);//初始化传感器结构体
