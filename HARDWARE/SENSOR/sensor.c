#include "sys.h"
#include "sensor.h"
void init_sensor(SENSOR *sensor){//��ʼ���������ṹ��
  sensor->if_open_sensor=0xf0;//��ʼ��
	sensor->sensor_state=1;//��ʼ��
}

