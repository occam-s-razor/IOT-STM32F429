#include "sys.h"
//���������ƽṹ��
typedef struct sensor {
	u8 if_open_sensor;
  u8 sensor_state;
}SENSOR;
void init_sensor(SENSOR *sensor);//��ʼ���������ṹ��
