#ifndef irsensor_h
#define irsensor_h

#include "sensor.h"

class IRSensor: public Sensor
{
	public:
		IRSensor(int);
		int detect();
};

#endif
