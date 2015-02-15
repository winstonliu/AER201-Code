#ifndef irsensor_h
#define irsensor_h

#include "sensor.h"

class IRSensor: public Sensor
{
	public:
		int detect();
};

#endif
