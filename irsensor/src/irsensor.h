#pragma once

#include "sensor.h"

class IRSensor: public Sensor
{
	public:
		IRSensor(int);
		int detect();
};
