/*
 * PhotoLineDetection.h - library for line detecion using Photoresistors.
 * C.W. Liu, January 11, 2015.
 * All rights reserved.
*/
#ifndef PhotoLineDetection_h
#define PhotoLineDetection_h

#include "Arduino.h"

class PhotoLineDetection
{
	private:
		int sensor_value;
		int sensorPin;
	public:
		PhotoLineDetection(int);		
		int checkSensor();
};

#endif
