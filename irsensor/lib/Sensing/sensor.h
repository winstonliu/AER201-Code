/*
 * sensor.h 
 * C.W. Liu, February 15, 2015.
 * All rights reserved.
*/
#ifndef sensor_h
#define sensor_h

#include "Arduino.h"

// Define color codes for calibration
#define WHITE 0
#define BLACK 1
#define RED 2
#define UNKNOWN 3

class Sensor
{
	// Abstract base class for sensing
	protected:
		int sensorVal;
		int sensorPin;
		int thresh[3];
	public:
		Sensor(int pin);
		int readSensor();
		void setThresh(int* values);
		void calibrate(int color);
		virtual int detect();

};

#endif // sensor_h
