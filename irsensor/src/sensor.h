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

class Sensor
{
	// Abstract base class for sensing
	protected:
		int sensorVal;
		int sensorPin;
		int thresh[3];
	public:
		Sensor(int);
		int read();
		void calibrate(int);	
		virtual int detect() =0;

};

#endif // sensor_h
