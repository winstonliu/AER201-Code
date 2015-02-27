/*
 * PhotoLineDetection.h - library for line detecion using Photoresistors.
 * C.W. Liu, January 11, 2015.
 * All rights reserved.
*/
#ifndef PhotoLineDetection_h
#define PhotoLineDetection_h

#include "Arduino.h"

// Define color codes for calibration
#define WHITE 0
#define BLACK 1
#define RED 2
#define CALIBRATED 3

struct PhotoThresholdValues
{
	int white;
	int red;
	int black;
};

class PhotoLineDetection
{
	private:
		int sensor_thresh;
		int sensor_value;
		int sensorPin;
		PhotoThresholdValues thresh;
	public:
		PhotoLineDetection(int);		
		int checkSensor();
		void calibrate(int);	
		// Calibrate color, where
		// 0 - white
		// 1 - red
		// 2 - black
		int detect();

};

#endif
