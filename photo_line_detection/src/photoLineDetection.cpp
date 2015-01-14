#include "Arduino.h"
#include "PhotoLineDetection.h"

PhotoLineDetection::PhotoLineDetection(int newSensorPin)
{
	// Initialize values and set sensor pin mode
	sensor_thresh = 15;
	pinMode(newSensorPin, INPUT);
	sensorPin = newSensorPin;
	sensor_value = 0;
}

int PhotoLineDetection::checkSensor()
{
	sensor_value = analogRead(sensorPin);
	return sensor_value;
}

void PhotoLineDetection::calibrate(int calibrate_color)
{
	switch (calibrate_color)
	{
		case WHITE:
			thresh.white = sensor_value;	
			break;
		case RED:
			thresh.red = sensor_value;
			break;
		case BLACK:
			thresh.black = sensor_value;
			break;
	}
}

int PhotoLineDetection::detect()
{
	if (pow(sensor_value - thresh.white, 2) <= pow(sensor_thresh, 2))
		return WHITE;
	else if (pow(sensor_value - thresh.red, 2) <= pow(sensor_thresh, 2))
		return RED;
	else if (pow(sensor_value - thresh.black, 2) <= pow(sensor_thresh, 2))
		return BLACK;
	else 
		return CALIBRATED;
}
