#include "Arduino.h"
#include "photoLineDetection.h"

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
	int wdiff = pow(sensor_value - thresh.white, 2);
	int rdiff = pow(sensor_value - thresh.red, 2);
	int bdiff = pow(sensor_value - thresh.black, 2);

	if (rdiff < wdiff && rdiff < bdiff)
		return RED;
	else if (bdiff < wdiff && bdiff < rdiff)
		return BLACK;
	else if (wdiff < rdiff && wdiff < bdiff)
		return WHITE;
	else
		return CALIBRATED;
}
