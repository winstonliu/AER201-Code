#include "Arduino.h"
#include "PhotoLineDetection.h"

PhotoLineDetection::PhotoLineDetection(int newSensorPin)
{
	// Initialize values and set sensor pin mode
	pinMode(newSensorPin, INPUT);
	sensorPin = newSensorPin;
	sensor_value = 0;
}

int PhotoLineDetection::checkSensor()
{
	sensor_value = analogRead(sensorPin);
	return sensor_value;
}
