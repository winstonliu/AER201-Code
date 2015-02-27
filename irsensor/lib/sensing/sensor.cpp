#include "sensor.h"

Sensor::Sensor(int pin) 
{
	pinMode(pin, INPUT);
	sensorPin = pin;
	sensorVal = 0;
}

int Sensor::readSensor()
{
	sensorVal = analogRead(sensorPin);
	return sensorVal;
}

int Sensor::getValue()
{
	return sensorVal;
}

void Sensor::setThresh(int* values)
{
	for (int i = 0; i < sizeof(thresh)/sizeof(int); ++i)
	{
		thresh[i] = values[i];
	}
}

void Sensor::calibrate(int color)
{
	switch (color)
	{
		case WHITE:
			thresh[WHITE] = sensorVal;	
			break;
		case BLACK:
			thresh[BLACK] = sensorVal;
			break;
		case RED:
			thresh[RED] = sensorVal;
			break;
	}
}

int Sensor::detect()
{
	//XXX Placeholder
	return 3;
}
