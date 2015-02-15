#include "sensor.h"

Sensor::Sensor(int sv)
{
	pinMode(sv, INPUT);
	sensorPin = sv;
	sensorVal = 0;
}

int Sensor::read()
{
	sensorVal = analogRead(sensorPin);
	return sensorVal;
}

void Sensor::calibrate(int cc)
{
	switch (cc)
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
