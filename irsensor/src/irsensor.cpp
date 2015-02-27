#include "irsensor.h"

IRSensor::IRSensor(int sv):Sensor(sv) {};

int IRSensor::detect()
{
	//int wdiff = pow(sensorVal - thresh[WHITE], 2);
	//int bdiff = pow(sensorVal - thresh[BLACK], 2);
	
	int wdiff = pow(sensorVal - thresh[WHITE], 2);
	int bdiff = pow(sensorVal - thresh[BLACK], 2);

	if (bdiff < 500)
		return BLACK;
	else
		return WHITE;
}
