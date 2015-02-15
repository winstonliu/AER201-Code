#include "irsensor.h"

int IRSensor::detect()
{
	int wdiff = pow(sensorVal - thresh[WHITE], 2);
	int bdiff = pow(sensorVal - thresh[BLACK], 2);

	else if (bdiff < wdiff)
		return BLACK;
	else if (wdiff < bdiff)
		return WHITE;
	else
		return UNKNOWN;

}
