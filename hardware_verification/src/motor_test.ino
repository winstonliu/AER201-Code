#include "motor.h"
#include "drivemotor.h"

const int NUMPINS = 4;
const int senPins[NUMPINS] = {A13,A14,A15,A12}; // l,m,r,offset

motor starboard(8,9);
motor port(10,11);

const int dmotor_scaling = 50; 
const int dmotor_initial = 2;

DriveMotor Driver(port, starboard, dmotor_scaling, dmotor_initial);

void setup()
{
	Serial.begin(9600);
	Driver.turnRight();
}

void loop()
{	
	delay(50);
	Serial.print("Sensor values: ");
	Serial.print(analogRead(A13));
	Serial.print(" ");
	Serial.print(analogRead(A14));
	Serial.print(" ");
	Serial.print(analogRead(A15));
	Serial.print(" ");
	Serial.println(analogRead(A12));
}
