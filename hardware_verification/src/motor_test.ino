#include "motor.h"

const int NUMPINS = 4;
const int senPins[NUMPINS] = {A13,A14,A15,A12}; // l,m,r,offset

motor port(8,9);
motor starboard(10,11);

void setup()
{
	port.right();
	starboard.left();
	Serial.begin(9600);
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
