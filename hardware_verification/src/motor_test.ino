#include "motor.h"
#include "drivemotor.h"

const int NUMPINS = 4;
const int senPins[NUMPINS] = {A13,A14,A15,A12}; // l,m,r,offset

motor starboard(8,9);
motor port(10,11);

const int dmotor_scaling = 50; 
const int dmotor_initial = 2;

DriveMotor Driver(port, starboard, dmotor_scaling, dmotor_initial);

void digL()
{
	Serial.println("LEFT");
}

void digR()
{
	Serial.println("RIGHT");
}

void setup()
{
	//attachInterrupt(5, digL, RISING);
	//attachInterrupt(4, digR, RISING);
	Serial.begin(9600);
	starboard.left(140);
	port.right(140);
}

void loop()
{	
	delay(50);
	Serial.print(analogRead(4));
	Serial.print(" ");
	Serial.println(analogRead(5));
	/*
	Serial.print("Sval: ");
	Serial.print(starboard.get_status());
	Serial.print(" Pval: ");
	Serial.println(port.get_status());
	Serial.print("Sensor values: ");
	Serial.print(analogRead(A13));
	Serial.print(" ");
	Serial.print(analogRead(A14));
	Serial.print(" ");
	Serial.print(analogRead(A15));
	Serial.print(" ");
	Serial.println(analogRead(A12));
	*/
}
