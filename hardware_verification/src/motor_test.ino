#include "motor.h"
#include "drivemotor.h"

const int NUMPINS = 4;
const int senPins[NUMPINS] = {A13,A14,A15,A12}; // l,m,r,offset

motor starboard(8,9);
motor port(10,11);

const int dmotor_scaling = 50; 
const int dmotor_initial = 2;

volatile int l = 0;
volatile int r = 0;

DriveMotor Driver(port, starboard, dmotor_scaling, dmotor_initial);

void digL()
{
	++l;
}

void digR()
{
	++r;
}

void setup()
{
	attachInterrupt(4, digL, RISING);
	attachInterrupt(5, digR, RISING);
	Serial.begin(9600);
	analogWrite(7, LOW);
	analogWrite(10, 40);
	digitalWrite(11, HIGH);
	analogWrite(8, 40);
	digitalWrite(9, LOW);
	//starboard.left();
	//port.right();
}

void loop()
{	
	delay(500);
	Serial.print(l);
	Serial.print(" ");
	Serial.println(r);
	/*
	Serial.print(analogRead(4));
	Serial.print(" ");
	Serial.println(analogRead(5));
	*/
	/*
	Serial.print("Sval: ");
	Serial.print(starboard.get_status());
	Serial.print(" Pval: ");
	Serial.println(port.get_status());
	Serial.print("Sensor values: ");
	*/
}
