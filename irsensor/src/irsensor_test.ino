#include "irsensor.h"

const int SP_1 = A0;
const int SP_2 = A1;
const int SP_3 = A2;

unsigned int lap_time = 0;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	if ((millis() - lap_time) > 500)
	{
		display();	
		lap_time = millis();
	}	
}

void display()
{
	Serial.print(analogRead(SP_1));
	Serial.print(" ");
	Serial.print(analogRead(SP_2));
	Serial.print(" ");
	Serial.println(analogRead(SP_3));

}
