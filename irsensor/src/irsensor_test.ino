#include "irsensor.h"

IRSensor irsen1(A0);
IRSensor irsen2(A1);
IRSensor irsen3(A2);

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
	Serial.print(irsen1.readSensor());
	Serial.print(" ");
	Serial.print(irsen2.readSensor());
	Serial.print(" ");
	Serial.println(irsen3.readSensor());
}
