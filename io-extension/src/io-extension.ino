#include "Wire.h"

void setup()
{
	Wire.begin();
	Wire.beginTransmission(0x20);
	Wire.write(0x00);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.beginTransmission(0x20);
	Wire.write(0x01);
	Wire.write(0x00);
	Wire.endTransmission();
}

void loop()
{
	delay(1000);
	Wire.beginTransmission(0x20);
	Wire.write(0x13);
	Wire.write(0x01);
	Wire.endTransmission();
	Wire.beginTransmission(0x20);
	Wire.write(0x12);
	Wire.write(0x01);
	Wire.endTransmission();
}
