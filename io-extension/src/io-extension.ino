#include <Wire.h>
#include <LiquidTWI2.h>

LiquidTWI2 lcd(0x20, 1);

void setup()
{
	Serial.begin(9600);
	lcd.setMCPType(LTI_TYPE_MCP23017);
	lcd.begin(16,2);
/*
	Wire.begin();
	Wire.beginTransmission(0x20);
	Wire.write(0x00);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.beginTransmission(0x20);
	Wire.write(0x01);
	Wire.write(0x00);
	Wire.endTransmission();
*/
}

void loop()
{
	lcd.print("Hello, world!");
	Serial.println("Hello, world");
	delay(500);
	lcd.clear();
	delay(500);
/*
	delay(1000);
	Wire.beginTransmission(0x20);
	Wire.write(0x13);
	Wire.write(0x01);
	Wire.endTransmission();
	Wire.beginTransmission(0x20);
	Wire.write(0x12);
	Wire.write(0x01);
	Wire.endTransmission();
*/
}
