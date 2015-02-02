#define RES_VAL A0

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

void setup()
{
    pinMode(RES_VAL, INPUT);
	lcd.begin(16, 2);
}

void loop()
{
	lcd.print(analogRead(RES_VAL));
	delay(500);
	lcd.clear();
}
