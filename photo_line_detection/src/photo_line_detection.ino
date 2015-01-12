#include <LiquidCrystal.h>
#include "PhotoLineDetection.h"

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
PhotoLineDetection lineDet(A0);

void setup()
{
	// Initialize LCD
	lcd.begin(16,2);
}

void loop()
{
	lcd.print(lineDet.checkSensor());
	delay(500);
	lcd.clear();
}
