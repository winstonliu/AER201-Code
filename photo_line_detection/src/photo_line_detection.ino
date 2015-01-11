#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int sensor_value = 0;
const int sensorPin = A0;

void setup()
{
  // Initialize LCD
  lcd.begin(16,2);
}

void loop()
{
  sensor_value = analogRead(sensorPin);
  lcd.print(sensor_value);
  delay(500);
  lcd.clear();
}
