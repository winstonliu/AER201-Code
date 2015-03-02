#include <Arduino.h>
#include <Wire.h>
#include <Keypad.h>
#include <rgb_lcd.h>
void setup();
void loop();
#line 1 "src/keypad.ino"
//#include <Wire.h>
//#include <Keypad.h>
//#include <rgb_lcd.h>

rgb_lcd lcd;

const byte rows = 4;
const byte cols = 3;

char keys[rows][cols] = {
 	{'1','2','3'},
	{'4','5','6'},
	{'7','8','9'},
	{'*','0','#'}
};
//connect to the row pinouts of the keypad
byte rowPins[rows] = {13, 12, 11, 10}; 

//connect to the column pinouts of the keypad
byte colPins[cols] = {9, 8, 7}; 

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);
void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);
	lcd.print("Hello cruel world.");
}

void loop()
{
	char key = keypad.getKey();
	
	if (key != NO_KEY)
	{
		Serial.print(key);
		lcd.print(key);
		if (key == '*')
		{
			lcd.clear();
		}
	}
}
