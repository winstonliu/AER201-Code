#include <Wire.h>
#include <Keypad.h>
#include <rgb_lcd.h>

rgb_lcd lcd;

const byte rows = 4;
const byte cols = 3;

char keys[rows][cols] = {
 	{'1','2','3'},
	{'4','5','6'},
	{'7','8','9'},
	{'*','0','#'}
};
char bsnumbers[3];

//connect to the row pinouts of the keypad
byte rowPins[rows] = {13, 12, 11, 10}; 

//connect to the column pinouts of the keypad
byte colPins[cols] = {9, 8, 7}; 

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);
void setup()
{
	lcd.begin(16,2);
	lcd.print("Hello cruel world.");
}

void loop()
{
	static int counter = 0;
	char key = keypad.getKey();
	bsnumbers[counter] = key; 
	
	if (key != NO_KEY)
	{
		lcd.print(key);
		if (key == '*')
		{
			lcd.clear();
			lcd.print("X: ");
			lcd.print(bsnumbers[0]);
			lcd.print("y: ");
			lcd.print(bsnumbers[1]);
			lcd.print("Z: ");
			lcd.print(bsnumbers[2]);
		}
	}
}
