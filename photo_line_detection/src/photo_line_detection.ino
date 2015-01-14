#include <LiquidCrystal.h>
#include "PhotoLineDetection.h"

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
PhotoLineDetection lineDet(A0);

struct button
{
	int pin;
	int state;
};

// Initialize calibration buttons
button btnCal;
button btnToggle;

// Color calibration menu
// 0 - white
// 1 - red
// 2 - black
// 3 - calibrated
int cal_color = 0;

void setup()
{
	btnCal.pin = 6;
	btnCal.state = 0;

	btnToggle.pin = 7;
	btnToggle.state = 0;

	pinMode(btnCal.pin, INPUT);
	pinMode(btnToggle.pin, INPUT);
	// Initialize LCD
	lcd.begin(16,2);
}

void loop()
{
	btnToggle.state = digitalRead(btnToggle.pin);
	btnCal.state = digitalRead(btnCal.pin);

	// Print sensor values
	lcd.print(lineDet.checkSensor());
	lcd.setCursor(0,2);

	if (btnToggle.state == HIGH)
	{
		if (cal_color != 3)
			++cal_color;
		else
			cal_color = 0;
	}

	switch(cal_color)
	{
		case WHITE:
			lcd.print("SET WHITE");
			break;
		case RED:
			lcd.print("SET RED");
			break;
		case BLACK:
			lcd.print("SET BLACK");
			break;
		case CALIBRATED:
			lcd.print(lineDet.detect());
			break;
		default:
			lcd.print("HE'S DEAD JIM"); 
	}

	if (btnCal.state == HIGH && cal_color != 3)
	{
		lineDet.calibrate(cal_color);
		lcd.print("CALIBRATED");
		
		if (cal_color == BLACK)
			cal_color = CALIBRATED;
	}

	delay(500);
	lcd.clear();
}
