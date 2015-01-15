#include <LiquidCrystal.h>
#include "PhotoLineDetection.h"

// XXX Change pins!!
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);
PhotoLineDetection lineDet(A0);

// Initialize calibration buttons
int btnCal;
int btnToggle;

// Color calibration menu FSM
// 0 - white
// 1 - red
// 2 - black
// 3 - calibrated
volatile int cal_color = 0;
unsigned long timestamp = 0;
bool override = false;

void display()
{	
	// Reset timer
	timestamp = millis();
	
	if (override)
	{
		override = false;
		return;
	}

	lcd.clear();
	lcd.print(lineDet.checkSensor());
	lcd.setCursor(0,2);

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
}

void toggle_btn_event()
{
	// Toggles the FSM, advances calibration
	if (digitalRead(btnToggle) == HIGH)
	{
		if (cal_color != 3)
			++cal_color;
		else
			cal_color = 0;

		display();
	}
}

void cal_btn_event()
{
	// Calibrate
	if (digitalRead(btnCal) == HIGH && cal_color != 3)
	{
		override = true;
		lineDet.calibrate(cal_color);
		lcd.setCursor(0,2);
		lcd.println("CALIBRATED");
		
		if (cal_color == BLACK)
			cal_color = CALIBRATED;
		
		display();
	}
}

void setup()
{
	//btnCal.pin = 6;
	btnCal = 2;

	//btnToggle.pin = 7;
	btnToggle = 3;

	pinMode(btnCal, INPUT);
	pinMode(btnToggle, INPUT);
	// Initialize LCD
	lcd.begin(16,2);
}

void loop()
{
	if (millis() - timestamp > 500) display();

	attachInterrupt(0, cal_btn_event, CHANGE);	
	attachInterrupt(1, toggle_btn_event, CHANGE);	
}
