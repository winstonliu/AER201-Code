#include <LiquidCrystal.h>
#include "photoLineDetection.h"

// Wiring
//
// A0: photoresistor + 220 
// A1: photoresistor + 220 
// A2: photoresistor + 220 
//  2: btnCal + 10k pulldown
//  3: btnToggle + 10k pulldown
//  4: LCD 14
//  5: LCD 13 
//  6: LCD 12 
//  7: LCD 11 
// 11: LCD 6
// 12: LCD 4

LiquidCrystal lcd(12, 11, 7, 6, 5, 4);
PhotoLineDetection line_1(A0);
PhotoLineDetection line_2(A1);
PhotoLineDetection line_3(A2);

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
	lcd.print(line_1.checkSensor());
	lcd.setCursor(4,0);
	lcd.print(line_2.checkSensor());
	lcd.setCursor(8,0);
	lcd.print(line_3.checkSensor());
	lcd.setCursor(0,1);

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
			lcd.print(line_1.detect());
			lcd.setCursor(4,1);
			lcd.print(line_2.detect());
			lcd.setCursor(8,1);
			lcd.print(line_3.detect());
			break;
		default:
			lcd.print("HE'S DEAD JIM"); 
	}
}

void advance_cal_color()
{
	if (cal_color != 3)
		++cal_color;
	else
		cal_color = 0;
}

void toggle_btn_event()
{
	// Toggles the FSM, advances calibration
	if (digitalRead(btnToggle) == HIGH)
	{
		advance_cal_color();
		display();
	}
}

void cal_btn_event()
{
	// Calibrate
	if (digitalRead(btnCal) == HIGH && cal_color != 3)
	{
		override = true;
		line_1.calibrate(cal_color);
		line_2.calibrate(cal_color);
		line_3.calibrate(cal_color);
		lcd.setCursor(0,2);
		lcd.print("CALIBRATED");
		
		advance_cal_color();
		display();
	}
}

void setup()
{
	btnCal = 2;
	btnToggle = 3;

	pinMode(btnCal, INPUT);
	pinMode(btnToggle, INPUT);
	// Initialize LCD
	lcd.begin(16,2);
}

void loop()
{
	// Delay 500 ms
	if (millis() - timestamp > 500) display();

	attachInterrupt(0, cal_btn_event, CHANGE);	
	attachInterrupt(1, toggle_btn_event, CHANGE);	
}
