#include <Wire.h>
#include "irsensor.h"
#include "motor.h"
#include "pid.h"
#include "line_pid.h"
#include "rgb_lcd.h"

// Initialize lcd 
rgb_lcd lcd;

// Initialize irsensors
IRSensor irsen1(A0);
IRSensor irsen2(A1);
IRSensor irsen3(A2);
IRSensor irsenL(A3);
IRSensor irsenR(A4);

// Initialize motors (en, dir)
Motor port(3,2);
Motor starboard(5,4);
const int btnCalibrate = 6;
int btnCal_state;

// Tasks
bool rotate = false;

// PID values
const int target_heading = 0;
int current_heading = 0;
int motor_pwm = 0;

// PID Control initialize
PID motor_ctrl(current_heading, target_heading, motor_pwm);

// Timing loops
unsigned int display_lap = 0;
unsigned int poll_lap = 0;

// White, black, red
int threshold_values[3] = {530, 777, 0};

void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);

	pinMode(btnCalibrate, INPUT);
	btnCal_state = digitalRead(btnCalibrate);

	// Set values
	irsen1.setThresh(threshold_values);
	irsen2.setThresh(threshold_values);
	irsen3.setThresh(threshold_values);

	// PID Control
	motor_ctrl.start();		
	motor_ctrl.set_cycle(50);		
	motor_ctrl.tune(2.0, 0.5, 0.5);		
}

void loop()
{
	// Display event	
	if ((millis() - display_lap) > 500)
	{
		display();
		display_lap = millis();
	}	

	// Check for button press
	if (btnCal_state == LOW && digitalRead(btnCalibrate) == HIGH)
	{
		calibrate_all();	
		lcd.clear();
		lcd.print("Calibrated");
	}
	btnCal_state = digitalRead(btnCalibrate);

	if ((millis() - poll_lap) > 20)
	{
		// Read sensors
		irsen1.readSensor();		
		irsen2.readSensor();		
		irsen3.readSensor();		

		// Update heading
		current_heading = mapLinePid(
			irsen1.detect(),
			irsen2.detect(),
			irsen3.detect()
		);

		poll_lap = millis();
	}
		
	// Toggles every 50 ms
	if (motor_ctrl.compute() == true && rotate == false)
	{
		// Motor driving code
		if (current_heading > 0)
		{
			starboard.right(motor_pwm);	
			port.left(255);	
		}
		else if (current_heading < 0)
		{
			starboard.right(255);
			port.left(motor_pwm);
		}
		else
		{
			starboard.right(255);
			port.left(255);
		}
	}
	else
	{
		// Rotate
				
	}
}

void display()
{
	Serial.print("| ");
	Serial.print(irsen1.getValue());
	Serial.print(" ");
	Serial.print(irsen1.detect());
	Serial.print(" | ");
	Serial.print(irsen2.getValue());
	Serial.print(" ");
	Serial.print(irsen2.detect());
	Serial.print(" | ");
	Serial.print(irsen3.getValue());
	Serial.print(" ");
	Serial.print(irsen3.detect());
	Serial.println(" |");
	Serial.print("Current heading: ");
	Serial.println(current_heading);
}

void display_lcd()
{
	lcd.clear();
	lcd.print("|");	
	lcd.print(irsen1.getValue());	
	lcd.setCursor(0,5);
	lcd.print("|");	
	lcd.print(irsen2.getValue());	
	lcd.setCursor(0,9);
	lcd.print("|");	
	lcd.print(irsen3.getValue());	
	lcd.print("|");	
	lcd.setCursor(1,0);
	lcd.print("|");	
	lcd.print(irsen1.detect());	
	lcd.print("|");	
	lcd.print(irsen2.detect());	
	lcd.print("|");	
	lcd.print(irsen3.detect());	
	lcd.print("||");	
	lcd.print(current_heading);	
	lcd.print("|");	
}

void calibrate_all()
{
	int white1, white2;
	// Middle sensor calibrates for black
	// Right and left sensor averages calibrate for white
	threshold_values[BLACK] = irsen2.readSensor();	
	threshold_values[WHITE] = (irsen1.readSensor() + irsen3.readSensor()) / 2;

	// Set values
	irsen1.setThresh(threshold_values);
	irsen2.setThresh(threshold_values);
	irsen3.setThresh(threshold_values);
	irsenL.setThresh(threshold_values);
	irsenR.setThresh(threshold_values);
}
