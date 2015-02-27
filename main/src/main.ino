#include <Wire.h>
#include "irsensor.h"
#include "motor.h"
#include "pid.h"
#include "line_pid.h"
#include "rgb_lcd.h"
#include "nav.h"

// Initialize lcd 
rgb_lcd lcd;

// Initialize nav
grid start_pos;
start_pos.x = 4;
start_pos.y = 1;
start_pos.d = 0;

grid end_pos;
end_pos.x = 6;
end_pos.y = 5;
end_pos.x = ;

nav Navigator(start_pos);

// Initialize irsensors
const int NUMPINS = 5;
int senPins [NUMPINS] = {A0,A1,A2,A3,A4};
IRSensor irsen[NUMPINS];

// Initialize motors (en, dir)
motor port(3,2);
motor starboard(5,4);

const int btnCalibrate = 6;
int btnCal_state;

// PID values
const int target_heading = 0;
int current_heading = 0;
int motor_pwm = 0;

// PID Control initialize
PID motor_ctrl(current_heading, target_heading, motor_pwm);

// Timing loops XXX
unsigned int display_lap = 0;
unsigned int poll_lap = 0;
unsigned int rot_lap = 0;

// White, black, red
int threshold_values[3] = {530, 777, 0};

void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);

	pinMode(btnCalibrate, INPUT);
	btnCal_state = digitalRead(btnCalibrate);

	// Set threshold values for irsensor
	for (int i = 0; i < NUMPINS; ++i)
	{
		// Initialize irsensors
		irsen[i](senPins(i));
		irsen[i].setThresh(threshold_values);
	}

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
		for (int i = 0; i < NUMPINS; ++i) 
		{
			irsen[i].readSensor(); 
		}

		// Update heading
		current_heading = mapLinePid(
			irsen[1].detect(),
			irsen[2].detect(),
			irsen[3].detect()
		);

		poll_lap = millis();
	}
		
	// Toggles every 50 ms
	if (motor_ctrl.compute() == true)
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

	if (rotate == true)
	{
		if ((millis() - rot_lap) > 50)
		{
			// Rotate will trigger when irsenL and irsenR
			// Turn left
			if (starboard.get_status() != MOTOR_RIGHT && port.get_status() != MOTOR_RIGHT)
			{
				starboard.right(255);
				port.right(255);
			}
			
			if (irsenLval == LOW && irsenLval == HIGH)
				irsenLval = HIGH;
			if (irsenRval == LOW && irsenRval == HIGH)
				irsenRval = HIGH;

			if (irsenLval == HIGH && irsenRval == HIGH)
				rotate = false;

			rot_lap = millis();
		}
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
	// Middle sensor calibrates for black
	// Right and left sensor averages calibrate for white
	threshold_values[BLACK] = irsen2.readSensor();	
	threshold_values[WHITE] = (irsen1.readSensor() + irsen3.readSensor()) / 2;

	// Set values
	for (int i = 0; i < NUMPINS; ++i)
	{
		irsen[i].setThresh(threshold_values);
	}
}
