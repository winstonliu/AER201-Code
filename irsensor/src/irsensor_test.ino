#include "irsensor.h"
#include "motor.h"
#include "line_pid.h"

// Initialize irsensors
IRSensor irsen1(A0);
IRSensor irsen2(A1);
IRSensor irsen3(A2);

// Initialize motors
Motor port(4,5);
Motor starboard(6,7);

// PID values
const int target_heading = 0;
int current_heading = 0;
int motor_pwm = 0;

// PID Control initialize
PID<int> motor_ctrl(current_heading, target_heading, motor_pwm);

// Timing loops
unsigned int display_lap = 0;
unsigned int poll_lap = 0;

// White, black, red
int threshold_values[3] = {530, 777, 0};

void setup()
{
	Serial.begin(9600);
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
	if ((millis() - display_lap) > 500)
	{
		display();
		display_lap = millis();
	}	

	if ((millis() - poll_lap) > 20)
	{
		// Read sensors
		irsen1.readSensor();		
		irsen2.readSensor();		
		irsen3.readSensor();		

		// Update heading
		current_heading = mapLinePid(
			(irsen1.detect() > 0) ? true : false,
			(irsen2.detect() > 0) ? true : false,
			(irsen3.detect() > 0) ? true : false
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
			port.left();	
		}
		else if (current_heading < 0)
		{
			starboard.right();
			port.left(motor_pwm);
		}
		else
		{
			starboard.right();
			port.left();
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
