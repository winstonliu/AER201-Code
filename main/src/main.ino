#include <Wire.h>
#include "rgb_lcd.h"
#include "EventManager.h"

#include "pid.h"
#include "line_pid.h"
#include "irsensor.h"
#include "motor.h"
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
end_pos.d = 90;

// Initialize Event Handling
EventManager EM();

// Initialize navigation
nav Navigator(start_pos);

// Initialize irsensors
std::vector<int> senPins = {A0,A1,A2,A3,A4};
std::vector<IRSensor> irsen;

// Initialize motors (en, dir)
motor port(3,2);
motor starboard(5,4);

// PID values
const int target_heading = 0;
int current_heading = 0;
int motor_pwm = 0;

// PID Control initialize
const int NUMMOTO = 2;
std::vector<PID> motoPID;	 // port 0, starboard 1

const int btnCalibrate = 6;
int btnCal_state;

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

	// Initialize PID control
	for (int i = 0; i < NUMMOTO; ++i)
	{
		motoPID.push_back(PID(current_heading, target_heading, motor_pwm));
		motoPID[i].start();		
		motoPID[i].set_cycle(50);		
		motoPID[i].tune(2.0, 0.5, 0.5);		
	}

	// Set threshold values for irsensor
	irsen.reserve(senPins.size());
	for (int i = 0; i < senPins.size(); ++i)
	{
		// Push new irsensor onto vector
		irsen.push_back(IRSensor(senPins(i)));
		irsen[i].setThresh(threshold_values);
	}

	// Navigation
	Navigation.computeRectilinearPath(end_pos);
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
		for (int i = 0; i < senPins.size(); ++i) 
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
	if (motoPID[0].compute() == true)
		port.adjustSpeed(motor_pwm);	
	if (motoPID[1].compute() == true)
		starboard.adjustSpeed(motor_pwm);	

	if ((millis() - rot_lap) > 50)
	{
		// Rotate will trigger when irsenL and irsenR
		// Turn left
			rot_lap = millis();
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

void rotate()
{
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
		return;
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
