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
EventManager eVM();

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

// White, black, red
int threshold_values[3] = {530, 777, 0};

// Listener functions (folded)
void displayFunction(int event, int param)
{
	if (event == EventManager::kEventDisplaySerial)
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
	else if (event == EventManager::kEventDisplayLCD)
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
}
void calibrateFunction(int event, int param)
{
	calibrate_all();
	lcd.clear();
	lcd.print("Calibrated");
}
void sensorPollingFunction(int event, int param)
{
	int sum_lines = 0;
	// Read sensors
	for (int i = 0; i < senPins.size(); ++i) 
	{
		irsen[i].readSensor(); 
		// Perpendicular line detection for past cycles
		sum_lines += irsen[i].pastEncounters();	
	}

	// If all sensors have been triggered in the past n cycles,
	// then trigger line detected
	if (sum_lines == senPins.size())
	{
		Navigator.interrupt(nav::LINE_ISR);			
	}

	// Update heading
	current_heading = mapLinePid(
		irsen[1].detect(),
		irsen[2].detect(),
		irsen[3].detect()
	);
} // end fold

// Call listeners
GenericCallable<void(int,int)> display(displayFunction);
GenericCallable<void(int,int)> calibrate(calibrateFunction);
GenericCallable<void(int,int)> sensorPolling(calibrateFunction);

void killMotors()
{
	port.stop();
	starboard.stop();
}

void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);

	// Event handling
	eVM.addListener( EventManager::kEventDisplaySerial, &display);
	eVM.addListener( EventManager::kEventDisplayLCD, &display);
	eVM.addListener( EventManager::kEventCalibrate, &calibrate);
	eVM.addListener( EventManager::kEventSensorPolling, &sensorPolling);

	// Pins
	pinMode(btnCalibrate, INPUT);

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
		// Make new IRSensor and push onto vector
		irsen.push_back(IRSensor(senPins(i)));
		irsen[i].setThresh(threshold_values);
	}

	// Check for navigation error
	if (Navigator.computeRectilinearPath(end_pos) == -1);
	{
		Serial.println("NAV ERROR");
		lcd.clear();
		lcd.print("NAV ERROR");
	}
}

void loop()
{
	// Check if there are any tasks left to do
	if (Navigator.tasksLeft() == 0)
	{
		Serial.println("DONE");
		lcd.clear();
		lcd.print("DONE");
	}
	else if (Navigator.getAction() == nav::IDLE)
	{
		Navigator.startTask();
	}
	else if (Navigator.checkTaskComplete() == 0)
	{
		killMotors();		
	}	
	
	// Event manager processing
	eVM.processEvent();
	addEvents();

}

void addEvents()
{
	// Button state declarations
	static int btnCal_state = digitalRead(btnCalibrate);
	// Timing loops 
	static unsigned int display_lap = 0;
	static unsigned int poll_lap = 0;
	static unsigned int rot_lap = 0;

	// Display event
	if ((millis() - display_lap) > 500)
	{
		eVM.queueEvent(EventManager::kEventDisplayLCD, 0);
		display_lap = millis();
	}

	// Poll sensors
	if ((millis() - poll_lap) > 20)
	{
		eVM.queueEvent(EventManager::kEventSensorPolling, 0);
		poll_lap = millis();
	}

	if (Navigator.getAction() == nav::MOVEFORWARD)	
	{
		// TODO Toggles every 50 ms
		if (motoPID[0].compute() == true)
			port.adjustSpeed(motor_pwm);	
		if (motoPID[1].compute() == true)
			starboard.adjustSpeed(motor_pwm);	
	}
	else if (Navigator.getAction() == nav::ROTATETO)
	{
		if ((millis() - rot_lap) > 50)
		{
			// Rotate will trigger when irsenL and irsenR
			// Turn left
			if (starboard.get_status() != MOTOR_RIGHT 
				&& port.get_status() != MOTOR_RIGHT)
			{
				starboard.right(255);
				port.right(255);
			}
			rot_lap = millis();
		}
	}

	// Check for button press
	int calRead = digitalRead(btnCalibrate);
	if (btnCal_state == LOW && calRead == HIGH)
	{
		eVM.queueEvent(EventManager::kEventCalibrate, 0)
	}
	btnCal_state = calRead;
}

void calibrate_all()
{
	// Middle sensor calibrates for black
	// Right and left sensor averages calibrate for white
	threshold_values[BLACK] = irsen2.readSensor();	
	threshold_values[WHITE] = ( irsen1.readSensor() 
								+ irsen3.readSensor()) / 2;

	// Set values
	for (int i = 0; i < NUMPINS; ++i)
	{
		irsen[i].setThresh(threshold_values);
	}
}
