#include <Wire.h>
#include <rgb_lcd.h>

#include <irsensor.h>
#include <motor.h>
#include <nav.h>

#include "drivemotor.h"

// ================================================================ //
// ADJUSTABLE PARAMETERS

const int btnCalibrate = 10;
const int NUMPINS = 4; // Initialize irsensors
const int senPins[NUMPINS] = {A0,A1,A2,A3}; 
// Left: A0, middle: A1, right A2

// Initialize nav x,y,d
grid start_pos(4, 1, 0);
grid end_pos(6, 5, 90);

int threshold_values[3] = {0, 800, 0};

// Heading motor proportional scaling factor:
const int dmotor_scaling = 50; 
// Initial change
const int dmotor_initial = 2;

// ================================================================ //

// Flags
bool FLAG_NAVERR = false;
bool FLAG_DONE = false;

rgb_lcd lcd;
nav Navigator(start_pos);
IRSensor irsen[NUMPINS];

// ================================================================ //
// MOTOR initialization

// Initialize motors (en, dir)
motor port(3,4);
motor starboard(5,6);
motor wheel(9,8);

int current_heading;

DriveMotor Driver(port, starboard, dmotor_scaling, dmotor_initial);

// ================================================================ //

// Listener functions (folded)
void displayFunction()
{
	grid temp_grid = Navigator.currentGrid;
/*
	Serial.print(irsen[1].getValue());
	Serial.print(" ");
	Serial.print(irsen[1].detect());
	Serial.print(" | ");
	Serial.print(irsen[2].getValue());
	Serial.print(" ");
	Serial.print(irsen[2].detect());
	Serial.print(" | ");
	Serial.print(irsen[3].getValue());
	Serial.print(" ");
	Serial.println(irsen[3].detect());
	Serial.print("Current heading: ");
	Serial.println(current_heading);
	Serial.print(" x: ");
	Serial.print(temp_grid.x);
	Serial.print(" y: ");
	Serial.print(temp_grid.y);
	Serial.print(" d: ");
	Serial.println(temp_grid.d);
	Serial.print("# Tasks: ");
	Serial.println(Navigator.countRemaining());
*/
	/*
	}
	else if (event == EventManager::kEventDisplayLCD)
	{
		lcd.clear();
		lcd.print("|");	
		lcd.print(irsen[1].getValue());	
		lcd.setCursor(0,5);
		lcd.print("|");	
		lcd.print(irsen[2].getValue());	
		lcd.setCursor(0,9);
		lcd.print("|");	
		lcd.print(irsen[3].getValue());	
		lcd.print("|");	
		lcd.setCursor(1,0);
		lcd.print("|");	
		lcd.print(irsen[1].detect());	
		lcd.print("|");	
		lcd.print(irsen[2].detect());	
		lcd.print("|");	
		lcd.print(irsen[3].detect());	
		lcd.print("||");	
		lcd.print(current_heading);	
		lcd.print("|");	
	}
	*/
}
void calibrateFunction()
{
	calibrate_all();
	lcd.clear();
	lcd.print("Calibrated");
}
void sensorPollingFunction()
{
	int sum_lines = 0;
	// Read sensors
	for (int i = 0; i < NUMPINS; ++i) 
	{
		irsen[i].readSensor(); 
		// Perpendicular line detection for past cycles
		sum_lines += irsen[i].pastEncounters();	
	}

	// If all sensors have been triggered in the past n cycles,
	// then trigger line detected
	//Serial.print("Sum pins ");
	//Serial.println(sum_lines);
	if (sum_lines == NUMPINS)
	{
		Navigator.interrupt(LINE_ISR);			
	}

	// Update heading
	current_heading = Driver.mapLine(
		irsen[1].detect(),
		irsen[2].detect(),
		irsen[3].detect()
	);
}
void doneFunction()
{
	//Serial.println("DONE");
	lcd.clear();
	lcd.print("DONE");
	FLAG_DONE = true;
}

void killMotors()
{
	port.stop();
	starboard.stop();
	//wheel.stop();
}

void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);

	//wheel.left(125);	

	// Event handling

	// Pins
	pinMode(btnCalibrate, INPUT);
	port.left();
	starboard.right();

	// DEBUG
	Navigator.currentAction = MOVEFORWARD;

	// Set threshold values for irsensor
	for (int i = 0; i < NUMPINS; ++i)
	{
		// Make new IRSensor
		irsen[i] = IRSensor(senPins[i]);
		irsen[i].setThresh(threshold_values);
	}

/*
	// Check for navigation error
	int ret_err = Navigator.computeRectilinearPath(end_pos);
	Serial.print("Computation result: ");
	Serial.println(ret_err);
	if (ret_err < 0)
	{
		grid temp_grid = Navigator.getDestination(); 

		// DEBUG
		Serial.print(" x: ");
		Serial.print(temp_grid.x);
		Serial.print(" y: ");
		Serial.print(temp_grid.y);
		Serial.print(" d: ");
		Serial.println(temp_grid.d);
		Serial.print("# Tasks: ");
		Serial.println(Navigator.countRemaining());

		Serial.println("NAV ERROR");
		lcd.clear();
		lcd.print("NAV ERROR");
		FLAG_NAVERR = true;
	}
*/
}

void loop()
{
	static int main_lap = 0;
	
	/*
	if (FLAG_NAVERR == true || FLAG_DONE == true)
		return;

	// Check if there are any tasks left to do
	if ((millis() - main_lap) > 20)
	{
		// DEBUG
		bool temp_ret = Navigator.doneTasks();

		Serial.print("Is done: ");
		Serial.println(temp_ret);
		if (temp_ret == true)
		{
			doneFunction();
		}
		else if (Navigator.getAction() == IDLE)
		{
			Navigator.startTask();
		}
		else if (Navigator.checkTaskComplete() == 0)
		{
			grid temp_grid = Navigator.taskdestination;
			Serial.print("Task destination: ");
			Serial.print(" x: ");
			Serial.print(temp_grid.x);
			Serial.print(" y: ");
			Serial.print(temp_grid.y);
			Serial.print(" d: ");
			Serial.println(temp_grid.d);
			Serial.println("Task Complete");
			killMotors();		
		}	
		Serial.print("Current action: ");
		Serial.println(Navigator.getAction());
	}
	*/

	// Event manager processing
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
	static unsigned int pauseCounter = 0;

	// Display event
	if ((millis() - display_lap) > 200)
	{
		// DEBUG
		lcd.clear();
		lcd.print(irsen[1].readSensor());
		lcd.print(" ");
		lcd.print(irsen[2].readSensor());
		lcd.print(" ");
		lcd.print(irsen[3].readSensor());
		lcd.setCursor(0,1);
		lcd.print(irsen[1].detect());
		lcd.print(" ");
		lcd.print(irsen[2].detect());
		lcd.print(" ");
		lcd.print(irsen[3].detect());
		lcd.print(" ");
		lcd.print(current_heading);
		display_lap = millis();
	}

	// Poll sensors
	if ((millis() - poll_lap) > 20)
	{
		// DEBUG
		Serial.println("Sensor Poll");
		sensorPollingFunction();	
		poll_lap = millis();
		Serial.print("White ");
		Serial.println(threshold_values[WHITE]);
		Serial.print("Black ");
		Serial.println(threshold_values[BLACK]);
	}

	if (Navigator.getAction() == PAUSE)
	{
		if (pauseCounter > 100)
			Navigator.FLAG_unpause = true;	
		//Serial.println(pauseCounter);
		++pauseCounter;
	}
	else if (Navigator.getAction() == MOVEFORWARD)	
	{
		int adjustSpeed;
		// DEBUG
		//Serial.println("Moving Forward");
		Serial.print("Adjust to Speed: ");
		
		// TODO Toggles every 50 ms
		adjustSpeed = Driver.lineMotorScaling();
		Serial.println(adjustSpeed);
	}
	else if (Navigator.getAction() == ROTATETO)
	{
		// DEBUG
		//Serial.println("Rotating");

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
		calibrateFunction();
	}
	btnCal_state = calRead;
}

void calibrate_all()
{
	// Middle sensor calibrates for black
	// Right and left sensor averages calibrate for white
	//threshold_values[BLACK] = irsen[2].readSensor();	
	threshold_values[WHITE] = ( irsen[1].readSensor() 
								+ irsen[3].readSensor()) / 2;

	// Set values
	for (int i = 0; i < NUMPINS; ++i)
	{
		irsen[i].setThresh(threshold_values);
	}
}
