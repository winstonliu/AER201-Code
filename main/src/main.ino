#include <Wire.h>
#include <rgb_lcd.h>

#include <irsensor.h>
#include <motor.h>

#include "nav.h"
#include "drivemotor.h"

// Enable debug messages through serial
#define SERIALDEBUG

#ifdef SERIALDEBUG
#define DEBUG( x ) Serial.print( x )
#else
#define DEBUG( x )
#endif

// ================================================================ //
// ADJUSTABLE PARAMETERS

const int btnCalibrate = 10;
const int NUMPINS = 4; // Initialize irsensors
const int senPins[NUMPINS] = {A0,A1,A2,A3}; 
// Left: A0, middle: A1, right A2
const int intpin_claw = 0;

// Initialize nav x,y,d
grid start_pos(4, 1, 0);
grid end_pos(6, 5, 90);

int threshold_values[3] = {0, 800, 0};

// Heading motor proportional scaling factor:
const int dmotor_scaling = 50; 
// Initial change
const int dmotor_initial = 2;

// ================================================================ //
// MOTOR initialization
const int wheel_pwm = 125;
const int claw_pwm = 100;

// Initialize motors (en, dir)
motor port(3,4);
motor starboard(5,6);
motor wheel(9,8, wheel_pwm);
motor clarm(11, 12, claw_pwm); // Claw arm

int current_heading;

DriveMotor Driver(port, starboard, dmotor_scaling, dmotor_initial);

// ================================================================ //

// Flags
bool FLAG_NAVERR = false;
bool FLAG_DONE = false;

rgb_lcd lcd;
nav Navigator(start_pos, Driver, clarm);
IRSensor irsen[NUMPINS];
// ================================================================ //
// Timers

int nav_timer = -42;
// ================================================================ //

// Listener functions (folded)
void displayFunction()
{
	grid temp_grid = Navigator.currentGrid;
/*
	DEBUG(irsen[1].getValue());
	DEBUG(" ");
	DEBUG(irsen[1].detect());
	DEBUG(" | ");
	DEBUG(irsen[2].getValue());
	DEBUG(" ");
	DEBUG(irsen[2].detect());
	DEBUG(" | ");
	DEBUG(irsen[3].getValue());
	DEBUG(" ");
	DEBUG(irsen[3].detect());
	DEBUG("\r\n");
	DEBUG("Current heading: ");
	DEBUG(current_heading);
	DEBUG("\r\n");
	DEBUG(" x: ");
	DEBUG(temp_grid.x);
	DEBUG(" y: ");
	DEBUG(temp_grid.y);
	DEBUG(" d: ");
	DEBUG(temp_grid.d);
	DEBUG("\r\n");
	DEBUG("# Tasks: ");
	DEBUG(Navigator.countRemaining());
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
	/*
	DEBUG("Sum pins ");
	DEBUG(sum_lines);
	DEBUG("\r\n");
	*/

	if (sum_lines == NUMPINS && Driver.get_status() != STOPPED)
	{
		Navigator.interrupt(LINE_ISR);			
	}

	// Update heading
	// DEBUG current_heading variable
	current_heading = Driver.mapLine(
		irsen[1].detect(),
		irsen[2].detect(),
		irsen[3].detect()
	);
}
void doneFunction()
{
	//DEBUG("DONE");
	lcd.clear();
	lcd.print("DONE");
	FLAG_DONE = true;
}

// ================================================================ //

void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);

	//wheel.left();	

	// Pins
	pinMode(btnCalibrate, INPUT);
	pinMode(intpin_claw, INPUT);

	// Set threshold values for irsensor
	for (int i = 0; i < NUMPINS; ++i)
	{
		// Make new IRSensor
		irsen[i] = IRSensor(senPins[i], 4);
		irsen[i].setThresh(threshold_values);
	}

	// DEBUG
	Navigator.tasklist.push(task(PAUSE, 2000));
	Navigator.tasklist.push(task(ROTATETO, 270));

/* // Check for navigation error
	int ret_err = Navigator.computeRectilinearPath(end_pos);
	DEBUG("Computation result: ");
	DEBUG(ret_err);
	DEBUG("\r\n");
	if (ret_err < 0)
	{
		grid temp_grid = Navigator.getDestination(); 

		// DEBUG
		DEBUG(" x: ");
		DEBUG(temp_grid.x);
		DEBUG(" y: ");
		DEBUG(temp_grid.y);
		DEBUG(" d: ");
		DEBUG(temp_grid.d);
		DEBUG("\r\n");
		DEBUG("# Tasks: ");
		DEBUG(Navigator.countRemaining());

		DEBUG("NAV ERROR");
		lcd.clear();
		lcd.print("NAV ERROR");
		FLAG_NAVERR = true;
	}
*/
}

void loop()
{
	static int main_lap = 0;
	if (FLAG_NAVERR == true || FLAG_DONE == true)
		return;

	// Check for rising claw interrupt
	static int intpin_last = LOW;
	int intpin_now = digitalRead(intpin_claw);
	if (intpin_now == HIGH && intpin_last == LOW)
		Navigator.interrupt(CLAW_TOUCH);	
	intpin_last = intpin_now;

	// Check for nav_timer expiration
	if (nav_timer > 0) 
	{
		--nav_timer;
		DEBUG("Nav Timer: ");
		DEBUG(nav_timer);
		DEBUG("\r\n");
	}
	else if (nav_timer <= 0 && nav_timer != -42)
	{
		Navigator.interrupt(TIMER);
		nav_timer = -42;
	}	

	// Check if there are any tasks left to do
	if ((millis() - main_lap) > 20)
	{
		// DEBUG
		grid temp_grid = Navigator.currentGrid;

		/*
		DEBUG("Current location: ");
		DEBUG(" x: ");
		DEBUG(temp_grid.x);
		DEBUG(" y: ");
		DEBUG(temp_grid.y);
		DEBUG(" d: ");
		DEBUG(temp_grid.d);
		DEBUG("\r\n");
		*/

		bool temp_ret = Navigator.doneTasks();
		/*
		DEBUG("Is done: ");
		DEBUG(temp_ret);
		DEBUG("\r\n");
		*/
		if (temp_ret == true)
		{
			doneFunction();
		}
		else if (Navigator.getMotion() == IDLE)
		{
			Navigator.startTask(nav_timer);
		}
		else if (Navigator.checkTaskComplete() == true)
		{
			Driver.stop();
			/*
			grid temp_grid = Navigator.taskdestination;
			DEBUG("Task destination: ");
			DEBUG(" x: ");
			DEBUG(temp_grid.x);
			DEBUG(" y: ");
			DEBUG(temp_grid.y);
			DEBUG(" d: ");
			DEBUG(temp_grid.d);
			DEBUG("\r\n");
			DEBUG("Task Complete");
			DEBUG("\r\n");
			*/
		}	
	}

	// Event manager processing
	addEvents();

}


// ================================================================ //

void addEvents()
{
	// Button state declarations
	static int btnCal_state = digitalRead(btnCalibrate);
	// Timing loops 
	static unsigned int display_lap = 0;
	static unsigned int poll_lap = 0;
	static unsigned int nav_lap = 0;

	// Display event
	if ((millis() - display_lap) > 200)
	{
		DEBUG(">> Current action: ");
		DEBUG(Navigator.getMotion());
		DEBUG("\r\n");

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
	if ((millis() - poll_lap) > 20) // 20
	{
		// DEBUG
		sensorPollingFunction();	
		poll_lap = millis();
		/*
		DEBUG("White ");
		DEBUG(threshold_values[WHITE]);
		DEBUG("Black ");
		DEBUG(threshold_values[BLACK]);
		*/
	}

	if (millis() - nav_lap > 50)
	{
		++Navigator.cycle_count;
		Navigator.processTask();
		nav_lap = millis();
	}

	// Check for button press
	int calRead = digitalRead(btnCalibrate);
	if (btnCal_state == LOW && calRead == HIGH)
	{
		calibrate_all();
		lcd.clear();
		lcd.print("Calibrated");
	}
	btnCal_state = calRead;
}

void calibrate_all() // TODO terrible hack
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
