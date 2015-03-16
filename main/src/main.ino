#include <Wire.h>
#include <rgb_lcd.h>
#include <irsensor.h>
#include <motor.h>
//#include <EventManager.h>
#include <Metro.h>

#include "nav.h"
#include "drivemotor.h"

// Enable debug messages through serial
#define SERIALDEBUG

#ifdef SERIALDEBUG
#define DEBUG( x ) Serial.print( x )
#else
#define DEBUG( x )
#endif

//EventManager myEvents();

// ================================================================ //
// ADJUSTABLE PARAMETERS

const int btnCalibrate = 10;
const int NUMPINS = 4; // Initialize irsensors
const int senPins[NUMPINS] = {A13,A14,A15,A12}; 
const int numCyclesTrack = 1;
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
motor starboard(5,4);
motor port(7,6);
motor wheel(12,13, wheel_pwm);
motor clarm(14, 15, claw_pwm); // Claw arm

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

Metro displayTimer = Metro(200);
Metro sensorPollTimer = Metro(30);
Metro navProcessTimer = Metro(50);
Metro navDelayTimer = Metro(3600000); // Set to 1 hour when unused

// ================================================================ //

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
		irsen[i] = IRSensor(senPins[i], numCyclesTrack);
		irsen[i].setThresh(threshold_values);
	}

	// DEBUG
	Navigator.tasklist.push(task(PAUSE, 5000));
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

	// Check if there are any tasks left to do
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
		int nav_timer;
		DEBUG("Is idle.");
		DEBUG("\r\n");
		Navigator.startTask(nav_timer);
		navDelayTimer.interval(nav_timer);
		navDelayTimer.reset();
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
	// Event manager processing
	addEvents();

}


// ================================================================ //

void display()
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
}

void addEvents()
{
	// Button state declarations
	static int btnCal_state = digitalRead(btnCalibrate);

	// Display event
	if (displayTimer.check() == 1) display();

	// Poll sensors
	if (sensorPollTimer.check() == 1) // 20
	{
		// DEBUG
		sensorPollingFunction();	
		/*
		DEBUG("White ");
		DEBUG(threshold_values[WHITE]);
		DEBUG("Black ");
		DEBUG(threshold_values[BLACK]);
		*/
	}

	if (navProcessTimer.check() == 1) Navigator.processTask();
	if (navDelayTimer.check() == 1) 
	{
		Navigator.interrupt(TIMER);
		DEBUG("Nav Timer tripped.");
		DEBUG("\r\n");
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
