#include <Wire.h>
#include <rgb_lcd.h>
#include <irsensor.h>
#include <motor.h>
//#include <EventManager.h>
#include <Metro.h>

#include "nav.h"
#include "taskmanager.h"
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
const int senPins[NUMPINS] = {A15,A14,A13,A12}; // l,m,r,offset
const int numCyclesTrack = 2;
const int clarmPin = 13;
const int blackthresh = 700; // Threshold for black line
const int INTencLeftPin= 0;
const int INTencRightPin = 1;

// Playing field constants (cm)

double TaskManager::lineSep = 20;
unsigned int TaskManager::lineSepTicks = floor(lineSep / (2*M_PI*Rw));

// Task Manager (cm)
double TaskManager::Rw = 1.905; // Wheel radii
double TaskManager::D = 24.5; // Wheel separation
double TaskManager::Tr = 8; // Ticks per rotation

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
motor starboard(8,9);
motor port(10,11);
motor wheel(12,13, wheel_pwm);
motor clarm(14, 15, claw_pwm); // Claw arm

int current_heading;

DriveMotor Driver(port, starboard, dmotor_scaling, dmotor_initial);

// ================================================================ //

// Flags
bool FLAG_NAVERR = false;
bool FLAG_DONE = false;

rgb_lcd lcd;
Nav Navigator(start_pos);
IRSensor irsen[NUMPINS];

DriveMotor* TaskManager::taskDriver = &Driver;
motor* TaskManager::taskClarm = &clarm;
Nav* TaskManager::taskNav = &Navigator;
int nav_timer;

// ================================================================ //
// Timers

Metro displayTimer = Metro(500);
Metro sensorPollTimer = Metro(30);
Metro navProcessTimer = Metro(50);
Metro navDelayTimer = Metro(3600000); // Set to 1 hour when unused

// ================================================================ //


// XXX DEBUG CODE
int debug_speed;
long main_lap = 0;
// XXX

void sensorPollingFunction()
{
	int sum_lines = 0;
	// Read sensors
	for (int i = 0; i < NUMPINS; ++i) 
	{
		irsen[i].readSensor(); 
		sum_lines += irsen[i].pastEncounters();
	}

	// If all sensors have been triggered in the past n cycles,
	// then trigger line detected
	/*
	DEBUG("Sum pins ");
	DEBUG(sum_lines);
	DEBUG("\r\n");
	*/

	// Check the offset sensor for line pass detection, look for rising edge
	static int pastPin = WHITE;
	int currentPin = irsen[3].detect();
	if (currentPin == BLACK && pastPin == WHITE 
		&& Driver.get_status() != STOPPED && sum_lines == NUMPINS)
	{
		TaskManager::interrupt(LINE_ISR);

		/* DEBUG("#");
		DEBUG(main_lap);
		DEBUG("# ");
		DEBUG("Line interrupt tripped. ");
		DEBUG(irsen[0].getValue());
		DEBUG(" ");
		DEBUG(irsen[1].getValue());
		DEBUG(" ");
		DEBUG(irsen[2].getValue());
		DEBUG(" ");
		DEBUG(irsen[3].getValue());
		DEBUG("\r\n"); */
	}
	pastPin = currentPin;

	// Update heading
	current_heading = Driver.mapLine(
		irsen[0].detect(),
		irsen[1].detect(),
		irsen[2].detect()
	);
}
void encLeftPin() { Navigator.incEncPortCNT(); }
void encRightPin() { Navigator.incEncStarboardCNT(); }

// ================================================================ //

void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);

	//wheel.left();	

	// Pins
	pinMode(btnCalibrate, INPUT);
	pinMode(clarmPin, INPUT);

	// Set threshold values for irsensor
	for (int i = 0; i < NUMPINS; ++i)
	{
		// Make new IRSensor
		irsen[i] = IRSensor(senPins[i], numCyclesTrack, blackthresh);
		irsen[i].setThresh(threshold_values);
	}

	// Interrupts
	attachInterrupt(INTencLeftPin, encLeftPin, RISING);
	attachInterrupt(INTencRightPin, encRightPin, RISING);

	// DEBUG COMMANDS
	Navigator.tasklist.push(task(PAUSE, 5000));
	Navigator.tasklist.push(task(MOVEONGRID, 1));
	Navigator.tasklist.push(task(PAUSE, 2000));
	Navigator.tasklist.push(task(MOVEINREVERSE, 0));
	//Navigator.tasklist.push(task(ROTATEONGRID, 90));

	// Start first task
	grid blah;
	int dope;
	TaskManager::startTask(nav_timer, blah, dope);
	navDelayTimer.interval(nav_timer);
	navDelayTimer.reset();
	DEBUG("DOPE: ");
	DEBUG(dope);
	DEBUG("\r\n");


/* 
	// Check for navigation error
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

void poop()
{
	port.left(50);
	starboard.right();
}

void loop()
{
	if (FLAG_NAVERR == true || FLAG_DONE == true)
		return;

	// Check for rising claw interrupt
	static int clarm_last = LOW;
	int clarm_now = digitalRead(clarmPin);
	if (clarm_now == HIGH && clarm_last == LOW)
		TaskManager::interrupt(CLAW_TOUCH);	
	clarm_last = clarm_now;

	// Check if there are any tasks left to do
	if (TaskManager::checkTaskComplete() == true)
	{
		nav_timer = 3600000; // default is 1 hour
		DEBUG("FULL STOP");
		DEBUG("\r\n");
		Driver.stop();
		Navigator.advance();
		if (Navigator.doneTasks() == false)
		{
			grid dest;
			int gg;

			Navigator.resetEncCNT();
			TaskManager::startTask(nav_timer, dest, gg);

			DEBUG("Starting new task. ");
			DEBUG(" DEST ");
			DEBUG(" x: ");
			DEBUG(dest.x);
			DEBUG(" y: ");
			DEBUG(dest.y);
			DEBUG(" d: ");
			DEBUG(dest.d);
			DEBUG(" NAVVAL: ");
			DEBUG(gg);
			DEBUG(" MOTION: ");
			DEBUG(TaskManager::taskNav->getMotion());
			DEBUG("\r\n");
		}
		else
		{
			FLAG_DONE = true;	
		}

		navDelayTimer.interval(nav_timer);
		navDelayTimer.reset();

		DEBUG("#");
		DEBUG(main_lap);
		DEBUG("# ");
		DEBUG("Task completed. ");
		DEBUG(TaskManager::taskNav->getMotion());
		grid home_grid = Navigator.getGrid();
		DEBUG(" CURR ");
		DEBUG(" x: ");
		DEBUG(home_grid.x);
		DEBUG(" y: ");
		DEBUG(home_grid.y);
		DEBUG(" d: ");
		DEBUG(home_grid.d);
		grid temp_grid = TaskManager::taskdestination;
		DEBUG(" DEST ");
		DEBUG(" x: ");
		DEBUG(temp_grid.x);
		DEBUG(" y: ");
		DEBUG(temp_grid.y);
		DEBUG(" d: ");
		DEBUG(temp_grid.d);
		DEBUG("\r\n");
	}	
	// Event manager processing
	addEvents();
	++main_lap;

}


// ================================================================ //

void display()
{
	DEBUG("#");
	DEBUG(main_lap);
	DEBUG("# ");
	DEBUG("Current motion: ");
	DEBUG(Navigator.getMotion());
	DEBUG(" Driver: ");
	DEBUG(Driver.get_status());
	DEBUG("\r\n");
	DEBUG("Heading: ");
	DEBUG(Driver.current_heading);
	DEBUG(" Adjust speed: ");
	DEBUG(debug_speed);
	DEBUG(" RAW ");
	DEBUG(irsen[0].readSensor());
	DEBUG(" ");
	DEBUG(irsen[1].readSensor());
	DEBUG(" ");
	DEBUG(irsen[2].readSensor());
	DEBUG(" ");
	DEBUG(irsen[3].readSensor());
	DEBUG(" DET ");
	DEBUG(irsen[0].detect());
	DEBUG(" ");
	DEBUG(irsen[1].detect());
	DEBUG(" ");
	DEBUG(irsen[2].detect());
	DEBUG(" ");
	DEBUG(irsen[3].detect());
	DEBUG(" P ");
	DEBUG(port.motorspeed);
	DEBUG(" S ");
	DEBUG(starboard.motorspeed);
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
		sensorPollingFunction();	
	}

	if (navProcessTimer.check() == 1) 
	{
		TaskManager::processTask(debug_speed);
	}
	if (navDelayTimer.check() == 1) 
	{
		TaskManager::interrupt(TIMER);
		DEBUG("#");
		DEBUG(main_lap);
		DEBUG("# ");
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
