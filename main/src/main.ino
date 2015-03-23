#include <Wire.h>
#include <rgb_lcd.h>
#include <irsensor.h>
#include <Keypad.h>
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

// ================================================================ //
// ADJUSTABLE PARAMETERS

const int btnCalibrate = 10;
const int NUMPINS = 4; // Initialize irsensors
const int senPins[NUMPINS] = {A15,A14,A13,A12}; // l,m,r,offset
const int numCyclesTrack = 2;
const int blackthresh = 600; // Threshold for black line

int TM::board_now;

// PINS

const int clarmPin = 3;
const int boardPin = 2;
const int hopperlPin = 5;
const int hopperrPin = 4;
const int solenoidPin = 6;

const int INTencPortPin= 5;
const int INTencStarboardPin = 4;


// Playing field constants (cm)

double TM::lineSep = 20;
unsigned int TM::lineSepTicks = floor(lineSep / (2*M_PI*Rw) * Tr);

// Task Manager (cm)
double TM::Rw = 1.905; // Wheel radii
double TM::D = 24.5; // Wheel separation
double TM::Tr = 8; // Ticks per rotation

// XXX SET THIS
int TM::timeforaline;

// Initialize nav x,y,d
grid start_pos(4, 1, 0);
grid end_pos(6, 5, 90);

int threshold_values[3] = {0, 800, 0};

// Heading motor proportional scaling factor:
const int dmotor_scaling = 50; 
// Initial change
const int dmotor_initial = 2;

// ================================================================ //

// Keypad

const byte rows = 4;
const byte cols = 3;

const int INBUFFERSIZE = 4;
char inputbuffer[INBUFFERSIZE];
char keys[rows][cols] = {
 	{'1','2','3'},
	{'4','5','6'},
	{'7','8','9'},
	{'*','0','#'}
};

// XXX TODO These pins are wrong XXX
//connect to the row pinouts of the keypad
byte rowPins[rows] = {31, 33, 35, 37}; 

//connect to the column pinouts of the keypad
byte colPins[cols] = {39, 41, 43}; 

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);
// ================================================================ //
// MOTOR initialization
int TM::wheel_pwm = 0;
int TM::clarm_pwm = 200;

// Initialize motors (en, dir)
motor starboard(8,9);
motor port(10,11);
motor wheel(7,50,TM::wheel_pwm);
motor clarm(12,13,TM::clarm_pwm); // Claw arm

int current_heading;

DriveMotor Driver(port, starboard, dmotor_scaling, dmotor_initial);
// ================================================================ //
// Important stuff

bool FLAG_NAVERR = false;
bool FLAG_DONE = false;
bool FLAG_CANSTART = false;

rgb_lcd lcd;
Nav Navigator(start_pos);
IRSensor irsen[NUMPINS];

DriveMotor* TM::taskDriver = &Driver;
motor* TM::taskClarm = &clarm;
motor* TM::taskWheel = &wheel;
Nav* TM::taskNav = &Navigator;
int nav_timer = 3600000;


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
	/*
	if (currentPin == BLACK && pastPin == WHITE 
		&& Driver.get_status() != STOPPED && sum_lines == NUMPINS)
	{
	*/
	if (currentPin == BLACK)
	{

		TM::interrupt(LINE_ISR);

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
void encLeftPin() 
{ 
	//Serial.println("INTLEFT");
	Navigator.incEncPortCNT(); 
}
void encRightPin() 
{ 
	//Serial.println("INTRIGHT");
	Navigator.incEncStarboardCNT(); 
}
int getKeyStuff()
{
	int counter = 0;
	for (int i = 0; i < INBUFFERSIZE; ++i)
		inputbuffer[i] = 0;
	while (true)
	{
		Serial.println(" waiting for key: ");
		char key = keypad.waitForKey();

		if (key != NO_KEY)
		{
			if (key == '*')
			{
				//lcd.clear();
				Serial.println(" ");
				return atoi(inputbuffer);
			}
			else if (key == '#')
			{
				//lcd.clear();
				Serial.println(" ");
				for (int i = 0; i < INBUFFERSIZE; ++i)
					inputbuffer[i] = 0;
				counter = 0;
				continue;
			}
			else if (counter < INBUFFERSIZE)
			{
				Serial.print(key);
				inputbuffer[counter] = key;
			}
			else
			{
				//lcd.clear();
				Serial.println(" ");
				for (int i = 0; i < INBUFFERSIZE; ++i)
					inputbuffer[i] = 0;
				counter = 0;
				continue;
			}
			++counter;
		}	
	}
}


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
	attachInterrupt(INTencPortPin, encLeftPin, RISING);
	attachInterrupt(INTencStarboardPin, encRightPin, RISING);

	// DEBUG COMMANDS
	Navigator.tasklist.push(task(PAUSE, 5000));
	Navigator.tasklist.push(task(ROTATEOFFGRID, 90));
	Navigator.tasklist.push(task(PAUSE, 2000));
	Navigator.tasklist.push(task(OFFGRIDOUTBOUND, 0));
	Navigator.tasklist.push(task(PAUSE, 2000));
	Navigator.tasklist.push(task(HOPPERALIGN, 0));
	Navigator.tasklist.push(task(PAUSE, 2000));
	Navigator.tasklist.push(task(CLAWRETRACT, 0));
	Navigator.tasklist.push(task(PAUSE, 2000));
	Navigator.tasklist.push(task(MOVEINREVERSE, 5000));
	Navigator.tasklist.push(task(CLAWEXTEND, 900));
	//Navigator.tasklist.push(task(PAUSE, 5000));
	//Navigator.tasklist.push(task(PAUSE, 2000));
	//Navigator.tasklist.push(task(ROTATEONGRID, 270));
	//Navigator.tasklist.push(task(ROTATEOFFGRID, 0));
	//Navigator.tasklist.push(task(PAUSE, 1000));
	//Navigator.tasklist.push(task(MOVEONGRID, 4));
	//Navigator.tasklist.push(task(PAUSE, 1000));
	//Navigator.tasklist.push(task(MOVEINREVERSE, 1));
	//Navigator.tasklist.push(task(PAUSE, 1000));

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

	/*
	// XXX BLOCKING and SHADY CODE Do keypad inputs first
	if (FLAG_CANSTART == false)
	{
		Serial.println("Hopper East x: ");
		//lcd.setCursor(0,1);
		Navigator.hopperEast.x = getKeyStuff();	
		Serial.println("Hopper East y: ");
		//lcd.setCursor(0,1);
		Navigator.hopperEast.y = getKeyStuff();	
		Serial.println("Hopper East d: ");
		//lcd.setCursor(0,1);
		Navigator.hopperEast.d = getKeyStuff();	
		Serial.print("Hopper East X ");
		Serial.print(Navigator.hopperEast.x);
		Serial.print(" Y ");
		Serial.print(Navigator.hopperEast.y);
		Serial.print(" D ");
		Serial.println(Navigator.hopperEast.d);
		FLAG_CANSTART = true;
	}
	else
	{
	*/
		// Start first task
		grid blah;
		int dope;
		TM::startTask(nav_timer, blah, dope);
		Navigator.sketchyTimer = millis();
		navDelayTimer.interval(nav_timer);
		navDelayTimer.reset();
		DEBUG("DOPE: ");
		DEBUG(dope);
		DEBUG("\r\n");
	//}

	
}

void poop()
{
	Driver.driveReverse(125);
}

void loop()
{
	wheel.left(TM::wheel_pwm);
	Navigator.currentTime = millis();

	if (FLAG_NAVERR == true)
		return;
	else if (Navigator.getMotion() == MOTIONIDLE)
	{
		// TODO, put in code to wait for further instructions
		return;
	}

	// Check for rising claw interrupt
	static int clarm_last = LOW;
	int clarm_now = digitalRead(clarmPin);
	if (clarm_now == HIGH && clarm_last == LOW)
	{
		TM::interrupt(CLAW_TOUCH);	
		//Serial.println("LINEINT");
	}
	clarm_last = clarm_now;

	// Check for rising gameboard interrupt
	static int board_last = LOW;
	TM::board_now = digitalRead(boardPin);
	if (TM::board_now == HIGH && board_last == LOW)
		TM::interrupt(BOARD_TOUCH);	
	board_last = TM::board_now;

	// Solenoid pin
	digitalWrite(solenoidPin, TM::board_now);
	//Serial.println(TM::board_now);

	// Check for rising hopper left interrupt
	if(TM::FLAG_hopperleft = digitalRead(hopperlPin) == true)
		TM::interrupt(HOPPER_TOUCH_LEFT);	

	if(TM::FLAG_hopperright = digitalRead(hopperrPin) == true)
		TM::interrupt(HOPPER_TOUCH_RIGHT);	
	// XXX
	/*
	static int hopperl_last = LOW;
	int hopperl_now = digitalRead(hopperlPin);
	TM::FLAG_hopperleft = hopperl_now;
	if (hopperl_now == HIGH && hopperl_last == LOW)
	{
		TM::interrupt(HOPPER_TOUCH_LEFT);	
		Serial.print("HL");
	}
	hopperl_last = hopperl_now;

	// Check for rising hopper right interrupt
	static int hopperr_last = LOW;
	int hopperr_now = digitalRead(hopperrPin);
	TM::FLAG_hopperright = hopperr_now;
	if (hopperr_now == HIGH && hopperr_last == LOW)
	{
		TM::interrupt(HOPPER_TOUCH_RIGHT);	
		Serial.print("HR");
	}
	hopperr_last = hopperr_now;
	*/

	// Check if there are any tasks left to do
	if (TM::checkTaskComplete() == true)
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
			TM::startTask(nav_timer, dest, gg);
			Navigator.sketchyTimer = millis();

			DEBUG("Starting new task. ");
			grid home_grid = Navigator.getGrid();
			DEBUG(" CURR ");
			DEBUG(" x: ");
			DEBUG(home_grid.x);
			DEBUG(" y: ");
			DEBUG(home_grid.y);
			DEBUG(" d: ");
			DEBUG(home_grid.d);
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
			DEBUG(TM::taskNav->getMotion());
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
		DEBUG(TM::taskNav->getMotion());
		grid home_grid = Navigator.getGrid();
		DEBUG(" CURR ");
		DEBUG(" x: ");
		DEBUG(home_grid.x);
		DEBUG(" y: ");
		DEBUG(home_grid.y);
		DEBUG(" d: ");
		DEBUG(home_grid.d);
		grid temp_grid = TM::taskdestination;
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
	DEBUG("MOT: ");
	DEBUG(Navigator.getMotion());
	DEBUG(" DRV: ");
	DEBUG(Driver.get_status());
	DEBUG(" HED: ");
	DEBUG(Driver.current_heading);
	DEBUG(" ADJSPD: ");
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
	DEBUG(" PMS ");
	DEBUG(port.motorspeed);
	DEBUG(" SMS ");
	DEBUG(starboard.motorspeed);
	DEBUG(" PT ");
	DEBUG(port.get_status());
	DEBUG(" ST ");
	DEBUG(starboard.get_status());
	DEBUG(" PENC ");
	DEBUG(Navigator.encPortCNT);
	DEBUG(" SENC ");
	DEBUG(Navigator.encStarboardCNT);
	DEBUG(" HR ");
	DEBUG(TM::FLAG_hopperright);
	DEBUG(" HL ");
	DEBUG(TM::FLAG_hopperleft);
	DEBUG(" WPWM ");
	DEBUG(TM::wheel_pwm);
	DEBUG("\r\n");
	/*
	DEBUG("GPOS X ");
	DEBUG(Navigator.currentGrid.x);
	DEBUG(" Y ");
	DEBUG(Navigator.currentGrid.y);
	DEBUG(" D ");
	DEBUG(Navigator.currentGrid.d);
	DEBUG("OPOS X ");
	DEBUG(Navigator.offgridpos.x);
	DEBUG(" Y ");
	DEBUG(Navigator.offgridpos.y);
	DEBUG(" D ");
	DEBUG(Navigator.offgridpos.d)
	DEBUG("\r\n");
	*/

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
		TM::processTask(debug_speed);
	}
	if (navDelayTimer.check() == 1) 
	{
		TM::interrupt(TIMER);
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
