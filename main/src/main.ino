#include <Wire.h>
#include <rgb_lcd.h>
#include <Keypad.h>
#include <motor.h>
#include <QTRSensors.h>
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
const int numCyclesTrack = 2;
const int blackthresh = 600; // Threshold for black line

int TM::board_now;

// PINS

const int clarmPin = 3;
const int boardPin = 2;
const int hopperlPin = 5;
const int hopperrPin = 4;
const int solenoidPin = 6;

const int INTencPortPin= 4; // 19
const int INTencStarboardPin = 5; // 18


// Playing field constants (cm)

double TM::lineSep = 20;
int TM::lineSepTicks = floor(lineSep / (2*M_PI*Rw) * Tr);

// Task Manager (cm)
const double TM::Rw = 1.905; // Wheel radii
const double TM::D = 24.5; // Wheel separation
const double TM::RwD = 0.0777; // 1.905 / 24.5
const double TM::Tr = 8; // Ticks per rotation
const double TM::Tr_TRD = 0.1429; // Ticks per turning radius degree

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

// Port, starboard, P, D of proportional-derivative adjustment
DriveMotor Driver(port, starboard, 0.1, 5);
// ================================================================ //
// Important stuff

bool FLAG_NAVERR = false;
bool FLAG_DONE = false;
bool FLAG_CANSTART = false;

rgb_lcd lcd;
Nav Navigator(start_pos);

// Line sensing thresh = 200 out of 1000
const int NUMPINS = 4; // Initialize irsensors
const int NUMEXT = 2;
unsigned char senPins[NUMPINS] = {15,13,12,11};
unsigned char senExt[NUMEXT] = {14,10};
unsigned int senPinVal[NUMPINS];
unsigned int senExtVal[NUMEXT];
QTRSensorsAnalog qtrline(senPins, NUMPINS);
QTRSensorsAnalog qtrext(senExt, NUMEXT);

DriveMotor* TM::tkDriver = &Driver;
motor* TM::tkClarm = &clarm;
motor* TM::tkWheel = &wheel;
Nav* TM::tkNav = &Navigator;
int nav_timer = 3600000;


// ================================================================ //
// Timers

Metro encoderTimer = Metro(500);
Metro displayTimer = Metro(500);
Metro sensorPollTimer = Metro(30);
Metro navProcessTimer = Metro(50);
Metro navDelayTimer = Metro(3600000); // Set to 1 hour when unused

// ================================================================ //

// DEBUG CODE
int debug_speed;
long main_lap = 0;
// 

void sensorPollingFunction()
{
	Driver.current_heading = qtrline.readLine(senPinVal);
	qtrext.readCalibrated(senExtVal);

	// Check the offset sensor for line pass detection, look for rising edge
	if (senExtVal[0] > 200 && senExtVal[1] > 200)
	{
		TM::interrupt(LINE_ISR);
		Navigator.setGrid(TM::dirLineInc(1));	// Update location heading
	}
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

	// ~ 2 seconds for calibration
	DEBUG("Calibrating now...\r\n");
	delay(100);
	Driver.driveStraight();
	for (int i = 0; i < 100; ++i)
	{
		qtrline.calibrate();
		qtrext.calibrate();
	}
	DEBUG("Done calibration:\r\n");
	
	// Min max debug code
	for (int i = 0; i < NUMPINS; ++i)
	{
		DEBUG(" LIN");
		DEBUG(i);
		DEBUG(" ");
		DEBUG(qtrline.calibratedMinimumOn[i]);
		DEBUG(" ");
		DEBUG(qtrline.calibratedMaximumOn[i]);
	}
	for (int i = 0; i < NUMEXT; ++i)
	{
		DEBUG(" EXT");
		DEBUG(i);
		DEBUG(" ");
		DEBUG(qtrext.calibratedMinimumOn[i]);
		DEBUG(" ");
		DEBUG(qtrext.calibratedMaximumOn[i]);
	}
	DEBUG("\r\n");

	// Pins
	pinMode(btnCalibrate, INPUT);
	pinMode(clarmPin, INPUT);

	// Interrupts
	attachInterrupt(INTencPortPin, encLeftPin, RISING);
	attachInterrupt(INTencStarboardPin, encRightPin, RISING);

	// DEBUG COMMANDS
	Navigator.tasklist.push(task(PPP, 5000));
	Navigator.tasklist.push(task(PPP, 5000));
	Navigator.tasklist.push(task(MOG, 1000));

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
		TM::start(nav_timer);
		DEBUG("Starting at ");
		DEBUG(nav_timer);
		DEBUG(" PAUSE ");
		DEBUG(TM::FLAG_pause);
		DEBUG("\r\n");
		Navigator.sketchyTimer = millis();
		navDelayTimer.interval(nav_timer);
		navDelayTimer.reset();
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
	else if (Navigator.getMotion() == MOI)
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
	if (TM::iscomplete() == true)
	{
		nav_timer = 3600000; // default is 1 hour
		DEBUG("FULL STOP VAL ");
		DEBUG((int)Navigator.offgridpos.d % 360);
		DEBUG("\r\n");
		Driver.stop();

		Navigator.advance();
		if (Navigator.doneTasks() == false)
		{

			Navigator.resetEncCNT();
			TM::start(nav_timer);
			navDelayTimer.interval(nav_timer);
			navDelayTimer.reset();

			Navigator.sketchyTimer = millis();

			DEBUG("Starting new task. ");
			grid home_grid = Navigator.getGrid();
			grid dest = Navigator.getDestination();
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
			DEBUG(" MOTION: ");
			DEBUG(Navigator.getMotion());
			DEBUG(" Value: ");
			DEBUG(Navigator.getTaskValue());
			DEBUG("\r\n");
		}
		else
		{
			FLAG_DONE = true;	
		}

				DEBUG("#");
		DEBUG(main_lap);
		DEBUG("# ");
		DEBUG("Task completed. ");
		DEBUG(TM::tkNav->getMotion());
		grid home_grid = Navigator.getGrid();
		DEBUG(" CURR ");
		DEBUG(" x: ");
		DEBUG(home_grid.x);
		DEBUG(" y: ");
		DEBUG(home_grid.y);
		DEBUG(" d: ");
		DEBUG(home_grid.d);
		grid temp_grid = TM::tkdest;
		DEBUG(" DEST ");
		DEBUG(" x: ");
		DEBUG(temp_grid.x);
		DEBUG(" y: ");
		DEBUG(temp_grid.y);
		DEBUG(" d: ");
		DEBUG(temp_grid.d);
		DEBUG(" OPOS X ");
		DEBUG(Navigator.offgridpos.x);
		DEBUG(" Y ");
		DEBUG(Navigator.offgridpos.y);
		DEBUG(" D ");
		DEBUG(Navigator.offgridpos.d);
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
	DEBUG("NS ");
	DEBUG(Driver.newSpeed);
	/*
	DEBUG("MOT: ");
	DEBUG(Navigator.getMotion());
	DEBUG(" DRV: ");
	DEBUG(Driver.get_status());
	DEBUG(" HED: ");
	DEBUG(Driver.current_heading);
	DEBUG(" ADJSPD: ");
	DEBUG(debug_speed);
	*/

	// IR READINGS
	DEBUG(" EXT ");
	DEBUG(senExtVal[0]);
	DEBUG(" LIN ");
	DEBUG(senPinVal[0]);
	DEBUG(" ");
	DEBUG(senPinVal[1]);
	DEBUG(" ");
	DEBUG(senPinVal[2]);
	DEBUG(" ");
	DEBUG(senPinVal[3]);
	DEBUG(" EXT ");
	DEBUG(senExtVal[1]);
	DEBUG(" HEADING ");
	DEBUG(Driver.current_heading);

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

	// Hopper flags
	/*
	DEBUG(" HR ");
	DEBUG(TM::FLAG_hopperright);
	DEBUG(" HL ");
	DEBUG(TM::FLAG_hopperleft);
	*/
	/*
	DEBUG(" WPWM ");
	DEBUG(TM::wheel_pwm);
	DEBUG(" GPOS X ");
	DEBUG(Navigator.currentGrid.x);
	DEBUG(" Y ");
	DEBUG(Navigator.currentGrid.y);
	DEBUG(" D ");
	DEBUG(Navigator.currentGrid.d);
	DEBUG(" OPOS X ");
	DEBUG(Navigator.offgridpos.x);
	DEBUG(" Y ");
	DEBUG(Navigator.offgridpos.y);
	DEBUG(" D ");
	DEBUG(Navigator.offgridpos.d);
	*/
	DEBUG("\r\n");

	// DEBUG
	//lcd.clear();
}

void addEvents()
{
	// Button state declarations
	static int btnCal_state = digitalRead(btnCalibrate);

	// Display event
	if (displayTimer.check() == 1) 
		display();

	if (encoderTimer.check() == 1)
	{
		Navigator.offgridpos = TM::calcOffGrid(Navigator.offgridpos);
	}

	// Poll sensors
	if (sensorPollTimer.check() == 1) // 20
	{
		sensorPollingFunction();	
	}

	if (navProcessTimer.check() == 1) 
	{
		TM::process();
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
		qtrline.calibrate();
		qtrext.calibrate();
		lcd.clear();
		lcd.print("Calibrated");
	}
	btnCal_state = calRead;
}
