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
#define IRSEN_DEBUG
#define ENC_DEBUG
//#define MOTO_DEBUG
#define NAV_DEBUG
//#define HOPPER_DEBUG
#else
#define DEBUG( x )
#endif

#ifdef IRSEN_DEBUG
#define DEBUG_IR( x ) Serial.print( x )
#else
#define DEBUG_IR( x )
#endif

#ifdef MOTO_DEBUG
#define DEBUG_MOTO( x ) Serial.print( x )
#else
#define DEBUG_MOTO( x )
#endif

#ifdef NAV_DEBUG
#define DEBUG_NAV( x ) Serial.print( x )
#else
#define DEBUG_NAV( x )
#endif

#ifdef HOPPER_DEBUG
#define DEBUG_HOP( x ) Serial.print( x )
#else
#define DEBUG_HOP( x )
#endif

#ifdef ENC_DEBUG
#define DEBUG_ENC( x ) Serial.print( x )
#else
#define DEBUG_ENC( x )
#endif



// ================================================================ //
// ADJUSTABLE PARAMETERS

const int btnCalibrate = 10;
const int numCyclesTrack = 2;
const double bthresh = 0.5; // Percentage threshold for black line

int TM::board_now = LOW;

// PINS

const int clarmPin = 3;
const int boardPin = 2;
const int hopperlPin = 5;
const int hopperrPin = 4;
const int servoPin = 6;

const int INTencPortPin= 4; // 19
const int INTencStarboardPin = 5; // 18


// Playing field constants (cm)

double TM::lineSep = 20;
int TM::lineSepTicks = floor(lineSep / (2*M_PI*Rw) * Tr);

// Task Manager (cm)
const double TM::Rw = 3.44; // Wheel radii
const double TM::D = 26; // Wheel separation
const double TM::RwD = TM::Rw/TM::D; // 4.5 / 25
const double TM::Tr = 20; // Ticks per rotation
const double TM::Tr_TRD = 0.079; // Ticks per turning radius degree

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
const int TM::wheel_norm = 50;
int TM::wheel_pwm = 0;
int TM::clarm_pwm = 225;

// Initialize motors (en, dir)
motor starboard(8,9);
motor port(10,11);
motor wheel(7,50,TM::wheel_pwm);
motor clarm(12,13,TM::clarm_pwm); // Claw arm

// Port, starboard, P, D of proportional-derivative adjustment
DriveMotor Driver(port, starboard, 0.05, 1);
Servo myservo;
// ================================================================ //
// Important stuff

bool FLAG_NAVERR = false;
bool FLAG_DONE = false;
bool FLAG_CANSTART = false;

rgb_lcd lcd;
Nav Navigator(start_pos);
macromotion currentMM;

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

Metro encoderTimer = Metro(100);
Metro displayTimer = Metro(500);
Metro sensorPollTimer = Metro(30);
Metro navProcessTimer = Metro(50);
Metro navDelayTimer = Metro(3600000); // Set to 1 hour when unused

// ================================================================ //

// DEBUG CODE
long main_lap = 0;
// 

void sensorPollingFunction()
{
	static long pollTime = millis();
	
	Driver.current_heading = qtrline.readLine(senPinVal);
	qtrext.readCalibrated(senExtVal);

	// Check the offset sensor for line pass detection, look for rising edge
	// XXX Reconfig line sensor detection
	bool extLeft = (senExtVal[0] > (qtrext.calibratedMaximumOn[0] * bthresh));
	bool extRight = (senExtVal[1] > (qtrext.calibratedMaximumOn[1] * bthresh));
	bool driveStat = (Driver.get_status() != STOPPED);
	bool minTime = (Navigator.currentTime - pollTime) > 500;
	bool lineTime = (TM::dirLineInc(1).x == 4) 
		&& (Navigator.absEncDistance() >= TM::lineSep);

	if (((extLeft & extRight & minTime & driveStat) == true) 
		|| ((driveStat & lineTime) == true))
	{ 
		TM::interrupt(LINE_ISR);
		pollTime = millis();
		DEBUG("LINEINT EX ");
		DEBUG(senExtVal[0]);
		DEBUG(" ");
		DEBUG(senExtVal[1]);
		DEBUG(" LL ");
		DEBUG(senPinVal[0]);
		DEBUG(" ");
		DEBUG(senPinVal[1]);
		DEBUG(" ");
		DEBUG(senPinVal[2]);
		DEBUG(" ");
		DEBUG(senPinVal[3]);
		DEBUG("\r\n");
	}
}
bool checkOnLine()
{
	qtrline.readCalibrated(senPinVal);
	qtrext.readCalibrated(senExtVal);

	bool extLeft = (senExtVal[0] > (qtrext.calibratedMaximumOn[0] * bthresh));
	bool extRight = (senExtVal[1] > (qtrext.calibratedMaximumOn[1] * bthresh));
	bool online = true;
	for (int i = 0; i < NUMPINS; ++i)
	{
		online = online & (senPinVal[i] > (qtrline.calibratedMaximumOn[i]*bthresh));
	}

	if ((extLeft & extLeft & online) == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}
void lineCalibrate()
{
	// ~ 3 seconds for calibration
	DEBUG("Calibrating now...\r\n");
	delay(100);
	for (int i = 0; i < 300; ++i)
	{
		qtrline.calibrate();
		qtrext.calibrate();
	}
	DEBUG("Done calibration:\r\n");

	// Min max debug code
	for (int i = 0; i < NUMPINS; ++i)
	{
		DEBUG("\tLIN");
		DEBUG(i);
		DEBUG("\t");
		DEBUG(qtrline.calibratedMinimumOn[i]);
		DEBUG("\t");
		DEBUG(qtrline.calibratedMaximumOn[i]);
	}
	for (int i = 0; i < NUMEXT; ++i)
	{
		DEBUG("\tEXT");
		DEBUG(i);
		DEBUG("\t");
		DEBUG(qtrext.calibratedMinimumOn[i]);
		DEBUG("\t");
		DEBUG(qtrext.calibratedMaximumOn[i]);
	}
	DEBUG("\r\n");
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

	myservo.attach(servoPin, 1000, 2000);
	
	// Pins
	pinMode(btnCalibrate, INPUT);
	pinMode(clarmPin, INPUT);

	// Interrupts
	attachInterrupt(INTencPortPin, encLeftPin, RISING);
	attachInterrupt(INTencStarboardPin, encRightPin, RISING);

	// DEBUG COMMANDS
	// Leaving home
	lineCalibrate();
	Navigator.tasklist.push(task(PPP, 2000));
	//Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 270));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(MOG, 2));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 0));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(MOG, 1));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 270));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 230));
	/*
	Navigator.tasklist.push(task(ROG, 0));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(MOG, 1));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(ROG, 90));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.boardAndBack();
	*/
	/*
	Navigator.tasklist.push(task(PPP, 2000));
	Navigator.tasklist.push(task(RFG, 0));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 45));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 0));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 270));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(RFG, 0));
	*/
	/*
	Navigator.tasklist.push(task(MOG, 2));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(ROG, 0));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(MOG, 1));
	Navigator.tasklist.push(task(PPP, 1000));
	Navigator.tasklist.push(task(ROG, 90));
	*/
	/*
	Navigator.tasklist.push(task(OGR, 0));
	*/
	//Navigator.hopperDocking();

	currentMM = mMTC;

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
	}
	clarm_last = clarm_now;

	// Check for rising gameboard interrupt
	// Solenoid pin
	TM::board_now = digitalRead(boardPin);
	if (board_now == HIGH)
		myservo.write(180);
	else
		myservo.write(0);

	// Check for rising hopper left interrupt
	if(digitalRead(hopperlPin) == HIGH)
	{
		TM::FLAG_hopperleft = true;
		TM::interrupt(HOPPER_TOUCH_LEFT);	
	}
	else
	{
		TM::FLAG_hopperleft = false;
	}

	if(digitalRead(hopperrPin) == HIGH)
	{
		TM::FLAG_hopperright = true;
		TM::interrupt(HOPPER_TOUCH_RIGHT);	
	}
	else
	{
		TM::FLAG_hopperright = false;
	}

	// Check if there are any tasks left to do
	if (TM::iscomplete() == true)
	{
		nav_timer = 3600000; // default is 1 hour

		DEBUG("FULL STOP VAL ");
		DEBUG((int)Navigator.getOffGridPos().d % 360);
		DEBUG("\r\n");

		Driver.stop();

		DEBUG("#");
		DEBUG(main_lap);
		DEBUG("# ");
		DEBUG("Task completed. ");
		DEBUG(TM::tkNav->getMotion());
		DEBUG(" ");
		grid home_grid = Navigator.getGrid();
		display();
		DEBUG("\r\n");

		Navigator.advance();
		if (Navigator.doneTasks() == false)
		{
			Navigator.resetEncCNT();
			TM::start(nav_timer);
			navDelayTimer.interval(nav_timer);
			navDelayTimer.reset();

			Navigator.sketchyTimer = millis();

			DEBUG("Starting new task. ");
			display();
		}
		else
		{
			FLAG_DONE = true;
		}
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
	DEBUG_IR("\tNS ");
	DEBUG_IR(Driver.newSpeed);
	DEBUG(" MOT ");
	DEBUG(Navigator.getMotion());
	DEBUG(" DRV ");
	DEBUG(Driver.get_status());
	DEBUG(" HED ");
	DEBUG(Driver.current_heading);
	DEBUG(" OL ");
	DEBUG(checkOnLine());

	// IR READINGS
	DEBUG_IR(" E ");
	DEBUG_IR(senExtVal[0]);
	DEBUG_IR(" L ");
	DEBUG_IR(senPinVal[0]);
	DEBUG_IR(" ");
	DEBUG_IR(senPinVal[1]);
	DEBUG_IR(" ");
	DEBUG_IR(senPinVal[2]);
	DEBUG_IR(" ");
	DEBUG_IR(senPinVal[3]);
	DEBUG_IR(" E ");
	DEBUG_IR(senExtVal[1]);
	DEBUG_MOTO(" PMS ");
	DEBUG_MOTO(port.motorspeed);
	DEBUG_MOTO(" SMS ");
	DEBUG_MOTO(starboard.motorspeed);
	DEBUG_MOTO(" PT ");
	DEBUG_MOTO(port.get_status());
	DEBUG_MOTO(" ST ");
	DEBUG_MOTO(starboard.get_status());

	DEBUG_ENC(" PENC ");
	DEBUG_ENC(Navigator.encPortCNT);
	DEBUG_ENC(" SENC ");
	DEBUG_ENC(Navigator.encStarboardCNT);

	// Hopper flags
	DEBUG_HOP(" HR ");
	DEBUG_HOP(TM::FLAG_hopperright);
	DEBUG_HOP(" HL ");
	DEBUG_HOP(TM::FLAG_hopperleft);
	DEBUG_HOP(" WPWM ");
	DEBUG_HOP(TM::wheel_pwm);
	DEBUG_HOP(" TMR ");
	DEBUG_HOP(Navigator.timeElapsed());

	DEBUG_NAV(" TC ");
	DEBUG_NAV(Navigator.turncoord);
	//DEBUG_NAV(" IC ");
	//DEBUG_NAV(TM::internalcount);
	DEBUG_NAV(" GPOS X ");
	DEBUG_NAV(Navigator.currentGrid.x);
	DEBUG_NAV(" Y ");
	DEBUG_NAV(Navigator.currentGrid.y);
	DEBUG_NAV(" D ");
	DEBUG_NAV(Navigator.currentGrid.d);
	DEBUG_NAV(" OPOS X ");
	DEBUG_NAV(Navigator.getOffGridPos().x);
	DEBUG_NAV(" Y ");
	DEBUG_NAV(Navigator.getOffGridPos().y);
	DEBUG_NAV(" D ");
	DEBUG_NAV(Navigator.getOffGridPos().d);

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
		Navigator.setOffGridPos(TM::calcOffGrid(Navigator.getOffGridPos()));
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
