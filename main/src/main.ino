#include <Wire.h>
#include <rgb_lcd.h>
#include <EventManager.h>

#include <pid.h>
#include <line_pid.h>
#include <irsensor.h>
#include <motor.h>
#include <nav.h>

// Flags
bool FLAG_NAVERR = false;
bool FLAG_DONE = false;

// Initialize lcd 
rgb_lcd lcd;

// Initialize nav x,y,d
grid start_pos(4, 1, 0);
grid end_pos(6, 5, 90);

// Initialize Event Handling
EventManager eVM;

// Initialize navigation
nav Navigator(start_pos);

// Initialize irsensors
const int NUMPINS = 4;
//const int senPins[NUMPINS] = {A0,A1,A2,A3,A4};
const int senPins[NUMPINS] = {A0,A1,A2,A3};
IRSensor irsen[NUMPINS];

// Initialize motors (en, dir)
motor port(3,2);
motor starboard(5,4);
motor wheel(9,8);

// PID values
const int target_heading = 0;
int current_heading = 0;
int motor_pwm = 0;

// PID Control initialize
const int NUMMOTO = 2;
PID motoPID[NUMMOTO];	 // port 0, starboard 1

const int btnCalibrate = 6;

// White, black, red
int threshold_values[3] = {220, 800, 0};

// Listener functions (folded)
void displayFunction(int event, int param)
{
	grid temp_grid = Navigator.getGrid();
	if (event == EventManager::kEventDisplaySerial)
	{
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
	for (int i = 0; i < NUMPINS; ++i) 
	{
		irsen[i].readSensor(); 
		// Perpendicular line detection for past cycles
		sum_lines += irsen[i].pastEncounters();	
	}

	// If all sensors have been triggered in the past n cycles,
	// then trigger line detected
	if (sum_lines == NUMPINS)
	{
		Navigator.interrupt(LINE_ISR);			
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
	wheel.stop();
}

void setup()
{
	Serial.begin(9600);
	lcd.begin(16,2);

	wheel.left(125);	

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
		motoPID[i] = PID(current_heading, target_heading, motor_pwm);
		motoPID[i].start();		
		motoPID[i].set_cycle(50);		
	}

	// Set threshold values for irsensor
	for (int i = 0; i < NUMPINS; ++i)
	{
		// Make new IRSensor
		irsen[i] = IRSensor(senPins[i]);
		irsen[i].setThresh(threshold_values);
	}

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
}

void loop()
{
	static int main_lap = 0;

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
			Serial.println("DONE");
			lcd.clear();
			lcd.print("DONE");
			FLAG_DONE = true;
		}
		else if (Navigator.getAction() == IDLE)
		{
			Navigator.startTask();
		}
		else if (Navigator.checkTaskComplete() == 0)
		{
			killMotors();		
		}	

		Serial.print("Current action: ");
		Serial.println(Navigator.getAction());
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
		// DEBUG
		Serial.println("Display");

		eVM.queueEvent(EventManager::kEventDisplaySerial, 0);
		display_lap = millis();
	}

	// Poll sensors
	if ((millis() - poll_lap) > 20)
	{
		// DEBUG
		Serial.println("Sensor Poll");

		eVM.queueEvent(EventManager::kEventSensorPolling, 0);
		poll_lap = millis();
	}

	if (Navigator.getAction() == MOVEFORWARD)	
	{
		// DEBUG
		Serial.println("Moving Forward");

		// TODO Toggles every 50 ms
		if (motoPID[0].compute() == true)
			port.adjustSpeed(motor_pwm);	
		if (motoPID[1].compute() == true)
			starboard.adjustSpeed(motor_pwm);	
	}
	else if (Navigator.getAction() == ROTATETO)
	{
		// DEBUG
		Serial.println("Rotating");

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
		eVM.queueEvent(EventManager::kEventCalibrate, 0);
	}
	btnCal_state = calRead;
}

void calibrate_all()
{
	// Middle sensor calibrates for black
	// Right and left sensor averages calibrate for white
	threshold_values[BLACK] = irsen[2].readSensor();	
	threshold_values[WHITE] = ( irsen[1].readSensor() 
								+ irsen[3].readSensor()) / 2;

	// Set values
	for (int i = 0; i < NUMPINS; ++i)
	{
		irsen[i].setThresh(threshold_values);
	}
}
