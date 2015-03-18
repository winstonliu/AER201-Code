#pragma once
#include <motor.h>

const int ON_WHITE = false;

enum drive_status
{
	DRIVINGFORWARD,
	TURNINGLEFT,
	TURNINGRIGHT,
	PIVOTLEFT,
	PIVOTRIGHT,
	STOPPED
};

class DriveMotor
{
	private:
		int current_heading;
		int scaling, initial;
		motor port, starboard, clarm;
		drive_status currentStatus;
	public:
		DriveMotor(motor& port, motor& starboard, 
				int scaling = 4, int initial = 10);
		int mapLine(bool left, bool middle, bool right);
		int lineMotorScaling();
		void driveStraight();
		void turnLeft();
		void turnRight();
		void pivotLeft();
		void pivotRight();
		drive_status get_status();
		void stop();
};

