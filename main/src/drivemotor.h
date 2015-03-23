#pragma once
#include "motor.h"

const int ON_WHITE = false;

enum drive_status
{
	DRIVINGFORWARD,
	DRIVINGREVERSE,
	TURNINGLEFT,
	TURNINGRIGHT,
	PIVOTLEFT,
	PIVOTLEFTBACK,
	PIVOTRIGHT,
	PIVOTRIGHTBACK,
	STOPPED
};

class DriveMotor
{
	private:
		int scaling, initial;
		motor *ptr_port;
		motor *ptr_starboard;
		drive_status currentStatus;
	public:
		int current_heading;
		DriveMotor(motor& port, motor& starboard, 
				int scaling = 4, int initial = 10);
		int mapLine(bool left, bool middle, bool right);
		int lineMotorScaling(int baseSpeed = 255);

		void driveStraight(int speed = 255);
		void driveReverse(int speed = 255);
		void turnLeft(int speed = 255);
		void turnRight(int speed = 255);
		void pivotLeft();
		void pivotLeftReverse();
		void pivotRight();
		void pivotRightReverse();
		drive_status get_status();
		void stop();
};
