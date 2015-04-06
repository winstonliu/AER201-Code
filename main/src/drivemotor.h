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
		int prop, deriv;
		drive_status currentStatus;
	public:
		motor *ptr_port;
		motor *ptr_starboard;
		int current_heading;
		int newSpeed;
		DriveMotor(motor& port, motor& starboard, int prop, int deriv);
		int mapLine(bool left, bool middle, bool right);
		int lineMotorScaling(int baseSpeed = 255);

		void driveStraight(int speed = 255);
		void driveReverse(int speed = 255);
		void turnLeft(int speed = 255);
		void turnRight(int speed = 255);
		void pivotLeft(int speed = 255);
		void pivotLeftReverse();
		void pivotRight(int speed = 255);
		void pivotRightReverse();
		void adjustSpeed(int speed);	
		void doOpposite();
		drive_status get_status();
		void stop();
};
