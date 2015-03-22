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
	PIVOTRIGHT,
	STOPPED
};

class DriveMotor
{
	private:
		int scaling, initial;
		unsigned int encPortCNT, encStarboardCNT;
		motor *ptr_port;
		motor *ptr_starboard;
		drive_status currentStatus;
	public:
		int current_heading;
		DriveMotor(motor& port, motor& starboard, 
				int scaling = 4, int initial = 10);
		int mapLine(bool left, bool middle, bool right);
		int lineMotorScaling();

		void incEncPortCNT();
		void incEncStarboardCNT();
		void resetEncCNT();
		unsigned int getEncPortCNT();
		unsigned int getEncStarboardCNT();
		
		void driveStraight(int speed = 255);
		void driveReverse(int speed = 255);
		void turnLeft(int speed = 255);
		void turnRight(int speed = 255);
		void pivotLeft();
		void pivotRight();
		drive_status get_status();
		void stop();
};
