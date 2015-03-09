#pragma once
#include "motor.h"

const int ON_WHITE = false;

class DriveMotor
{
	private:
		int current_heading;	
		int scaling, initial;
		motor port, starboard;
	public:
		DriveMotor(motor& port, motor& starboard, 
				int scaling = 4, int initial = 10);
		int mapLine(bool left, bool middle, bool right);
		int lineMotorScaling();
		void driveStraight();
		void driveInCircles();
};

