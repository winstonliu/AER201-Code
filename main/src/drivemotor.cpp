#include "drivemotor.h"

DriveMotor::DriveMotor(motor& p, motor& s, int ds, int di) : 
	port(p), starboard(s), scaling(ds), initial(di) 
{
	encPortCNT = 0;
	encStarboardCNT = 0;
	currentStatus = STOPPED;
}

int DriveMotor::mapLine(bool l, bool m, bool r) 
{
	//static int current_heading = 0;
	// positive is right
	// false is 0, true is 1
	
	current_heading = 0;	
	/*
	// 000
	if ((l|m|r) == ON_WHITE)
		current_heading = ((current_heading > 0) ? 3 : -3);
	// 011
	if ((l|!m|!r) == ON_WHITE)
		current_heading = 2;
	// 001
	else if ((l|m|!r) == ON_WHITE)
		current_heading = 1;
	// 010 or 111 or 000
	else if ((l|!m|r) == ON_WHITE 
			|| (!l|!m|!r) == ON_WHITE 
			|| (l|m|r) == ON_WHITE)
		current_heading = 0;
	// 100
	else if ((!l|m|r) == ON_WHITE)
		current_heading = -1;
	// 110
	else if ((!l|!m|r) == ON_WHITE)
		current_heading = -2;
	*/

	return current_heading;
}

int DriveMotor::lineMotorScaling(int& mestatus)
{
	// If the heading is less than zero, then PWM the port (left) wheel and
	// v.v. If the heading is zero, then full steam ahead.
	// DEBUG
	int adjustedSpeed;
	if (current_heading < 0)
	{
		adjustedSpeed = 255 - initial - (-1)*current_heading*scaling;
		port.adjustSpeed(adjustedSpeed); 
		//port.left(adjustedSpeed);
		mestatus = 0;
	}
	else if (current_heading > 0)
	{
		adjustedSpeed = 255 - initial - current_heading*scaling;
		starboard.adjustSpeed(adjustedSpeed); 
		//starboard.right(adjustedSpeed);
		mestatus = 1;
	}
	else
	{
		adjustedSpeed = 255;
		port.adjustSpeed(adjustedSpeed);
		starboard.adjustSpeed(adjustedSpeed);
		mestatus = 2;
	}
	return adjustedSpeed;
}

void DriveMotor::incEncPortCNT() { ++encPortCNT; }
void DriveMotor::incEncStarboardCNT() { ++encStarboardCNT; }
void DriveMotor::resetEncCNT() 
{ 
	encPortCNT = 0; 
	encStarboardCNT = 0;
}
unsigned int DriveMotor::getEncPortCNT() { return encPortCNT; }
unsigned int DriveMotor::getEncStarboardCNT() { return encStarboardCNT; }

void DriveMotor::driveStraight(int speed)
{
	port.left(speed);
	starboard.right(speed);
	currentStatus = DRIVINGFORWARD;
}

void DriveMotor::driveReverse(int speed)
{
	starboard.left(speed);
	port.right(speed);
	currentStatus = DRIVINGREVERSE;
}

void DriveMotor::turnLeft(int speed)
{
	// Turn left
	starboard.right(speed);
	port.right(speed);
	currentStatus = TURNINGLEFT;
}

void DriveMotor::turnRight(int speed)
{
	// Turn right
	starboard.left(speed);
	port.left(speed);
	currentStatus = TURNINGRIGHT;
}

void DriveMotor::pivotLeft()
{
	starboard.right(125);
	port.stop();
	currentStatus = PIVOTLEFT;
}

void DriveMotor::pivotRight()
{
	starboard.stop();
	port.left(125);
	currentStatus = PIVOTRIGHT;
}

void DriveMotor::stop()
{
	currentStatus = STOPPED;	
	starboard.stop();
	port.stop();
}

drive_status DriveMotor::get_status() { return currentStatus; } 
