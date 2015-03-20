#include "drivemotor.h"

DriveMotor::DriveMotor(motor& p, motor& s, int ds, int di) : 
	port(p), starboard(s), scaling(ds), initial(di) 
{
	encPortCNT = 0;
	encStarboardCNT = 0;
}

int DriveMotor::mapLine(bool l, bool m, bool r) 
{
	//static int current_heading = 0;
	// false is 0, true is 1
	
	/*
	// 000
	if ((l|m|r) == ON_WHITE)
		current_heading = ((current_heading > 0) ? 3 : -3);
	*/
	// 110
	if ((!l|!m|r) == ON_WHITE)
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
	// 011
	else if ((l|!m|!r) == ON_WHITE)
		current_heading = -2;

	return current_heading;
}

int DriveMotor::lineMotorScaling()
{
	// If the heading is less than zero, then PWM the port (left) wheel and
	// v.v. If the heading is zero, then full steam ahead.
	// DEBUG
	int adjustedSpeed;
	if (current_heading < 0)
	{
		adjustedSpeed = 255 - initial - (-1)*current_heading*scaling;
		//port.adjustSpeed(adjustedSpeed); 
		port.left(adjustedSpeed);
	}
	else if (current_heading > 0)
	{
		adjustedSpeed = 255 - initial - current_heading*scaling;
		//starboard.adjustSpeed(adjustedSpeed); 
		starboard.right(adjustedSpeed);
	}
	else
	{
		adjustedSpeed = 255;
		port.adjustSpeed(adjustedSpeed);
		starboard.adjustSpeed(adjustedSpeed);
	}
	return adjustedSpeed;
}

void DriveMotor::incEncPortCNT() { ++encPortCNT; }
void DriveMotor::incEncStarboardCNT() { ++encStarboardCNT; }
void DriveMotor::resetEncPortCNT() { encPortCNT = 0; }
void DriveMotor::resetEncStarboardCNT() { encStarboardCNT = 0; }
unsigned int DriveMotor::getEncPortCNT() { return encPortCNT; }
unsigned int DriveMotor::getEncStarboardCNT() { return encStarboardCNT; }

void DriveMotor::driveStraight()
{
	port.left(255);
	starboard.right(255);
	currentStatus = DRIVINGFORWARD;
}

void DriveMotor::turnLeft()
{
	// Turn left
	if (starboard.get_status() != MOTOR_RIGHT 
		&& port.get_status() != MOTOR_RIGHT)
	{
		starboard.right(255);
		port.right(255);
		currentStatus = TURNINGLEFT;
	}
}

void DriveMotor::turnRight()
{
	// Turn right
	if (starboard.get_status() != MOTOR_LEFT 
		&& port.get_status() != MOTOR_LEFT)
	{
		starboard.left(255);
		port.left(255);
		currentStatus = TURNINGRIGHT;
	}
}

void DriveMotor::pivotLeft()
{
	if (starboard.get_status() != MOTOR_LEFT 
		&& port.get_status() != MOTOR_OFF)
	{
		starboard.right(125);
		port.stop();
		currentStatus = PIVOTLEFT;
	}
}

void DriveMotor::pivotRight()
{
	if (starboard.get_status() != MOTOR_OFF 
		&& port.get_status() != MOTOR_LEFT)
	{
		starboard.stop();
		port.left(125);
		currentStatus = PIVOTRIGHT;
	}
}

void DriveMotor::stop()
{
	port.stop();
	starboard.stop();
}

drive_status DriveMotor::get_status() { return currentStatus; } 
