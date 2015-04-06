#include "drivemotor.h"

DriveMotor::DriveMotor(motor& p, motor& s, double kp, double kd) :
	prop(kp), deriv(kd)
{
	ptr_port = &p;
	ptr_starboard = &s;
	currentStatus = STOPPED;
}

int DriveMotor::mapLine(bool l, bool m, bool r) 
{
	//static int current_heading = 0;
	// positive is right
	// false is 0, true is 1
	
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

	return current_heading;
}

int DriveMotor::lineMotorScaling(int baseSpeed)
{
	// If the heading is less than zero, then PWM the port (left) wheel and
	// v.v. If the heading is zero, then full steam ahead.
	
	// For four pins
	static int lastError = 0;
	int error = current_heading - 3500;
	newSpeed = prop * error + deriv * (error - lastError);
	lastError = error;

	ptr_port->adjustSpeed(baseSpeed + newSpeed);
	ptr_starboard->adjustSpeed(baseSpeed - newSpeed);

	/*
	int newSpeed;
	if (current_heading != 0)
	{
		newSpeed = baseSpeed - current_heading*scaling*initial;
		//ptr_port->adjustSpeed(newSpeed); 
		if (newSpeed < 0) newSpeed = 0;
		ptr_port->adjustSpeed(newSpeed);
		ptr_starboard->adjustSpeed(baseSpeed);
	else
	{
		newSpeed = baseSpeed;
		ptr_port->adjustSpeed(newSpeed);
		ptr_starboard->adjustSpeed(newSpeed);
	}
	return newSpeed;
	*/

	return newSpeed;
}

void DriveMotor::driveStraight(int speed)
{
	ptr_port->left(speed);
	ptr_starboard->right(speed);
	currentStatus = DRIVINGFORWARD;
}

void DriveMotor::driveReverse(int speed)
{
	ptr_starboard->left(speed);
	ptr_port->right(speed);
	currentStatus = DRIVINGREVERSE;
}

void DriveMotor::turnLeft(int speed)
{
	// Turn left
	ptr_starboard->right(speed);
	ptr_port->right(speed);
	currentStatus = TURNINGLEFT;
}

void DriveMotor::turnRight(int speed)
{
	// Turn right
	ptr_starboard->left(speed);
	ptr_port->left(speed);
	currentStatus = TURNINGRIGHT;
}

void DriveMotor::pivotLeft(int speed)
{
	ptr_starboard->right(speed);
	ptr_port->stop();
	currentStatus = PIVOTLEFT;
}

void DriveMotor::pivotLeftReverse()
{
	ptr_starboard->left(200);
	ptr_port->stop();
	currentStatus = PIVOTLEFTBACK;
}

void DriveMotor::pivotRight(int speed)
{
	ptr_starboard->stop();
	ptr_port->left(speed);
	currentStatus = PIVOTRIGHT;
}

void DriveMotor::pivotRightReverse()
{
	ptr_starboard->stop();
	ptr_port->right(200);
	currentStatus = PIVOTRIGHTBACK;
}

void DriveMotor::adjustSpeed(int speed)
{
	switch(currentStatus)
	{
		case DRIVINGFORWARD:
		case DRIVINGREVERSE:
		case TURNINGRIGHT:
		case TURNINGLEFT:
			ptr_starboard->adjustSpeed(speed);
			ptr_port->adjustSpeed(speed);
			break;
		case PIVOTLEFT:
			ptr_starboard->adjustSpeed(speed * 0.8);
			break;
		case PIVOTRIGHT:
			ptr_port->adjustSpeed(speed * 0.8);
			break;
	}
}

void DriveMotor::doOpposite()
{
	switch(currentStatus)
	{
		case DRIVINGFORWARD:
			this->driveReverse();
			break;
		case TURNINGRIGHT:
			this->turnLeft();
			break;
		case TURNINGLEFT:
			this->turnRight();
			break;
		case PIVOTLEFT:
			this->pivotRight();
			break;
		case PIVOTRIGHT:
			this->pivotLeft();
			break;
	}	
}

void DriveMotor::stop()
{
	currentStatus = STOPPED;	
	ptr_starboard->stop();
	ptr_port->stop();
}

drive_status DriveMotor::get_status() { return currentStatus; } 
