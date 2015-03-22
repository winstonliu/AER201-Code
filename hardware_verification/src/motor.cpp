#include "Arduino.h"
#include "motor.h"

motor::motor(int pin_enable, int pin_direction, int speed)
{
	enable = pin_enable;
	direction = pin_direction;
	motorspeed = speed;
	status = MOTOR_OFF;

	// Enable motor pins
	pinMode(enable, OUTPUT);
	pinMode(direction, OUTPUT);
}

void motor::stop()
{
	status = MOTOR_OFF;
    digitalWrite(enable, LOW);
    digitalWrite(direction, LOW);    
}

void motor::right(int speed)
{
	status = MOTOR_RIGHT;

	// PWM
	if (speed == -42)
	{
		analogWrite(enable, motorspeed);
	}
	else
	{
		digitalWrite(enable, speed);
		motorspeed = speed;
	}

    digitalWrite(direction, LOW);
}

void motor::left(int speed)
{
	status = MOTOR_LEFT;

	// PWM
	if (speed == -42)
	{
		analogWrite(enable, motorspeed);
	}
	else
	{
		digitalWrite(enable, speed);
		motorspeed = speed;
	}
    digitalWrite(direction, HIGH);    
}

void motor::adjustSpeed(int speed)
{
	motorspeed = speed;
	switch(status)
	{
		case MOTOR_RIGHT:
			right(speed);
			break;
		case MOTOR_LEFT:
			left(speed);
			break;
	}	
}

void motor::reverseDirection()
{
	switch(status)
	{
		case MOTOR_RIGHT:
			left(motorspeed);
			break;
		case MOTOR_LEFT:
			right(motorspeed);
			break;
	}	
}

motor_states motor::get_status() 
{ 
	return status; 
}
