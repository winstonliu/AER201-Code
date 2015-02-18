#include "Arduino.h"
#include "motor.h"

Motor::Motor(int pin_enable, int pin_direction)
{
	enable = pin_enable;
	direction = pin_direction;

	// Enable motor pins
	pinMode(enable, OUTPUT);
	pinMode(direction, OUTPUT);
}

void Motor::stop()
{
	status = OFF;
    digitalWrite(enable, LOW);
    digitalWrite(direction, LOW);    
}

void Motor::right(int speed)
{
	status = RIGHT;

	// PWM
	if (speed < 255)
		analogWrite(enable, speed);
	else
		digitalWrite(enable, HIGH);

    digitalWrite(direction, LOW);
}

void Motor::left(int speed)
{
	status = LEFT;

	// PWM
	if (speed < 255)
		analogWrite(enable, speed);
	else
		digitalWrite(enable, HIGH);

    digitalWrite(direction, HIGH);    
}

motor_states get_status() 
{ 
	return status; 
}
