#include "Arduino.h"
#include "motor.h"

motor::motor(int pin_enable, int pin_direction)
{
	enable = pin_enable;
	direction = pin_direction;

	// Enable motor pins
	pinMode(enable, OUTPUT);
	pinMode(direction, OUTPUT);
}

void motor::stop()
{
	status = OFF;
    digitalWrite(enable, LOW);
    digitalWrite(direction, LOW);    
}

void motor::right()
{
	status = RIGHT;
    digitalWrite(enable, HIGH);
    digitalWrite(direction, LOW);
}

void motor::left()
{
	status = LEFT;
    digitalWrite(enable, HIGH);
    digitalWrite(direction, HIGH);    
}

motor_states get_status() 
{ 
	return status; 
}
