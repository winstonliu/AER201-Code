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

void motor::right(int pwm)
{
	status = RIGHT;
    analogWrite(enable, pwm);
    digitalWrite(direction, LOW);
}

void motor::left(int pwm)
{
	status = LEFT;
    analogWrite(enable, pwm);
    digitalWrite(direction, HIGH);    
}

motor_states motor::get_status() 
{ 
	return status; 
}
