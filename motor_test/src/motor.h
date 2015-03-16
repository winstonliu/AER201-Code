#pragma once

enum motor_states
{
	MOTOR_RIGHT,
	MOTOR_LEFT,
	MOTOR_OFF
};

class motor
{
	private:
		int enable;
		int direction;
		int motorspeed;
		motor_states status;
	public:
		motor() {}
		motor(int pin_enable, int pin_direction, int speed = 255);	
		void stop();
		// LOW direction pin implies rightward rotation
		void right(int speed = -42);
		// HIGH direction pin implies leftward rotation
		void left(int speed = -42);
		void adjustSpeed(int speed = 255);
		void reverseDirection();
		motor_states get_status();
};
