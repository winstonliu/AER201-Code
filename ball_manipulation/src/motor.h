#pragma once

enum motor_states
{
	RIGHT,
	LEFT,
	OFF
};

class motor
{
	private:
		int enable;
		int direction;
		motor_states status;
	public:
		motor(int pin_enable, int pin_direction);	
		void stop();
		// LOW direction pin implies rightward rotation
		void right(int pwm);
		// HIGH direction pin implies leftward rotation
		void left(int pwm);
		motor_states get_status();
};
