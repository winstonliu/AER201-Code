#pragma once

enum motor_states
{
	RIGHT,
	LEFT,
	OFF
}

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
		void right();
		// HIGH direction pin implies leftward rotation
		void left();
		motor_states get_status();
}
