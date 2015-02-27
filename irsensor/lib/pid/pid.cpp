#include "Arduino.h"
#include "pid.h"

PID::PID(int& in, const int& set, int& out, double p, double i, double d, bool res)
{
	input = &in;
	setpoint = &set;
	output = &out;
	response = res;
	integral = 0;
	input_prev = 0;
	time_prev = 0;
	cycle_time = 0;
	low = 0;
	high = 255;
	on = 0;

	tune(p, i, d);
}

// clamp to external limits
void PID::clamp(int& parameter) {
	if (parameter > high) parameter = high;
	else if (parameter < low) parameter = low;	
}

bool PID::compute() {
	if (!on) return false;
	unsigned long now = millis();
	// after 110 days, millis() will reset to 0
	unsigned long elapsed = now - time_prev;

	// in the middle of a cycle, ignore
	if (elapsed < cycle_time) return false;
	// proportional
	int error = (int)(*setpoint) - (int)(*input);

	// integral
	integral += ki * error;
	clamp(integral);

	// derivative on measurement to fix derivative kick
	int input_change = (*input - input_prev);

	int out = kp*error + integral - kd*input_change;
	clamp(out);
	*output = out;

	// update state
	input_prev = *input;
	time_prev = now;
	return true;
}

void PID::tune(double p, double i, double d) {
	double cycle_time_sec = ((double)cycle_time) / 1000;
	kp = p;
	ki = i * cycle_time_sec;
	kd = d / cycle_time_sec;
	if (response == 0) {
		kp = -kp;
		ki = -ki;
		kd = -kd;
	} 
}

void PID::set_cycle(unsigned int new_cycle_time) {
	double adjust_ratio = ((double)new_cycle_time) / ((double)cycle_time);
	ki *= adjust_ratio;
	kd /= adjust_ratio;
	cycle_time = new_cycle_time;
}

void PID::set_limits(int l, int h) {
	// ignore troll value
	if (l >= h) return;
	low = l;
	high = h;
	// adjust exiting values
	if (on) {
		clamp(integral);
		clamp(*output);
	}
}

void PID::set_response(bool r) {
	if (on && r != response) {
		kp = -kp;
		ki = -ki;
		kd = -kd;
	}
	response = r;
}

void PID::start() {
	if (!on) {
		on = 1;
		reinitialize();
	}
}
void PID::stop() {on = 0;}

void PID::reinitialize() {
	input_prev = *input;
	integral = *output;
	clamp(integral);
}

bool PID::get_onoff() const {return on;}

bool PID::get_response() const {return response;}

