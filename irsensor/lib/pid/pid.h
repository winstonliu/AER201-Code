#pragma once

const bool OFF = 0;
const bool ON = 1;
const bool NEGATIVE = 0;
const bool POSITIVE = 1;

class PID {
	protected:
		// not going to change input and setpoint inside
		const int* input;
		const int* setpoint;
		int* output;
		// adjustable constants
		double kp, ki, kd;
		
		int integral, input_prev;
		unsigned long time_prev;
		unsigned int cycle_time;	// 100ms 0.1s
		// limits
		int low, high;

		// byte flags;
		bool on;
		// what's the relation between input and output the controller should expect
		// POSITIVE means increasing output causes increasing input
		bool response;

		// clamp to external limits
		void clamp(int& parameter);

	public:
		// by default off
		PID(int& in, const int& set, int& out, double p = 0.5, double i = 0, double d = 0, bool res = 1);

		bool compute();
		// change parameters (1/s)
		void tune(double p, double i, double d);
		// change cycle time
		void set_cycle(unsigned int new_cycle_time);
		// change output limits
		void set_limits(int l, int h);
		void set_response(bool r);

		void start();
		void stop();

		void reinitialize();

		bool get_onoff() const;
		bool get_response() const;
};
