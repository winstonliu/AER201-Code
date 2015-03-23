#pragma once
#include <Arduino.h>
#include "nav.h"

namespace TM	// TaskManager
{
	// Flags
	extern bool FLAG_clawextended;
	extern bool FLAG_dockedboard;
	extern bool FLAG_pause;
	extern bool FLAG_hopperleft;
	extern bool FLAG_hopperright;

	extern int wheel_pwm;
	extern int clarm_pwm;

	extern int board_now;
	extern int timeforaline;

	extern double lineSep;
	extern unsigned int lineSepTicks;

	extern double Rw; // Wheel radii
	extern double D; // Wheel separation
	extern double Tr; // ticks per rotation

	extern int offGridTicks;
	extern int predockingheading;
	extern int internalcount;

	// Stuff from nav class
	extern DriveMotor* taskDriver;
	extern motor* taskClarm;
	extern motor* taskWheel;
	extern Nav* taskNav;	
	extern grid taskdestination;
	extern drcoord departingpoint;

	double euclideanDist(int x, int y);

	grid dirLineInc(int i);
	drcoord calcOffGrid(drcoord lastPos);

	void startTask(int& timer, grid& alfd, int& gg);
	void processTask(int& debug_speed);
	int interrupt(sensors sensor_interrupt);	
	bool checkTaskComplete();

	namespace GOG // 0  - Go on grid
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace MOG // 1  - Move on grid
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace MIR // 2  - Move in reverse
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace GFG // 3  - Go off grid
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace OOB // 4  - Off grid outbound
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace OGR // 5  - Off grid return
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace ROG // 6  - Rotate on grid
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace RFG // 7  - Rotate off grid
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace HAL // 8  - Hopper alignment
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace GAL // 9  - Gameboard alignment
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace CEX // 10 - Claw extend
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace CRT // 11 - Claw retract
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace PPP // 12 - Pause
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
	namespace MOI // 13 - Motion idle
	{
		void start();
		void process();
		void interrupt();
		void check();
	}
}
