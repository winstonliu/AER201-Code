#pragma once
#include <Arduino.h>
#include "nav.h"

namespace TaskManager
{
	// Flags
	extern bool FLAG_clawextended;
	extern bool FLAG_pause;
	extern bool FLAG_hopperleft;
	extern bool FLAG_hopperright;

	extern int Rw; // Wheel radii
	extern int D; // Wheel separation
	extern int Tr; // ticks per rotation

	// Stuff from nav class
	extern DriveMotor* taskDriver;
	extern motor* taskClarm;
	extern Nav* taskNav;	
	extern grid taskdestination;

	grid dirLineInc(int i);
	drcoord calcOffGrid(drcoord lastPos);

	void startTask(int& timer);
	void processTask();
	int interrupt(sensors sensor_interrupt);	
	bool checkTaskComplete();
}
