#pragma once
#include "nav.h"

namespace TaskManager
{
	// Flags
	extern bool FLAG_clawextended;
	extern bool FLAG_pause;
	extern bool FLAG_hopperleft;
	extern bool FLAG_hopperright;

	// Stuff from nav class
	extern DriveMotor* taskDriver;
	extern motor* taskClarm;
	extern Nav* taskNav;	
	extern grid taskdestination;

	grid dirLineInc(int i);

	void startTask(int& timer);
	void processTask();
	int interrupt(sensors sensor_interrupt);	
	bool checkTaskComplete();
}
