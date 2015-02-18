#include "line_pid.h"

int mapLinePid(bool l, bool m, bool r) 
{
	int state;
	//static int state = 0;
	// false is 0, true is 1
	
	/*
	// 000
	if ((l|m|r) == ON_WHITE)
		state = ((state > 0) ? 3 : -3);
	*/
	// 110
	if ((!l|!m|r) == ON_WHITE)
		state = 2;
	// 001
	else if ((l|m|!r) == ON_WHITE)
		state = 1;
	// 010 or 111
	else if ((l|!m|r) == ON_WHITE or (!l|!m|!r) == ON_WHITE)
		state = 0;
	// 100
	else if ((!l|m|r) == ON_WHITE)
		state = -1;
	// 011
	else if ((l|!m|!r) == ON_WHITE)
		state = -2;

	return state;
}
