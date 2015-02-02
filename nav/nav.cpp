#include "nav.h"

nav::nav(grid start_position)
{
	current = start_position;	
	destination = start_position;
}

// Check the validity of grid coordinates
bool nav::check_validity(grid coordinates)
{
	if (grid.x < 1 || grid.x > 7)
		return false;
	else if (grid.y < 1 || grid.y > 8)
		return false;
	else if (grid.d < 0 || grid.d > 359)
		return false;
	else
		return true;
}

// Sensory input events
int nav::interrupt(isr action)
{
	switch(action)
	{
		case LINE_ISR:
			break;
		case TOUCH_ISR:
			break;
	}	
}

// FOR EMERGENCIES ONLY, reset grid coordinates
int nav::reset(grid new_position)
{
	if (nav::check_validity(new_position) == true)
	{
		current = new_position;
		return 0;
	}
	return -1;
}

// Set next destination coordinates of the robot
int nav::set_destination(grid new_destination)
{
	if (nav::check_validity(new_destination) == true)
	{
		destination = new_destination;
		return 0;
	}
	return -1;
}

// Return current location
grid get_current() { return current; }
