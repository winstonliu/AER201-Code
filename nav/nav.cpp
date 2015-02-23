#include <cmath>
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

void nav::computePath()
{
	int next_xd;
	int next_yd;
	task thendo;
	grid difference;

	difference.x = abs(destination.x - current.x);
	difference.y = abs(destination.y - current.y);	

	// Calculate taxicab distance
	// XXX Does not take into account hopper locations
	
	if (difference.x < 0) // destination is WEST
	{
		next_xd = 270;
	}
	else if (difference.x > 0) // destination is EAST
	{
		next_xd = 90;
	}

	if (difference.y < 0) // destination is SOUTH
	{
		next_yd = 180;
	}
	else if (difference.x > 0) // destination is NORTH
	{
		next_yd = 0;
	}

	// Rotate to face x
	thendo.do_now = ROTATETO;		
	thendo.value = next_xd;
	taskMaster.push(thendo);

	// Move x
	thendo.do_now = MOVEFORWARD;
	thendo.value = difference.x;
	taskMaster.push(thendo);

	// Rotate to face y
	thendo.do_now = ROTATETO;		
	thendo.value = next_yd;
	taskMaster.push(thendo);

	// Move y
	thendo.do_now = MOVEFORWARD;
	thendo.value = difference.y;
	taskMaster.push(thendo);

	// Rotate to face final
	thendo.do_now = ROTATETO;		
	thendo.value = destination.d;
	taskMaster.push(thendo);
}

task nav::nextTask() { return taskMaster.front(); }
void nav::doneTask() { taskMaster.pop(); }

// Return current location
grid nav::get_current() { return current; }
