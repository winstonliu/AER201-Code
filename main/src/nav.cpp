#include "nav.h"

Nav::Nav(grid sp) : currentGrid(sp), destination(sp) {}

bool Nav::check_validity(grid coordinates)
{
	// Check the validity of grid coordinates
	if (coordinates.x < 1 || coordinates.x > 7)
		return false;
	else if (coordinates.y < 1 || coordinates.y > 8)
		return false;
	else if (coordinates.d < 0 || coordinates.d > 359)
		return false;
	else
		return true;
}

int Nav::reset(grid new_position)
{
	// FOR EMERGENCIES ONLY, reset grid coordinates
	if (Nav::check_validity(new_position) == true)
	{
		currentGrid = new_position;
		return 0;
	}
	return -1;
}

int Nav::set_destination(grid new_destination)
{
	// Set next destination coordinates of the robot
	if (Nav::check_validity(new_destination) == true)
	{
		destination = new_destination;
		return 0;
	}
	return -1;
}

int Nav::computeRectilinearPath(grid new_destination)
{
	// Check validity and set destination
	if (set_destination(new_destination) == -1)
		return -1;

	// Computes the path the robot will take
	int next_xd;
	int next_yd;
	grid difference;

	difference.x = destination.x - currentGrid.x;
	difference.y = destination.y - currentGrid.y;	

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

	tasklist.push(task(PAUSE, 2000));
	tasklist.push(task(ROTATETO, next_xd)); // Rotate to face x
	tasklist.push(task(MOVEONGRID, difference.x)); // Move x
	tasklist.push(task(ROTATETO, next_yd)); // Rotate to face y
	tasklist.push(task(MOVEONGRID, difference.y)); // Move y
	tasklist.push(task(ROTATETO, destination.d)); // Rotate to face final

	return 0;
}

int Nav::hopperBerthing()
{
	tasklist.push(task(MOVEOFFGRID, 0)); // Keep moving until interrupt
	tasklist.push(task(HOPPERALIGN, 0)); // Align with hopper
	tasklist.push(task(CLAWRETRACT, 0)); // Retract claw
	tasklist.push(task(MOVEOFFGRID, 0)); // Reverse
	tasklist.push(task(CLAWEXTEND, 0)); // Extend claw
}

int Nav::setGrid(grid new_grid)
{
	if (check_validity(new_grid) == true)
	{
		currentGrid = new_grid;
		return 0;
	}
	else
	{
		return -1;
	}
}

void Nav::advance() { tasklist.pop(); }
motions Nav::getMotion() 
{ 
	return tasklist.isEmpty() ? IDLE : tasklist.peek().do_now; 
}
int Nav::getValue()
{
	return tasklist.isEmpty() ? 0 : tasklist.peek().value; 
}
grid Nav::getGrid() { return currentGrid; }
grid Nav::getDestination() { return destination; }
bool Nav::doneTasks() { return tasklist.count() == 0; }
int Nav::countRemaining() { return tasklist.count(); }
