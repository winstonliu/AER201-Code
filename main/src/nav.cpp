#include "nav.h"

Nav::Nav(grid sp) : currentGrid(sp), destination(sp) 
{
	encPortCNT = 0;
	encStarboardCNT = 0;
	hopperEast = grid(0,0,0);
	hopperWest = grid(0,0,0);
	offgridpos = drcoord(0,0,0);
}

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
	tasklist.push(task(ROTATEONGRID, next_xd)); // Rotate to face x
	tasklist.push(task(MOVEONGRID, difference.x)); // Move x
	tasklist.push(task(ROTATEONGRID, next_yd)); // Rotate to face y
	tasklist.push(task(MOVEONGRID, difference.y)); // Move y
	tasklist.push(task(ROTATEONGRID, destination.d)); // Rotate to face final

	return 0;
}

int Nav::hopperDocking()
{
	tasklist.push(task(ROTATEOFFGRID, 315)); // Move until interrupt
	tasklist.push(task(OFFGRIDOUTBOUND, 0)); // Move until interrupt
	tasklist.push(task(HOPPERALIGN, 0)); // Align with hopper
	tasklist.push(task(CLAWRETRACT, 0)); // Retract claw
	tasklist.push(task(OFFGRIDRETURN, 0)); // Reverse
	tasklist.push(task(CLAWEXTEND, 0)); // Extend claw
}

int Nav::hopperUndocking()
{
	// Using cosine law to calculate degrees of rotation
	// It's a triangle
	/*
	int len_b = 10;
	double len_c = sqrt(len_a*len_a + len_b*len_b 
			- 2*len_a*len_b*cos(angleC));
	int angleA = floor(acos((len_b*len_b + len_c*len_c - len_a*len_a)
		/ (2*len_b*len_c)));
	*/

	// negative implies use counted lines
	tasklist.push(task(OFFGRIDRETURN, -1));
	tasklist.push(task(ROTATEOFFGRID, currentGrid.d)); // Move until interrupt
}

int Nav::gameBoardDocking()
{
}
void Nav::incEncPortCNT() { ++encPortCNT; }
void Nav::incEncStarboardCNT() { ++encStarboardCNT; }
void Nav::resetEncCNT() 
{ 
	encPortCNT = 0; 
	encStarboardCNT = 0;
}
unsigned int Nav::getEncPortCNT() { return encPortCNT; }
unsigned int Nav::getEncStarboardCNT() { return encStarboardCNT; }
unsigned int Nav::absEncDistance()
{
	// Return the absolute distance
	return abs((int)sqrt(offgridpos.x*offgridpos.x 
				+ offgridpos.y*offgridpos.y)); 
}
void Nav::resetOffGridToZero() 
{ 
	offgridpos.x = 0; 
	offgridpos.y = 0; 
	offgridpos.d = 0; 
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
	return tasklist.isEmpty() ? MOTIONIDLE : tasklist.peek().do_now; 
}
int Nav::getTaskValue()
{
	return tasklist.isEmpty() ? 0 : tasklist.peek().value; 
}
grid Nav::getGrid() { return currentGrid; }
grid Nav::getDestination() { return destination; }
bool Nav::doneTasks() { return tasklist.count() == 0; }
int Nav::countRemaining() { return tasklist.count(); }
