#include "nav.h"

Nav::Nav(grid sp) : currentGrid(sp), destination(sp) 
{
	encPortCNT = 0;
	encStarboardCNT = 0;
	lastpcnt = 0;
	lastscnt = 0;
	encPortLOG = 0;
	encStarboardLOG = 0;
	hopperEast = grid(0,0,0);
	hopperWest = grid(0,0,0);
	offgridpos = drcoord(sp.x, sp.y, sp.d);
}

long Nav::timeElapsed() { return currentTime - sketchyTimer; }

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

	tasklist.push(task(PPP, 2000));
	tasklist.push(task(ROG, next_xd)); // Rotate to face x
	tasklist.push(task(MOG, difference.x)); // Move x
	tasklist.push(task(ROG, next_yd)); // Rotate to face y
	tasklist.push(task(MOG, difference.y)); // Move y
	tasklist.push(task(ROG, destination.d)); // Rotate to face final

	return 0;
}

void Nav::incEncPortCNT() { ++encPortCNT; }
void Nav::incEncStarboardCNT() { ++encStarboardCNT; }
void Nav::resetEncCNT() 
{ 
	encPortLOG += encPortCNT;
	encStarboardLOG += encStarboardCNT;
	lastpcnt = encPortLOG;
	lastscnt = encStarboardLOG;
	encPortCNT = 0; 
	encStarboardCNT = 0;
}
int Nav::getEncPortCNT() { return encPortCNT; }
int Nav::getEncStarboardCNT() { return encStarboardCNT; }
bool Nav::spikeCheck()
{
	if ((abs(encPortLOG - lastpcnt) > 100)
		||(abs(encStarboardLOG - lastscnt) > 100))
	{
		lastpcnt = encPortLOG;
		lastscnt = encStarboardLOG;
		return true;
	}
	return false;
}
int Nav::absEncDistance()
{
	// Return the absolute distance
	return abs((int)sqrt(offgridpos.x*offgridpos.x 
				+ offgridpos.y*offgridpos.y)); 
}
void Nav::zeroOGXY() 
{ 
	offgridpos.x = 0; 
	offgridpos.y = 0; 
	//offgridpos.d = 0; 
}
void Nav::setOGPtoGrid()
{
	offgridpos.d = currentGrid.d;
}
int Nav::setGrid(grid new_grid)
{
	if (check_validity(new_grid) == true)
	{
		//setOffGridPos(new_grid.d);
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
	return tasklist.isEmpty() ? MOI : tasklist.peek().do_now; 
}
int Nav::getTaskValue()
{
	return tasklist.isEmpty() ? 0 : tasklist.peek().value; 
}
grid Nav::getGrid() { return currentGrid; }
void Nav::setOffGridPos(drcoord newpos)
{
	newpos.d = fmod((720 + newpos.d), 360);
	offgridpos = newpos;
}
void Nav::setOffGridPos(double d)
{
	offgridpos.d = fmod((720 + d), 360);
}
drcoord Nav::getOffGridPos() { return offgridpos; }
grid Nav::getDestination() { return destination; }
bool Nav::doneTasks() { return tasklist.count() == 0; }
int Nav::countRemaining() { return tasklist.count(); }

void Nav::processMM()
{
	static int lastMM = mMTC;	
	// Check for change
	if (lastMM == currentMM)
		return;

	switch(currentMM)
	{
		case mMTC:
			tasklist.push(task(PPP, 3000));		
			tasklist.push(task(RFG, 90));		
			tasklist.push(task(PPP, 500));		
			tasklist.push(task(MOG, 2));		
			tasklist.push(task(PPP, 500));		
			lineAlign();
			tasklist.push(task(PPP, 500));		
			tasklist.push(task(RFG, 0));		
			tasklist.push(task(PPP, 500));		
			tasklist.push(task(MOG, 1));		
			tasklist.push(task(PPP, 500));		
			tasklist.push(task(RFG, 90));		
			tasklist.push(task(PPP, 500));		
			tasklist.push(task(RFG, 135));		
			tasklist.push(task(PPP, 500));		
			break;
		case mCBL:
			hopperDocking();
			tasklist.push(task(PPP, 500));		
			hopperUndocking();
			break;
		case mMTB:
			boardAndBack();
			break;
	}		

	lastMM = currentMM;
}

int Nav::hopperDocking()
{
	//tasklist.push(task(RFG, 315)); // Move until interrupt
	tasklist.push(task(HAL, 0)); // Align with hopper
	tasklist.push(task(CRT, 0)); // Retract claw
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

	tasklist.push(task(OGR, 0));
	tasklist.push(task(CEX, 500)); // Extend claw
	tasklist.push(task(RFG, currentGrid.d));
}
int Nav::gameboardAlign()
{
	// at 4,8,90/180
	tasklist.push(task(ROG, 180)); // Rotate to face y
	tasklist.push(task(GAL, 0));
	tasklist.push(task(PPP, 1000));
}
int Nav::boardAndBack()
{
	tasklist.push(task(MOG, 1));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(RFG, 0));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(MOG, 6));
	tasklist.push(task(PPP, 1000));
	lineAlign();
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(RFG, 270));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(MOG, 3));
	gameboardAlign();
	// Return 
	tasklist.push(task(MOG, 1));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(RFG, 90));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(MOG, 3));
	tasklist.push(task(PPP, 1000));
	lineAlign();
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(RFG, 180));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(MOG, 6));
	tasklist.push(task(PPP, 1000));
	lineAlign();
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(RFG, 270));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(MOG, 1));
	tasklist.push(task(PPP, 1000));
	tasklist.push(task(RFG, 135));
}
void Nav::lineAlign()
{
	tasklist.push(task(MTL, -8));
	tasklist.push(task(PPP, 500));
	tasklist.push(task(MTL, 6));
	tasklist.push(task(PPP, 500));
	/*
	tasklist.push(task(RTL, 25));
	tasklist.push(task(PPP, 500));
	tasklist.push(task(RTL, -50));
	*/
}
void Nav::rotateAlign()
{
	tasklist.push(task(RTL, -10));
	tasklist.push(task(PPP, 500));
	tasklist.push(task(RTL, 10));
	tasklist.push(task(PPP, 500));
	tasklist.push(task(RTL, -45));
	tasklist.push(task(PPP, 500));
	tasklist.push(task(RTL, 90));
}


