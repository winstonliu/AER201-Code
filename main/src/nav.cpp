#include "nav.h"

nav::nav(grid sp, DriveMotor& d) : Driver(d)
{
	cycle_count = 0;
	currentGrid = sp;	
	destination = sp;
	currentMotion = IDLE;
}

bool nav::check_validity(grid coordinates)
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

int nav::reset(grid new_position)
{
	// FOR EMERGENCIES ONLY, reset grid coordinates
	if (nav::check_validity(new_position) == true)
	{
		currentGrid = new_position;
		return 0;
	}
	return -1;
}

int nav::set_destination(grid new_destination)
{
	// Set next destination coordinates of the robot
	if (nav::check_validity(new_destination) == true)
	{
		destination = new_destination;
		return 0;
	}
	return -1;
}

grid nav::directionalLineIncrement(int i)
{
	grid temp_grid = currentGrid;
	switch(currentGrid.d)
	{
		case 0:
			temp_grid.y + i;
			break;
		case 90:
			temp_grid.x + i;
			break;
		case 180:
			temp_grid.y - i;
			break;
		case 270:
			temp_grid.x - i;
			break;
	}
	return temp_grid;
}

int nav::computeRectilinearPath(grid new_destination)
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

	tasklist.push(task(PAUSE, 0));
	tasklist.push(task(ROTATETO, next_xd)); // Rotate to face x
	tasklist.push(task(MOVEONGRID, difference.x)); // Move x
	tasklist.push(task(ROTATETO, next_yd)); // Rotate to face y
	tasklist.push(task(MOVEONGRID, difference.y)); // Move y
	tasklist.push(task(ROTATETO, destination.d)); // Rotate to face final

	return 0;
}

int nav::hopperBerthing()
{
	tasklist.push(task(CLAWEXTEND, 0)); // Extend claw
	tasklist.push(task(MOVEOFFGRID, 0)); // Keep moving until interrupt
	tasklist.push(task(HOPPERALIGN, 0)); // Align with hopper
	tasklist.push(task(CLAWRETRACT, 0)); // Retract claw
	tasklist.push(task(MOVEOFFGRID, 0)); // Reverse
}

void nav::startTask()
{
	// Initialize tasks
	currentMotion = tasklist.peek().do_now;
	switch (currentMotion)
	{
		case PAUSE:
			// Pause for 40 cycles (2 seconds)
			pause_counter = 40;
			Driver.stop();
			break;
		case MOVEONGRID:
			Driver.driveStraight();
			taskdestination = directionalLineIncrement(tasklist.peek().value);
			break;
		case ROTATETO:
			taskdestination = currentGrid;
			taskdestination.d = tasklist.peek().value;
			Driver.driveInCircles();
			break;
	}
}

void nav::processTask()
{
	// Things that need to be done in the loop
	switch (currentMotion)
	{
		case PAUSE:
			--pause_counter;
			break;
		case MOVEONGRID:
			Driver.lineMotorScaling();	
			break;
		case ROTATETO:
			Driver.driveInCircles();
			break;
	}
}

int nav::interrupt(isr senInt)
{
	// Forward drive intersects line
	switch(senInt)
	{
		case LINE_ISR:
			if (currentMotion == MOVEONGRID)
			{
				grid new_grid = directionalLineIncrement(1);
				if (check_validity(new_grid) == true) currentGrid = new_grid;
			}
			else if (currentMotion == ROTATETO)
			{
				// XXX Assuming that the robot only turns to the left
				currentGrid.d = (360 + currentGrid.d - 90) % 360;
			}
			break;
		case TOUCH_ISR:
			if (currentMotion == CLAWRETRACT)
			{
				// kill claw motor		
			}
			break;
	}
}

bool nav::checkTaskComplete() 
{ 
	bool advance = false;
	// Checks for task completion
	switch (currentMotion)
	{
		case PAUSE:
			if (pause_counter <= 0) advance = true;	
			break;
		case MOVEONGRID:
		case ROTATETO:
			if (currentGrid == taskdestination) advance = true;
			break;
		case CLAWEXTEND: 
			if (FLAG_extended == true) advance = true;
			break;
		case CLAWRETRACT:
			// See ISR
			break;
	}

	if (advance == true)
	{
		tasklist.pop(); 
		currentMotion = IDLE;
	}
	return advance;
}

bool nav::doneTasks() { return tasklist.isEmpty(); }
int nav::countRemaining() { return tasklist.count(); }
motions nav::getMotion() { return currentMotion; }
grid nav::getGrid() { return currentGrid; }
grid nav::getDestination() { return destination; }
