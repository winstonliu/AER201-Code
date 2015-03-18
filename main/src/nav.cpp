#include "nav.h"

nav::nav(grid sp, DriveMotor& d, motor& c) : Driver(d), clarm(c)
{
	currentGrid = sp;	
	destination = sp;
	currentMotion = IDLE;

	FLAG_clawextended = true;
	FLAG_pause = false;
	FLAG_hopperleft = false;
	FLAG_hopperright = false;
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

grid nav::dirLineInc(int i)
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

	tasklist.push(task(PAUSE, 2000));
	tasklist.push(task(ROTATETO, next_xd)); // Rotate to face x
	tasklist.push(task(MOVEONGRID, difference.x)); // Move x
	tasklist.push(task(ROTATETO, next_yd)); // Rotate to face y
	tasklist.push(task(MOVEONGRID, difference.y)); // Move y
	tasklist.push(task(ROTATETO, destination.d)); // Rotate to face final

	return 0;
}

int nav::hopperBerthing()
{
	tasklist.push(task(MOVEOFFGRID, 0)); // Keep moving until interrupt
	tasklist.push(task(HOPPERALIGN, 0)); // Align with hopper
	tasklist.push(task(CLAWRETRACT, 0)); // Retract claw
	tasklist.push(task(MOVEOFFGRID, 0)); // Reverse
	tasklist.push(task(CLAWEXTEND, 0)); // Extend claw
}

void nav::startTask(int& timer)
{
	// Initialize tasks
	timer = 3600000; // default is 1 hour
	currentMotion = tasklist.peek().do_now;
	switch (currentMotion)
	{
		case PAUSE:
			timer = tasklist.peek().value;
			FLAG_pause = true;
			Driver.stop();
			break;
		case MOVEONGRID:
			Driver.driveStraight();
			taskdestination = dirLineInc(tasklist.peek().value);
			break;
		case MOVEOFFGRID:
			Driver.driveStraight();
			break;
		case ROTATETO:
			taskdestination = currentGrid;
			taskdestination.d = tasklist.peek().value;
			Driver.turnLeft();
			break;
		case CLAWRETRACT:
			clarm.right();
			break;
		case CLAWEXTEND:
			clarm.left();
			timer = 1000;
			break;
	}
}

void nav::processTask()
{
	// Non-time critical things that need to be done in the loop
	switch (currentMotion)
	{
		case MOVEONGRID:
			Driver.lineMotorScaling();	
			break;
	}
}

int nav::interrupt(sensors senInt)
{
	// Forward drive intersects line
	switch(senInt)
	{
		case LINE_ISR:
			if (currentMotion == MOVEONGRID)
			{
				grid new_grid = dirLineInc(1);
				if (check_validity(new_grid) == true) currentGrid = new_grid;
			}
			else if (currentMotion == ROTATETO)
			{
				// XXX Assuming that the robot only turns to the left
				currentGrid.d = (360 + currentGrid.d - 90) % 360;
			}
			break;
		case CLAW_TOUCH:
			if (currentMotion == CLAWRETRACT)
			{
				// kill claw motor		
				clarm.stop();
				FLAG_clawextended = false;
			}
			break;
		case HOPPER_TOUCH_LEFT:
			if (FLAG_hopperright == true)
				Driver.stop();
			else
				Driver.pivotLeft();
			FLAG_hopperleft = true;
			break;
		case HOPPER_TOUCH_RIGHT:
			if (FLAG_hopperleft == true)
				Driver.stop();
			else
				Driver.pivotRight();
			FLAG_hopperright = true;
			break;
		case TIMER:
			if (currentMotion == PAUSE) 
			{
				FLAG_pause = false;
			}
			else if (currentMotion == CLAWEXTEND)
			{
				clarm.stop();
				FLAG_clawextended = true;
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
			if (FLAG_pause == false) advance = true;	
			break;
		case MOVEONGRID:
			if (currentGrid.x == taskdestination.x &&
					currentGrid.y == taskdestination.y)
				advance = true;
			break;
		case ROTATETO:
			if (currentGrid == taskdestination) advance = true;
			break;
		case CLAWEXTEND: 
			if (FLAG_clawextended == true) { advance = true; }
			break;
		case CLAWRETRACT:
			if (FLAG_clawextended == false) { advance = true; }
			break;
		case HOPPERALIGN:
			if ((FLAG_hopperleft && FLAG_hopperright) == true) 
				advance = true;
			break;
		case MOVEOFFGRID:
			advance = true;
			break;
	}

	if (advance == true)
	{
		tasklist.pop(); 
		currentMotion = IDLE;
	}
	return advance;
}

bool nav::doneTasks() { return tasklist.count() == 0; }
int nav::countRemaining() { return tasklist.count(); }
motions nav::getMotion() { return currentMotion; }
grid nav::getGrid() { return currentGrid; }
grid nav::getDestination() { return destination; }
