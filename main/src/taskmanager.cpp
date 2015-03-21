#include "taskmanager.h"

bool TaskManager::FLAG_clawextended = true;
bool TaskManager::FLAG_pause = false;
bool TaskManager::FLAG_hopperleft = false;
bool TaskManager::FLAG_hopperright = false;
grid TaskManager::taskdestination = taskNav->getDestination();

grid TaskManager::dirLineInc(int i)
{
	grid temp_grid = taskNav->getGrid();
	switch(temp_grid.d)
	{
		case 0:
			temp_grid.y += i;
			break;
		case 90:
			temp_grid.x += i;
			break;
		case 180:
			temp_grid.y -= i;
			break;
		case 270:
			temp_grid.x -= i;
			break;
	}
	return temp_grid;
}

drcoord TaskManager::calcOffGrid(drcoord lastPos)
{
	drcoord newPos = lastPos;
	unsigned int encPort = taskDriver->getEncPortCNT();
	unsigned int encStarboard = taskDriver->getEncStarboardCNT();
	// Turning to starboard is positive
	newPos.d = 2 * M_PI * (Rw/D) * (encPort - encStarboard) / Tr;
	newPos.x = Rw * cos(lastPos.d) * (encPort + encStarboard) * M_PI / Tr;
	newPos.y = Rw * sin(lastPos.d) * (encPort + encStarboard) * M_PI / Tr;
	// Reset count
	taskDriver->resetEncCNT();
	return newPos;
}

void TaskManager::startTask(int& timer, grid& alfd, int& gg)
{
	// Initialize tasks
	int navVal = taskNav->getValue();
	grid navGrid = taskNav->getGrid();
	switch (taskNav->getMotion())
	{
		case PAUSE:
			timer = navVal;
			FLAG_pause = true;
			taskDriver->stop();
			break;
		case MOVEONGRID:
			taskDriver->driveStraight();
			taskdestination = dirLineInc(navVal);
			break;
		case GOONGRID:
			taskNav->on_grid = true;
			break;
		case GOOFFGRID:
			taskDriver->resetEncCNT();
			taskNav->on_grid = false;
			break;
		case OFFGRIDOUTBOUND:
			taskDriver->driveStraight();
			break;
		case ROTATEONGRID:
			taskdestination = navGrid;
			taskdestination.d = navVal;
			// TODO only turns left for now
			taskDriver->turnLeft();
			break;
		case ROTATEOFFGRID:
			// Determine direction of rotation
			// Normalize current heading	
			navVal = (navVal - navGrid.d) % 360;
			if (navVal > 180) 
				taskDriver->turnLeft();
			else
				taskDriver->turnRight();
			break;
		case CLAWRETRACT:
			taskClarm->right();
			break;
		case CLAWEXTEND:
			taskClarm->left();
			timer = 1000;
			break;
	}
	alfd = taskdestination;
	gg = navVal;
}

void TaskManager::processTask()
{
	// Non-time critical things that need to be done in the loop
	switch (taskNav->getMotion())
	{
		case MOVEONGRID:
			taskDriver->lineMotorScaling();	
			break;
		case ROTATEOFFGRID:
		case OFFGRIDOUTBOUND:
			// Update off grid position
			taskNav->offgridpos = calcOffGrid(taskNav->offgridpos);
			break;
	}
}

int TaskManager::interrupt(sensors senInt)
{
	// Forward drive intersects line
	switch(senInt)
	{
		case LINE_ISR:
			if (taskNav->getMotion() == MOVEONGRID)
			{
				grid new_grid = dirLineInc(1);
				taskNav->setGrid(new_grid);
			}
			else if (taskNav->getMotion() == ROTATEONGRID)
			{
				// TODO Assuming that the robot only turns to the left
				grid new_grid = taskNav->getGrid();
				new_grid.d = (360 + new_grid.d - 90) % 360;
				taskNav->setGrid(new_grid);
			}
			break;
		case CLAW_TOUCH:
			if (taskNav->getMotion() == CLAWRETRACT)
			{
				// kill claw motor		
				taskClarm->stop();
				FLAG_clawextended = false;
			}
			break;
		case HOPPER_TOUCH_LEFT:
			if (FLAG_hopperright == true)
				taskDriver->stop();
			else
				taskDriver->pivotLeft();
			FLAG_hopperleft = true;
			break;
		case HOPPER_TOUCH_RIGHT:
			if (FLAG_hopperleft == true)
				taskDriver->stop();
			else
				taskDriver->pivotRight();
			FLAG_hopperright = true;
			break;
		case TIMER:
			if (taskNav->getMotion() == PAUSE) 
			{
				FLAG_pause = false;
			}
			else if (taskNav->getMotion() == CLAWEXTEND)
			{
				taskClarm->stop();
				FLAG_clawextended = true;
			}
			break;
	}
}

bool TaskManager::checkTaskComplete() 
{ 
	bool advance = false;
	int navVal = taskNav->getValue();
	grid gridNow = taskNav->getGrid();
	
	// Checks for task completion
	switch (taskNav->getMotion())
	{
		case PAUSE:
			if (FLAG_pause == false) advance = true;	
			break;
		case MOVEONGRID:
			if (gridNow.x == taskdestination.x &&
					gridNow.y == taskdestination.y)
				advance = true;
			break;
		case ROTATEONGRID:
			if (gridNow == taskdestination) advance = true;
			break;
		case ROTATEOFFGRID:
			if ((taskDriver->get_status() == TURNINGRIGHT 
					&& taskNav->offgridpos.d > navVal)
				|| (taskDriver->get_status() == TURNINGLEFT 
					&& taskNav->offgridpos.d < navVal))
			{
				advance = true;
			}
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
		case OFFGRIDOUTBOUND:
			if ((FLAG_hopperleft || FLAG_hopperright) == true) 
				advance = true;
			break;
		case GOOFFGRID:
		case GOONGRID:
			advance = true;
			break;
	}
	return advance;
}
