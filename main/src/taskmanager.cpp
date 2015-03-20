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

void TaskManager::startTask(int& timer)
{
	// Initialize tasks
	timer = 3600000; // default is 1 hour
	switch (taskNav->getMotion())
	{
		case PAUSE:
			timer = taskNav->getValue();
			FLAG_pause = true;
			taskDriver->stop();
			break;
		case MOVEONGRID:
			taskDriver->driveStraight();
			taskdestination = dirLineInc(taskNav->getValue());
			break;
		case MOVEOFFGRID:
			taskDriver->driveStraight();
			break;
		case ROTATETO:
			taskdestination = taskNav->getGrid();
			taskdestination.d = taskNav->getValue();
			// TODO only turns left for now
			taskDriver->turnLeft();
			break;
		case CLAWRETRACT:
			taskClarm->right();
			break;
		case CLAWEXTEND:
			taskClarm->left();
			timer = 1000;
			break;
	}
}

void TaskManager::processTask()
{
	// Non-time critical things that need to be done in the loop
	switch (taskNav->getMotion())
	{
		case MOVEONGRID:
			taskDriver->lineMotorScaling();	
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
			else if (taskNav->getMotion() == ROTATETO)
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
		case ROTATETO:
			if (gridNow == taskdestination) advance = true;
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
	return advance;
}
