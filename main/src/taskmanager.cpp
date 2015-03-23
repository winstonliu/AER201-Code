#include "taskmanager.h"

bool TM::FLAG_clawextended = true;
bool TM::FLAG_dockedboard = false;
bool TM::FLAG_pause = false;
bool TM::FLAG_hopperleft = false;
bool TM::FLAG_hopperright = false;
grid TM::taskdestination = taskNav->getDestination();
drcoord TM::departingpoint = taskNav->offgridpos;

int TM::predockingheading = 0;
int TM::internalcount = 0;

double TM::euclideanDist(int x, int y) { return sqrt(x*x + y*y); }

grid TM::dirLineInc(int i)
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

drcoord TM::calcOffGrid(drcoord lastPos)
{
	drcoord newPos = lastPos;
	unsigned int encPort = taskNav->getEncPortCNT();
	unsigned int encStarboard = taskNav->getEncStarboardCNT();
	// Turning to starboard is positive
	newPos.d = 2 * M_PI * (Rw/D) * (encPort - encStarboard) / Tr;
	newPos.x = Rw * cos(lastPos.d) * (encPort + encStarboard) * M_PI / Tr;
	newPos.y = Rw * sin(lastPos.d) * (encPort + encStarboard) * M_PI / Tr;
	// Reset count
	taskNav->resetEncCNT();
	return newPos;
}

void TM::startTask(int& timer, grid& alfd, int& gg)
{
	// Initialize tasks
	int navVal = taskNav->getTaskValue();
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
		case OFFGRIDRETURN:
			if (navVal > 0)
				internalcount = navVal;
		case MOVEINREVERSE:
			//timer = navVal;
			taskDriver->driveReverse();
			break;
		case OFFGRIDOUTBOUND:
			//predockingheading = taskNav->offgridpos.d;
			taskDriver->driveStraight();
			break;
		case HOPPERALIGN:
			taskDriver->driveStraight();
			break;
		case ROTATEONGRID:
			taskdestination = navGrid;
			taskdestination.d = navVal;
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
			taskClarm->left(clarm_pwm);
			wheel_pwm = 0;
			break;
		case CLAWEXTEND:
			taskClarm->right(clarm_pwm);
			wheel_pwm = 0;
			timer = navVal;
			break;
		case GAMEBOARDALIGN:
			taskDriver->pivotLeftReverse();
			break;
		// Grid ON, OFF	
		case GOONGRID:
			taskNav->on_grid = true;
			break;
		case GOOFFGRID:
			taskNav->on_grid = false;
			break;
	}
	alfd = taskdestination;
	gg = navVal;
}

void TM::processTask(int& debug_speed)
{
	int baseSpeed = 255;
	// Non-time critical things that need to be done in the loop
	switch (taskNav->getMotion())
	{
		case MOVEONGRID:
			/*
			if (taskNav->absEncDistance() >= floor(lineSepTicks * 0.75))
				baseSpeed = 125;
			else
				baseSpeed = 255;
			debug_speed = taskDriver->lineMotorScaling(baseSpeed);
			*/
			/*
			if (taskNav->timeElapsed() >= timeforaline) 
				baseSpeed = 125;
			else
				baseSpeed = 255;
			*/
			debug_speed = taskDriver->lineMotorScaling(baseSpeed);
			break;
		case ROTATEOFFGRID:
		case OFFGRIDOUTBOUND:
			// Update off grid position
			//taskNav->offgridpos = calcOffGrid(taskNav->offgridpos);
			break;
		case MOVEINREVERSE:
			taskDriver->driveReverse();
			break;
	}
}

int TM::interrupt(sensors senInt)
{
	motions currentTask = taskNav->getMotion();
	switch(senInt)
	{
		// Forward drive intersects line
		case LINE_ISR:
			if (currentTask == MOVEONGRID)
			{
				taskNav->resetOffGridToZero();
				taskNav->setGrid(dirLineInc(1));
			}
			else if (currentTask == ROTATEONGRID)
			{
				// Calculating 
				grid new_grid = taskNav->getGrid();
				if (taskDriver->get_status() == TURNINGLEFT)
					new_grid.d = (360 + new_grid.d - 90) % 360;
				else if (taskDriver->get_status() == TURNINGRIGHT)
					new_grid.d = (360 + new_grid.d + 90) % 360;
				taskNav->setGrid(new_grid);
			}
		case CLAW_TOUCH:
			if (currentTask == CLAWRETRACT)
			{
				// kill claw motor		
				taskClarm->stop();
				FLAG_clawextended = false;
			}
			break;
		case BOARD_TOUCH:
			taskDriver->stop();
			FLAG_dockedboard = true;
			break;
		case HOPPER_TOUCH_LEFT:
			// Stop if both have touched, else pivot
			if (currentTask == HOPPERALIGN)
			{
				if (FLAG_hopperright == true)
					taskDriver->stop();
				else
					taskDriver->pivotLeft();
			}
			break;
		case HOPPER_TOUCH_RIGHT:
			// Stop if both have touched, else pivot
			if (currentTask == HOPPERALIGN)
			{
				if (FLAG_hopperleft == true)
					taskDriver->stop();
				else
					taskDriver->pivotRight();
			}
			break;
		case TIMER:
			if (currentTask == PAUSE) 
			{
				FLAG_pause = false;
			}
			else if (currentTask == CLAWEXTEND)
			{
				taskClarm->stop();
				FLAG_clawextended = true;
			}
			/*
			else if (currentTask == MOVEINREVERSE)
			{
				taskDriver->stop();
			}
			*/
			break;
	}
}

bool TM::checkTaskComplete() 
{ 
	bool advance = false;
	int navVal = taskNav->getTaskValue();
	grid gridNow = taskNav->getGrid();
	
	// Checks for task completion
	switch (taskNav->getMotion())
	{
		case PAUSE:
			if (FLAG_pause == false) advance = true;	
			break;
		case MOVEONGRID:
			if (taskNav->currentGrid.x == taskdestination.x &&
					taskNav->currentGrid.y == taskdestination.y)
				advance = true;
			break;
		/*
		case MOVEINREVERSE:
			if (taskDriver->get_status() == STOPPED && taskNav->timeElapsed() > 1000)
				advance = true;
			break;
		*/
		case ROTATEONGRID:
			if (gridNow == taskdestination && taskNav->timeElapsed() > 2000) 
				advance = true;
			break;
		case ROTATEOFFGRID:
			/*
			if ((taskDriver->get_status() == TURNINGRIGHT 
					&& taskNav->offgridpos.d > navVal)
				|| (taskDriver->get_status() == TURNINGLEFT 
					&& taskNav->offgridpos.d < navVal))
			{
				advance = true;
			}
			*/
			if (taskNav->timeElapsed() > 1350)
				advance = true;
			break;
		case CLAWEXTEND: 
			if (FLAG_clawextended == true) 
			{ 
				taskWheel->left();
				wheel_pwm = 255;
				advance = true; 
			}
			break;
		case CLAWRETRACT:
			if (FLAG_clawextended == false) 
			{ 
				advance = true; 
			}
			break;
		case HOPPERALIGN:
			if ((FLAG_hopperleft && FLAG_hopperright) == true) 
			{
				taskDriver->stop();
				internalcount = floor(euclideanDist(taskNav->encStarboardCNT, 
						taskNav->encPortCNT));
				advance = true;
			}
			break;
		case GAMEBOARDALIGN:
			if (FLAG_dockedboard == true)
			{
				FLAG_dockedboard = false;
				advance = true;
			}
			break;
		case OFFGRIDOUTBOUND:
			if ((FLAG_hopperleft || FLAG_hopperright) == true && taskNav->timeElapsed() > 500) 
				advance = true;
			break;
		case OFFGRIDRETURN:
			if (internalcount <= floor(euclideanDist(taskNav->encStarboardCNT, 
					taskNav->encPortCNT)))
			{
				advance = true;
			}
			break;
		case MOVEINREVERSE:
			if (board_now == true && taskNav->timeElapsed() > 1500);
				advance = true;
			break;
		case GOOFFGRID:
		case GOONGRID:
			advance = true;
			break;
	}
	return advance;
}

namespace TM::GOG // 0  - Go on grid
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::MOG // 1  - Move on grid
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::MIR // 2  - Move in reverse
{
	void start();
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::GFG // 3  - Go off grid
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::OOB // 4  - Off grid outbound
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::OGR // 5  - Off grid return
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::ROG // 6  - Rotate on grid
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::RFG // 7  - Rotate off grid
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::HAL // 8  - Hopper alignment
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::GAL // 9  - Gameboard alignment
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::CEX // 10 - Claw extend
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::CRT // 11 - Claw retract
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::PPP // 12 - Pause
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
namespace TM::MOI // 13 - Motion idle
{
	void start()
	{

	}
	void process()
	{

	}
	void interrupt()
	{

	}
	void check()
	{

	}
}
