#include "taskmanager.h"

using namespace TM;

// Create class instance list
motionMOG myMOG = motionMOG(MOG); // Move on grid
motionMIR myMIR = motionMIR(MIR); // Move in reverse
motionNOE myGFG = motionNOE(NOE); // Go off grid
motionNOE myOGB = motionNOE(NOE); // Off grid outbound
motionNOE myOGR = motionNOE(NOE); // Off grid return
motionNOE myROG = motionNOE(NOE); // Rotate on grid
motionNOE myRFG = motionNOE(NOE); // Rotate off grid
motionNOE myHAL = motionNOE(NOE); // Hopper alignment
motionNOE myGAL = motionNOE(NOE); // Gameboard alignment
motionNOE myCEX = motionNOE(NOE); // Claw extend
motionNOE myCRT = motionNOE(NOE); // Claw retract
motionPPP myPPP = motionPPP(PPP); // Pause
motionMOI myMOI = motionMOI(MOI); // Motion idle


motionlist = 
{
	&myMOG,
	&myMIR,
	&myGFG,
	&myOGB,
	&myOGR,
	&myROG,
	&myRFG,
	&myHAL,
	&myGAL,
	&myCEX,
	&myCRT,
	&myPPP,
	&myMOI
};

bool FLAG_clawextended = true;
bool FLAG_dockedboard = false;
bool FLAG_pause = false;
bool FLAG_hopperleft = false;
bool FLAG_hopperright = false;
grid taskdestination = tkNav->getDestination();
drcoord departingpoint = tkNav->offgridpos;

int predockingheading = 0;
int internalcount = 0;

double euclideanDist(int x, int y) { return sqrt(x*x + y*y); }

grid dirLineInc(int i)
{
	grid temp_grid = tkNav->getGrid();
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

drcoord calcOffGrid(drcoord lastPos)
{
	drcoord newPos = lastPos;
	unsigned int encPort = tkNav->getEncPortCNT();
	unsigned int encStarboard = tkNav->getEncStarboardCNT();
	// Turning to starboard is positive
	newPos.d = 2 * M_PI * (Rw/D) * (encPort - encStarboard) / Tr;
	newPos.x = Rw * cos(lastPos.d) * (encPort + encStarboard) * M_PI / Tr;
	newPos.y = Rw * sin(lastPos.d) * (encPort + encStarboard) * M_PI / Tr;
	// Reset count
	tkNav->resetEncCNT();
	return newPos;
}

/*
	Start
	// Initialize tasks
	int navVal = tkNav->getTaskValue();
	grid navGrid = tkNav->getGrid();
	switch (tkNav->getMotion())
	{
		case PAUSE:
			timer = navVal;
			FLAG_pause = true;
			tkDriver->stop();
			break;
		case MOVEONGRID:
			tkDriver->driveStraight();
			tkdestination = dirLineInc(navVal);
			break;
		case OFFGRIDRETURN:
			if (navVal > 0)
				internalcount = navVal;
		case MOVEINREVERSE:
			//timer = navVal;
			tkDriver->driveReverse();
			break;
		case OFFGRIDOUTBOUND:
			//predockingheading = tkNav->offgridpos.d;
			tkDriver->driveStraight();
			break;
		case HOPPERALIGN:
			tkDriver->driveStraight();
			break;
		case ROTATEONGRID:
			tkdestination = navGrid;
			tkdestination.d = navVal;
		case ROTATEOFFGRID:
			// Determine direction of rotation
			// Normalize current heading	
			navVal = (navVal - navGrid.d) % 360;
			if (navVal > 180) 
				tkDriver->turnLeft();
			else
				tkDriver->turnRight();
			break;
		case CLAWRETRACT:
			tkClarm->left(clarm_pwm);
			wheel_pwm = 0;
			break;
		case CLAWEXTEND:
			tkClarm->right(clarm_pwm);
			wheel_pwm = 0;
			timer = navVal;
			break;
		case GAMEBOARDALIGN:
			tkDriver->pivotLeftReverse();
			break;
		// Grid ON, OFF	
		case GOONGRID:
			tkNav->on_grid = true;
			break;
		case GOOFFGRID:
			tkNav->on_grid = false;
			break;
	}
	alfd = tkdestination;
	gg = navVal;
}
*/

/*
	int baseSpeed = 255;
	// Non-time critical things that need to be done in the loop
	switch (tkNav->getMotion())
	{
		case MOVEONGRID:
			MOG::processTask();
			break;
		case ROTATEOFFGRID:
		case OFFGRIDOUTBOUND:
			// Update off grid position
			//tkNav->offgridpos = calcOffGrid(tkNav->offgridpos);
			break;
		case MOVEINREVERSE:
			tkDriver->driveReverse();
			break;
	}
}

int interrupt(sensors senInt)
{
	motions currentTask = tkNav->getMotion();
	switch(senInt)
	{
		// Forward drive intersects line
		case LINE_ISR:
			if (currentTask == MOVEONGRID)
			{
				tkNav->resetOffGridToZero();
				tkNav->setGrid(dirLineInc(1));
			}
			else if (currentTask == ROTATEONGRID)
			{
				// Calculating 
				grid new_grid = tkNav->getGrid();
				if (tkDriver->get_status() == TURNINGLEFT)
					new_grid.d = (360 + new_grid.d - 90) % 360;
				else if (tkDriver->get_status() == TURNINGRIGHT)
					new_grid.d = (360 + new_grid.d + 90) % 360;
				tkNav->setGrid(new_grid);
			}
		case CLAW_TOUCH:
			if (currentTask == CLAWRETRACT)
			{
				// kill claw motor		
				tkClarm->stop();
				FLAG_clawextended = false;
			}
			break
		case BOARD_TOUCH:
			tkDriver->stop();
			FLAG_dockedboard = true;
			break;
		case HOPPER_TOUCH_LEFT:
			// Stop if both have touched, else pivot
			if (currentTask == HOPPERALIGN)
			{
				if (FLAG_hopperright == true)
					tkDriver->stop();
				else
					tkDriver->pivotLeft();
			}
			break;
		case HOPPER_TOUCH_RIGHT:
			// Stop if both have touched, else pivot
			if (currentTask == HOPPERALIGN)
			{
				if (FLAG_hopperleft == true)
					tkDriver->stop();
				else
					tkDriver->pivotRight();
			}
			break;
		case TIMER:
			if (currentTask == PAUSE) 
			{
				FLAG_pause = false;
			}
			else if (currentTask == CLAWEXTEND)
			{
				tkClarm->stop();
				FLAG_clawextended = true;
			}
			else if (currentTask == MOVEINREVERSE)
			{
				tkDriver->stop();
			}
			break;
	}
}
*/

/*
	bool advance = false;
	int navVal = tkNav->getTaskValue();
	grid gridNow = tkNav->getGrid();
	
	// Checks for tk completion
	switch (tkNav->getMotion())
	{
		case PAUSE:
			if (FLAG_pause == false) advance = true;	
			break;
		case MOVEONGRID:
			if (tkNav->currentGrid.x == tkdestination.x &&
					tkNav->currentGrid.y == tkdestination.y)
				advance = true;
			break;
		case MOVEINREVERSE:
			if (tkDriver->get_status() == STOPPED && tkNav->timeElapsed() > 1000)
				advance = true;
			break;
		case ROTATEONGRID:
			if (gridNow == tkdestination && tkNav->timeElapsed() > 2000) 
				advance = true;
			break;
		case ROTATEOFFGRID:
			if ((tkDriver->get_status() == TURNINGRIGHT 
					&& tkNav->offgridpos.d > navVal)
				|| (tkDriver->get_status() == TURNINGLEFT 
					&& tkNav->offgridpos.d < navVal))
			{
				advance = true;
			}
			if (tkNav->timeElapsed() > 1350)
				advance = true;
			break;
		case CLAWEXTEND: 
			if (FLAG_clawextended == true) 
			{ 
				tkWheel->left();
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
				tkDriver->stop();
				internalcount = floor(euclideanDist(tkNav->encStarboardCNT, 
						tkNav->encPortCNT));
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
			if ((FLAG_hopperleft || FLAG_hopperright) == true && tkNav->timeElapsed() > 500) 
				advance = true;
			break;
		case OFFGRIDRETURN:
			if (internalcount <= floor(euclideanDist(tkNav->encStarboardCNT, 
					tkNav->encPortCNT)))
			{
				advance = true;
			}
			break;
		case MOVEINREVERSE:
			if (board_now == true && tkNav->timeElapsed() > 1500);
				advance = true;
			break;
		case GOOFFGRID:
		case GOONGRID:
			advance = true;
			break;
	}
	return advance;
}
*/

class Motion
{
	Motion(motions m) mymotion(m)
	{
		taskval = tkNav->getTaskValue();
	}
	virtual void start() {}
	virtual void interrupt() {}
	virtual void process() {}
	virtual void complete() {}
	motions get_motion() { return mymotion; }
};

class motionMOG // Move on grid
{
	public:
		using Motion::Motion;
		void start()
		{
			tkDriver->driveStraight();
			linecount = taskval;
		}
		void process()
		{
			int basespeed = 255;
			// Slow down on line approach
			if (tkNav->absEncDistance() >= floor(lineSepTicks * 0.75))
				baseSpeed = 125;
			tkDriver->lineMotorScaling(baseSpeed);
		}
		void interrupt(sensors intsensor)
		{
			if (intsensor == LINE_ISR)
				--linecount;
		}
		bool complete()
		{
			if (linecount == 0)
				return true;
		}
};

class motionMIR // Move in reverse
{
	public:
		using Motion::Motion;
		void start();
		{
			tkDriver->driveReverse();
		}
		void process()
		{
			if (tkNav->absEncDistance() >= floor(lineSepTicks * 0.75))
				baseSpeed = 125;
			else
				baseSpeed = 255;
			debug_speed = tkDriver->lineMotorScaling(baseSpeed);
		}
		bool complete()
		{
			if (tkDriver->get_status() == STOPPED 
					&& tkNav->timeElapsed() > 1000)
				return true;
		}
};

class motionNOE {};
