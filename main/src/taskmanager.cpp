#include "taskmanager.h"

using namespace TM;

// Create class instance list
motionMOG myMOG = motionMOG(MOG); // Move on grid
motionMIR myMIR = motionMIR(MIR); // Move in reverse
motionNOE myGFG = motionNOE(NOE); // Go off grid
motionNOE myOGB = motionNOE(NOE); // Off grid outbound
motionNOE myOGR = motionNOE(NOE); // Off grid return
motionROG myROG = motionROG(ROG); // Rotate on grid
motionRFG myRFG = motionRFG(RFG); // Rotate off grid
motionHAL myHAL = motionHAL(HAL); // Hopper alignment
motionNOE myGAL = motionNOE(NOE); // Gameboard alignment
motionCEX myCEX = motionCEX(CEX); // Claw extend
motionCRT myCRT = motionCRT(CRT); // Claw retract
motionPPP myPPP = motionPPP(PPP); // Pause
motionMOI myMOI = motionMOI(MOI); // Motion idle


TM::Motion *TM::listofmotions[MOTIONSCOUNT] = 
{
	// XXX Change this to reflect list in nav.h XXX
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

bool TM::FLAG_pause = false;
bool TM::FLAG_clawextended = true;
bool TM::FLAG_dockedboard = false;
bool TM::FLAG_hopperleft = false;
bool TM::FLAG_hopperright = false;

grid TM::tkdest = tkNav->getDestination();
drcoord TM::departingpoint = tkNav->offgridpos;

int TM::predockingheading = 0;
int TM::internalcount = 0;

void TM::start(int& timer)
{
	listofmotions[tkNav->getMotion()]->start(timer);
}
void TM::process()
{
	listofmotions[tkNav->getMotion()]->process();
}
void TM::interrupt(sensors intsensor)
{
	listofmotions[tkNav->getMotion()]->interrupt(intsensor);
}
bool TM::iscomplete()
{
	return listofmotions[tkNav->getMotion()]->iscomplete();
}


double TM::euclideanDist(int x, int y) { return sqrt(x*x + y*y); }
grid TM::dirLineInc(int i)
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

drcoord TM::calcOffGrid(drcoord lastPos)
{
	int pcnt, scnt;
	pcnt = tkNav->getEncPortCNT();
	scnt = tkNav->getEncStarboardCNT();
	drcoord newPos = lastPos;
	// Turning to starboard is positive
	switch(tkDriver->get_status())
	{
		case TURNINGRIGHT:
			scnt *= -1;
			break;
		case TURNINGLEFT:
			pcnt *= -1;
			break;
		case DRIVINGREVERSE:
			scnt *= -1;	
			pcnt *= -1;
			break;
	}
	newPos.d += 360 * RwD * (pcnt - scnt) / Tr;
	newPos.x += Rw * cos(lastPos.d) * (pcnt + scnt) * M_PI / Tr;
	newPos.y += Rw * sin(lastPos.d) * (pcnt + scnt) * M_PI / Tr;
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
			tkdest = dirLineInc(navVal);
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
			tkdest = navGrid;
			tkdest.d = navVal;
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
	alfd = tkdest;
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
 * Complete
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
			if (tkNav->currentGrid.x == tkdest.x &&
					tkNav->currentGrid.y == tkdest.y)
				advance = true;
			break;
		case MOVEINREVERSE:
			if (tkDriver->get_status() == STOPPED && tkNav->timeElapsed() > 1000)
				advance = true;
			break;
		case ROTATEONGRID:
			if (gridNow == tkdest && tkNav->timeElapsed() > 2000) 
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


// ================================================================ //
// Motion

TM::Motion::Motion(motions m) : mymotion(m) {}
void TM::Motion::start(int& timer) {}
void TM::Motion::process() {}
void TM::Motion::interrupt(sensors intsensor) {}
bool TM::Motion::iscomplete() {}
motions TM::Motion::get_motion() { return mymotion; }

// ================================================================ //
// MOG

TM::motionMOG::motionMOG(motions m) : Motion(m)
{
	taskval = tkNav->getTaskValue();
}
void TM::motionMOG::start(int& timer)
{
	tkNav->resetOffGridToZero();
	tkDriver->driveStraight();
	linecount = taskval;
}
void TM::motionMOG::process()
{
	int basespeed = 255;
	// Slow down on line approach
	/*
	if (tkNav->absEncDistance() >= floor(lineSepTicks * 0.75))
		basespeed = 125;
	*/
	tkDriver->lineMotorScaling(basespeed);
}
void TM::motionMOG::interrupt(sensors intsensor)
{
	if (intsensor == LINE_ISR)
		--linecount;
}
bool TM::motionMOG::iscomplete()
{
	if (linecount == 0)
	{
		return true;
	}
	return false;
}

// ================================================================ //
// MIR

TM::motionMIR::motionMIR(motions m) : Motion(m) 
{
}
void TM::motionMIR::start(int& timer)
{
	tkDriver->driveReverse();
}
void TM::motionMIR::process()
{
}
void TM::motionMIR::interrupt(sensors intsensor)
{

}
bool TM::motionMIR::iscomplete()
{
	if (tkDriver->get_status() == STOPPED 
			&& tkNav->timeElapsed() > 1000)
	{
		return true;
	}
	return false;
}

// ================================================================ //
// ROG
TM::motionROG::motionROG(motions m) : Motion(m)
{
}
void TM::motionROG::start(int& timer)
{
	tkNav->resetOffGridToZero();
	tkdest = tkNav->getGrid();
	tkdest.d = taskval;
	
	// Normalize current heading 
	taskval = (taskval - tkdest.d) % 360;
	if (taskval > 180) 
		tkDriver->turnLeft();
	else
		tkDriver->turnRight();

}
void TM::motionROG::interrupt(sensors intsensor)
{
	// Calculating 
	if (intsensor == LINE_ISR)
	{
		grid new_grid = tkNav->getGrid();
		if (tkDriver->get_status() == TURNINGLEFT)
		{
			// Verification by encoder	
			// Value per turn is 13
			if (tkNav->getEncPortCNT() + tkNav->getEncStarboardCNT() >= 20)
			{
				new_grid.d = (360 + new_grid.d - 90) % 360;
				//tkNav->resetOffGridToZero();	
			}
		}
		else if (tkDriver->get_status() == TURNINGRIGHT)
		{
			if (tkNav->getEncPortCNT() + tkNav->getEncStarboardCNT() >= 20)
			{
				new_grid.d = (360 + new_grid.d + 90) % 360;
				//tkNav->resetOffGridToZero();
			}
		}
		tkNav->setGrid(new_grid);
	}
}
bool TM::motionROG::iscomplete()
{
	if ((tkDriver->get_status() == TURNINGRIGHT 
			&& tkNav->offgridpos.d > taskval)
		|| (tkDriver->get_status() == TURNINGLEFT 
			&& tkNav->offgridpos.d < taskval))
	{
		return true;
	}
	return false;
}

// ================================================================ //
// RFG
TM::motionRFG::motionRFG(motions m) : Motion(m)
{
	tkNav->resetOffGridToZero();
}
void TM::motionRFG::start(int& timer)
{
	// Normalize current heading 
	int tempval = (taskval - tkdest.d) % 360;
	if (tempval > 180) 
		tkDriver->turnLeft();
	else
		tkDriver->turnRight();
}
void TM::motionRFG::interrupt(sensors intsensor)
{
}
bool TM::motionRFG::iscomplete()
{
	if ((tkDriver->get_status()==TURNINGRIGHT)
			&&(tkNav->offgridpos.d > tkNav->getTaskValue())
		||((tkDriver->get_status()==TURNINGLEFT)
			&&(tkNav->offgridpos.d < tkNav->getTaskValue())))
	{
		return true;
	}
	return false;
}

// ================================================================ //
// CEX

TM::motionCEX::motionCEX(motions m) : Motion(m)
{
}
void TM::motionCEX::start(int& timer)
{
	tkClarm->right(clarm_pwm);
	wheel_pwm = 0;
	timer = taskval;
}
void TM::motionCEX::interrupt(sensors intsensor)
{
	if (intsensor == TIMER)
	{
		tkClarm->stop();
		FLAG_clawextended = true;
	}
}
bool TM::motionCEX::iscomplete()
{
	if (FLAG_clawextended == true)
	{
		tkWheel->left();
		wheel_pwm = 255;
		return true;
	}
	return false;
}

// ================================================================ //

TM::motionCRT::motionCRT(motions m) : Motion(m)
{
}
void TM::motionCRT::start(int& timer)
{
	tkClarm->left(clarm_pwm);
	wheel_pwm = 0;
}
void TM::motionCRT::interrupt(sensors intsensor)
{
	if (intsensor == CLAW_TOUCH)
	{
		tkClarm->stop();
		FLAG_clawextended = false;
	}
}
bool TM::motionCRT::iscomplete()
{
	if (FLAG_clawextended == false)
	{
		return true;
	}
	return false;
}

// ================================================================ //

TM::motionHAL::motionHAL(motions m) : Motion(m)
{
}
void TM::motionHAL::start(int& timer)
{
	tkDriver->driveReverse();
}
void TM::motionHAL::interrupt(sensors intsensor)
{
	if (intsensor == HOPPER_TOUCH_LEFT)
	{
		if (FLAG_hopperright == true)
			tkDriver->stop();
		else
			tkDriver->pivotLeft();
	}	
	else if (intsensor == HOPPER_TOUCH_RIGHT)
	{
		if (FLAG_hopperleft == true)
			tkDriver->stop();
		else
			tkDriver->pivotRight();
	}
}
bool TM::motionHAL::iscomplete()
{
	if ((FLAG_hopperleft && FLAG_hopperright) == true) 
	{
		tkDriver->stop();
		//internalcount = floor(euclideanDist(tkNav->encStarboardCNT, 
	//			tkNav->encPortCNT));
		return true;
	}
	return false;
}

// ================================================================ //

TM::motionPPP::motionPPP(motions m) : Motion(m) {}
void TM::motionPPP::start(int& timer)
{
	timer = tkNav->getTaskValue();
	FLAG_pause = true;
}
void TM::motionPPP::interrupt(sensors intsensor)
{
	if (intsensor == TIMER)
		FLAG_pause = false;
}
bool TM::motionPPP::iscomplete() 
{
	if (FLAG_pause == false)
		return true;
	return false;
}

// ================================================================ //
TM::motionMOI::motionMOI(motions m) : Motion(m) {}
TM::motionNOE::motionNOE(motions m) : Motion(m) {}
