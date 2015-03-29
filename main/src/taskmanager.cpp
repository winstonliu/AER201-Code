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
motionGAL myGAL = motionGAL(GAL); // Gameboard alignment
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
	newPos.d += fmod((360 * RwD * (pcnt - scnt) / Tr), 360);
	newPos.x += Rw * cos(lastPos.d) * (pcnt + scnt) * M_PI / Tr;
	newPos.y += Rw * sin(lastPos.d) * (pcnt + scnt) * M_PI / Tr;
	// Reset count
	tkNav->resetEncCNT();
	return newPos;
}

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

TM::motionMOG::motionMOG(motions m) : Motion(m) {}
void TM::motionMOG::start(int& timer)
{
	tkNav->resetOffGridToZero();
	tkDriver->driveStraight();
	tkdest = dirLineInc(tkNav->getTaskValue());
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
	{
		tkNav->resetOffGridToZero();
		tkNav->setGrid(dirLineInc(1));
	}
}
bool TM::motionMOG::iscomplete()
{
	if (tkNav->currentGrid >= tkdest)
	{
		return true;
	}
	return false;
}

// ================================================================ //
// MIR

TM::motionMIR::motionMIR(motions m) : Motion(m) {}
void TM::motionMIR::start(int& timer)
{
	tkDriver->driveReverse();
}
void TM::motionMIR::process() {}
void TM::motionMIR::interrupt(sensors intsensor) {}
bool TM::motionMIR::iscomplete()
{
	if (tkNav->timeElapsed() > 1000)
	{
		tkDriver->stop();
		wheel_pwm = wheel_norm;	
		return true;
	}
	return false;
}

// ================================================================ //
// ROG - Rotate on grid to position
TM::motionROG::motionROG(motions m) : Motion(m) {}
void TM::motionROG::start(int& timer)
{
	tkNav->resetOffGridToZero();
	//int turndir = (tkNav->offgridpos.d - tkNav->getTaskValue()) % 180;
	// Normalize current heading 
	//if (tkNav->getTaskValue() > 0)
	//	tkDriver->turnLeft();
	//else
		tkDriver->turnRight();

}
void TM::motionROG::interrupt(sensors intsensor)
{
	// Calculating 
	if (intsensor == LINE_ISR)
	{
		grid newgrid = tkNav->getGrid();
		// Round to nearest cardinal direction 
		newgrid.d = (int)(floor(tkNav->offgridpos.d/90 + 0.5) * 90) % 360;
		tkNav->setGrid(newgrid);
	}
}
bool TM::motionROG::iscomplete()
{
	if (tkNav->getGrid().d >= tkNav->getTaskValue())	
	{
		return true;
	}
	return false;
}

// ================================================================ //
// RFG - rotate to position
TM::motionRFG::motionRFG(motions m) : Motion(m)
{
	tkNav->resetOffGridToZero();
}
void TM::motionRFG::start(int& timer)
{
	// Normalize current heading 
	//if (tkNav->getTaskValue() > 180) 
	//	tkDriver->turnLeft();
	//else
		tkDriver->turnRight();
}
void TM::motionRFG::interrupt(sensors intsensor) {}
bool TM::motionRFG::iscomplete()
{
	if (tkNav->offgridpos.d > tkNav->getTaskValue())
	{
		return true;
	}
	return false;
}

// ================================================================ //
// CEXfunc

TM::motionCEX::motionCEX(motions m) : Motion(m) {}
void TM::motionCEX::start(int& timer)
{
	tkClarm->right(clarm_pwm);
	wheel_pwm = 0;
	timer = tkNav->getTaskValue();
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
		wheel_pwm = wheel_norm;
		return true;
	}
	return false;
}

// ================================================================ //

TM::motionCRT::motionCRT(motions m) : Motion(m) {}
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

TM::motionHAL::motionHAL(motions m) : Motion(m) {}
void TM::motionHAL::start(int& timer)
{
	tkDriver->driveStraight();
}
void TM::motionHAL::interrupt(sensors intsensor)
{
	if (intsensor == HOPPER_TOUCH_LEFT)
	{
		if (FLAG_hopperright == true)
		{
			FLAG_hopperleft = true;
			tkDriver->stop();
		}
		else
		{
			tkDriver->pivotLeft();
		}
	}	
	else if (intsensor == HOPPER_TOUCH_RIGHT)
	{
		if (FLAG_hopperleft == true)
		{
			FLAG_hopperright = true;
			tkDriver->stop();
		}
		else
		{
			tkDriver->pivotRight();
		}
		FLAG_hopperright = true;
	}
}
bool TM::motionHAL::iscomplete()
{
	if ((FLAG_hopperleft & FLAG_hopperright) == true) 
	{
		tkDriver->stop();
		//internalcount = floor(euclideanDist(tkNav->encStarboardCNT, 
	//			tkNav->encPortCNT));
		return true;
	}
	return false;
}

// ================================================================ //

TM::motionGAL::motionGAL(motions m) : Motion(m) {}
void TM::motionGAL::start(int& timer) {}
void TM::motionGAL::interrupt(sensors intsensor) {}
bool TM::motionGAL::iscomplete() {} 

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
	{
		tkDriver->stop();
		return true;
	}
	return false;
}

// ================================================================ //
TM::motionMOI::motionMOI(motions m) : Motion(m) {}
TM::motionNOE::motionNOE(motions m) : Motion(m) {}
