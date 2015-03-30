#include "taskmanager.h"

using namespace TM;

// Create class instance list
motionMOG myMOG = motionMOG(MOG); // Move on grid
motionMIR myMIR = motionMIR(MIR); // Move in reverse
motionNOE myGFG = motionNOE(NOE); // Go off grid
motionNOE myOGB = motionNOE(NOE); // Off grid outbound
motionOGR myOGR = motionOGR(OGR); // Off grid return
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
bool TM::FLAG_ball_dropped = false;

grid TM::tkdest = tkNav->getDestination();
drcoord TM::departingpoint = tkNav->getOffGridPos();

int TM::predockingheading = 0;
double TM::internalcount = 0.0;
int TM::numloops = 0;

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
	double turnnow;
	pcnt = tkNav->getEncPortCNT();
	scnt = tkNav->getEncStarboardCNT();
	drcoord newPos = lastPos;
	// Turning to starboard is positive
	switch(tkDriver->get_status())
	{
		case PIVOTLEFTBACK:
		case TURNINGRIGHT:
			scnt *= -1;
			//turnnow = 6.61;
			break;
		case PIVOTRIGHTBACK:
		case TURNINGLEFT:
			pcnt *= -1;
			//turnnow = -6.61;
			break;
		case DRIVINGREVERSE:
			scnt *= -1;	
			pcnt *= -1;
			break;
	}
	//tkNav->turncoord += fmod((360 + turnnow), 360);
	tkNav->turncoord += fmod((360 * RwD * (pcnt - scnt) / Tr), 360);

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
	numloops = tkNav->getTaskValue();
	tkNav->zeroOGXY();
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
		tkNav->zeroOGXY();
		tkNav->setGrid(dirLineInc(1));
	}
}
bool TM::motionMOG::iscomplete()
{
	if ((tkNav->currentGrid >= tkdest) || (numloops == 0))
	{
		tkDriver->stop();
		return true;
	}
	return false;
	--numloops;
}

// ================================================================ //
// MIR

TM::motionMIR::motionMIR(motions m) : Motion(m) {}
void TM::motionMIR::start(int& timer)
{
	tkDriver->driveReverse();
	//timer = tkNav->getTaskValue();
}
void TM::motionMIR::process() {}
void TM::motionMIR::interrupt(sensors intsensor) 
{
	if (intsensor == LINE_ISR)
	{
		tkDriver->stop();
	}
}
bool TM::motionMIR::iscomplete()
{

	if ((tkNav->timeElapsed() > 1000) 
			|| (tkDriver->get_status() == STOPPED))
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
	tkNav->zeroOGXY();
	// Normalize current heading 
	if ((tkNav->getOffGridPos().d - tkNav->getTaskValue()) > 0)
		tkDriver->turnRight();
	else
		tkDriver->turnLeft();

}
void TM::motionROG::interrupt(sensors intsensor)
{
	// Calculating 
	if (intsensor == LINE_ISR)
	{
		grid newgrid = tkNav->getGrid();
		// Round to nearest cardinal direction 
		newgrid.d =(int)(floor(tkNav->getOffGridPos().d/90 + 0.2) * 90) % 360;
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
TM::motionRFG::motionRFG(motions m) : Motion(m) {}
void TM::motionRFG::start(int& timer)
{
	// Calculate turning direction
	int turndir = (int)(tkNav->getTaskValue()
		   - tkNav->getOffGridPos().d) % 360;
	if ((turndir >= 180) || ((-180 < turndir) && (turndir < 0)))
		tkDriver->turnLeft();
	else if ((turndir < -180) || ((0 <= turndir) && (turndir < 180)))
		tkDriver->turnRight();
	/*
	if (tkNav->getTaskValue() > 180)
		tkDriver->turnLeft();
	else
		tkDriver->turnRight();
	*/
	tkNav->turncoord = 0;
}
void TM::motionRFG::interrupt(sensors intsensor) {}
bool TM::motionRFG::iscomplete()
{
	int diff = tkNav->getOffGridPos().d - tkNav->getTaskValue();
	if (((diff < 0) & (tkDriver->turnLeft == true)) == true)
	{
			
	}
	/*
	bool greater = tkNav->turncoord > tkNav->getTaskValue();
	bool lesser = tkNav->turncoord < tkNav->getTaskValue();

	if (((tkDriver->get_status() == TURNINGLEFT) && lesser) 
		|| ((tkDriver->get_status() == TURNINGRIGHT) && greater))
	{
		return true;
	}
	*/
	if (abs(tkNav->getOffGridPos().d - tkNav->getTaskValue()) < 15)
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
// HAL

TM::motionHAL::motionHAL(motions m) : Motion(m) {}
void TM::motionHAL::start(int& timer)
{
	tkDriver->driveStraight();
}
void TM::motionHAL::process()
{
	internalcount += euclideanDist(tkNav->encStarboardCNT, tkNav->encPortCNT);
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
		return true;
	}
	return false;
}

// ================================================================ //

TM::motionOGR::motionOGR(motions m) : Motion(m) {}
void TM::motionOGR::start(int& timer) 
{
	tkDriver->stop();
	tkDriver->driveReverse();
}
void TM::motionOGR::interrupt(sensors intsensor) {}
void TM::motionOGR::process() 
{
	internalcount -= euclideanDist(tkNav->encStarboardCNT,tkNav->encPortCNT);
	//tkNav->resetEncCNT();
}
bool TM::motionOGR::iscomplete()
{
	if (internalcount <= 0)
	{
		return true;
	}
	return false;
} 

// ================================================================ //

TM::motionGAL::motionGAL(motions m) : Motion(m) {}
void TM::motionGAL::start(int& timer) 
{
	/*
	tkNav->setOffGridPos(tkNav->getGrid().d);
	if (tkNav->currentGrid == grid(4,8,90))
	{
		tkDriver->turnRight();
	}
	else if (tkNav->currentGrid == grid(4,8,270))
	{
		tkDriver->turnLeft();
	}
	*/
	tkDriver->driveReverse();
}
bool TM::motionGAL::iscomplete() 
{
	if ((board_now == HIGH))
	{
		return true;
	}	
	return false;
}

// ================================================================ //
// PPP

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
