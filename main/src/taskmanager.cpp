#include "taskmanager.h"

using namespace TM;

// Create class instance list
motionMOG myMOG = motionMOG(MOG); // Move on grid
motionMIR myMIR = motionMIR(MIR); // Move in reverse
motionMTL myMTL = motionMTL(MTL); // Go off grid
motionMCC myMCC = motionMCC(MCC); // Off grid outbound
motionOGR myOGR = motionOGR(OGR); // Off grid return
motionPFG myPFG = motionPFG(PFG); // Rotate on grid
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
	&myMTL,
	&myMCC,
	&myOGR,
	&myPFG,
	&myRFG,
	&myHAL,
	&myGAL,
	&myCEX,
	&myCRT,
	&myPPP,
	&myMOI
};

bool TM::FLAG_pause = false;
bool TM::FLAG_clawextended = false;
bool TM::FLAG_dockedboard = false;
bool TM::FLAG_hopperleft = false;
bool TM::FLAG_hopperright = false;
bool TM::FLAG_ball_dropped = false;

grid TM::tkdest = tkNav->getDestination();
drcoord TM::departingpoint = tkNav->getOffGridPos();

int TM::predockingheading = 0;
double TM::internalcount = 0.0;

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

double TM::modAbsDiff(double a, double b)
{
	double diff = abs(a - b);
	return (a < 180) ? diff : 360 - diff;
}

drcoord TM::calcOffGrid(drcoord lastPos)
{
	int pcnt, scnt;
	double turnnow;
	pcnt = tkNav->getEncPortCNT();
	scnt = tkNav->getEncStarboardCNT();
	tkNav->turncoord = 0;
	drcoord newPos = lastPos;
	double calcpos;

	// Ignore encoder spikes
	if ((tkDriver->get_status() == STOPPED) 
		|| (tkNav->spikeCheck() == true))
	{
		return lastPos;
	}

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

	if ((tkNav->getMotion() == RFG) || (tkNav->getMotion() == PFG))
	{
		tkNav->turncoord = fmod((360 * RwD * (pcnt - scnt) / Tr), 360);
	}

	if (abs(tkNav->turncoord) > 15)
	{
		tkNav->turncoord = 0;
	}

	calcpos += 2 * M_PI * RwD * (pcnt - scnt) / Tr;
	newPos.d += tkNav->turncoord; // convert to degrees
	newPos.x += Rw * cos(calcpos) * (pcnt + scnt) * M_PI / Tr;
	newPos.y += Rw * sin(calcpos) * (pcnt + scnt) * M_PI / Tr;
	return newPos;
}
void TM::turnDirInit(int speed)
{
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
	tkNav->zeroOGXY();
	tkDriver->driveStraight();
	tkdest = dirLineInc(tkNav->getTaskValue());
}
void TM::motionMOG::process()
{
	// Slow down on line approach
	/*
	if (tkNav->absEncDistance() >= floor(lineSepTicks * 0.75))
		basespeed = 125;
	*/
	tkDriver->lineMotorScaling(basespeed);

	/*
	if (tkNav->absEncDistance() >= 22)
	{
		tkNav->zeroOGXY();
		tkNav->setGrid(dirLineInc(1));
	}
	*/

}
void TM::motionMOG::interrupt(sensors intsensor)
{
	if (tkNav->absEncDistance() > 10)
	{	
		if (intsensor == LINE_ISR)
		{
			tkNav->zeroOGXY();
			tkNav->setGrid(dirLineInc(1));
		}
		else if (dirLineInc(1) == tkdest)
		{
			if (intsensor == IRLEFT)
			{
				tkDriver->ptr_port->stop();
			}
			else if (intsensor == IRRIGHT)
			{
				tkDriver->ptr_starboard->stop();
			}
		}
	}
}
bool TM::motionMOG::iscomplete()
{
	if (tkNav->currentGrid == tkdest)
	{
		tkNav->zeroOGXY();
		tkDriver->stop();
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

	if (tkDriver->get_status() == STOPPED)
	{
		tkDriver->stop();
		//wheel_pwm = wheel_norm;	
		return true;
	}
	return false;
}

// ================================================================ //
// MTL - moves off grid for x cm unless reaches line first.
// Part of error correction suite

TM::motionMTL::motionMTL(motions m) : Motion(m) {}
void TM::motionMTL::start(int& timer)
{
	tkNav->zeroOGXY();
	tkDriver->stop();
	this->FLAG_done = false;
	if (tkNav->online == true)
		this->FLAG_done = true;
	else if (tkNav->getTaskValue() < 0)
		tkDriver->driveReverse(basespeed - 20);
	else
		tkDriver->driveStraight(basespeed - 20);
}
void TM::motionMTL::process() {}
void TM::motionMTL::interrupt(sensors intsensor) 
{
	if ((intsensor == LINE_ISR) 
			|| ((tkNav->extLeft & tkNav->extRight) == true))
	{
		tkDriver->stop();
		this->FLAG_done = true;
	}
	// For small values of correction, don't bother with angle corrections
	else if (abs(tkNav->getTaskValue()) > 4) 
	{
		if (intsensor == IRLEFT)
		{
			tkDriver->ptr_starboard->adjustSpeed(basespeed - 20);
			tkDriver->ptr_port->adjustSpeed(0);
		}
		else if (intsensor == IRRIGHT)
		{
			tkDriver->ptr_starboard->adjustSpeed(0);
			tkDriver->ptr_port->adjustSpeed(basespeed - 20);
		}
	}
}
bool TM::motionMTL::iscomplete()
{
	static double degElapsed = -1;
	bool farenuf = false;
	if (degElapsed < 0)
	{
		degElapsed = tkNav->offgridpos.d;
	}
	else
	{
		farenuf = modAbsDiff(tkNav->getOffGridPos().d, degElapsed) 
			> abs(tkNav->getTaskValue() * 2);
	}

	if ((tkNav->absEncDistance() > abs(tkNav->getTaskValue()))
			|| farenuf
			|| (this->FLAG_done == true))
	{
		degElapsed = -1;
		return true;
	}
	return false;
}

// ================================================================ //
// MCC

TM::motionMCC::motionMCC(motions m) : Motion(m) {}
void TM::motionMCC::start(int& timer) 
{
	tkNav->zeroOGXY();
	if (tkNav->getTaskValue() < 0)
		tkDriver->driveReverse();
	else
		tkDriver->driveStraight();
}
void TM::motionMCC::process() {}
void TM::motionMCC::interrupt(sensors intsensor) {}
bool TM::motionMCC::iscomplete() 
{
	if (tkNav->absEncDistance() >= abs((double)tkNav->getTaskValue()/10.0))
	{
		tkDriver->stop();
		return true;
	}
	return false;
}

// ================================================================ //
// PFG - Rotate on grid to position
TM::motionPFG::motionPFG(motions m) : Motion(m) {}
void TM::motionPFG::start(int& timer)
{
	// Normalize current heading 
	tkNav->zeroOGXY();
	int turndir = (int)(tkNav->getTaskValue()
		   - tkNav->getOffGridPos().d) % 360;
	if ((turndir >= 180) || ((-180 < turndir) && (turndir < 0)))
		tkDriver->pivotLeft(basespeed);
	else if ((turndir < -180) || ((0 <= turndir) && (turndir < 180)))
		tkDriver->pivotRight(basespeed);

}
void TM::motionPFG::process() {}
void TM::motionPFG::interrupt(sensors intsensor) {}
bool TM::motionPFG::iscomplete()
{
	int todiff = abs(tkNav->getOffGridPos().d - tkNav->getTaskValue());
	if (((todiff < 180) ? todiff : 360 - todiff) < 8)
	{
		grid newgrid = tkNav->getGrid();
		newgrid.d =(int)(floor(tkNav->getOffGridPos().d/90 + 0.3) * 90) % 360;
		tkNav->setGrid(newgrid);
		return true;
	}
	return false;
}

// ================================================================ //
// RFG - rotate to position
TM::motionRFG::motionRFG(motions m) : Motion(m) {}
void TM::motionRFG::start(int& timer)
{
	tkNav->resetEncCNT();
	tkNav->zeroOGXY();
	int turndir = (int)(tkNav->getTaskValue()
		   - tkNav->getOffGridPos().d) % 360;
	if ((turndir >= 180) || ((-180 < turndir) && (turndir < 0)))
		tkDriver->turnLeft(basespeed);
	else if ((turndir < -180) || ((0 <= turndir) && (turndir < 180)))
		tkDriver->turnRight(basespeed);

	// Calculate turning direction
		/*
	if (tkNav->getTaskValue() > 180)
		tkDriver->turnLeft();
	else
		tkDriver->turnRight();
	*/
}
void TM::motionRFG::interrupt(sensors intsensor) {}
bool TM::motionRFG::iscomplete()
{
	int todiff = abs(tkNav->getOffGridPos().d - tkNav->getTaskValue());
	if (((todiff < 180) ? todiff : 360 - todiff) < 8)
	{
		grid newgrid = tkNav->getGrid();
		newgrid.d =(int)(floor(tkNav->getOffGridPos().d/90 + 0.3) * 90) % 360;
		tkNav->setGrid(newgrid);
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
	tkClarm->left();
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
	internalcount = 0.0;
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
	if (tkNav->getTaskValue() > 0)
		tkDriver->driveStraight();
	else
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
		tkDriver->stop();
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
	internalcount = 0.0;
	tkDriver->driveReverse();
}
void TM::motionGAL::process()
{
	internalcount += euclideanDist(tkNav->encStarboardCNT, tkNav->encPortCNT);
}
bool TM::motionGAL::iscomplete() 
{
	if ((board_now == HIGH))
	{
		// CHANGE TO 180
		tkNav->offgridpos.d = 90;
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
