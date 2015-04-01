#include "taskmanager.h"

using namespace TM;

// Create class instance list
motionMOG myMOG = motionMOG(MOG); // Move on grid
motionMIR myMIR = motionMIR(MIR); // Move in reverse
motionNOE myGFG = motionNOE(NOE); // Go off grid
motionLCK myLCK = motionLCK(LCK); // Off grid outbound
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
	&myLCK,
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

bool TM::extLeft = false;
bool TM::extRight = false;

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
	double calcpos;
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
		case STOPPED:
			scnt = 0;
			pcnt = 0;
			break;
	}
	//tkNav->turncoord += fmod((360 + turnnow), 360);
	tkNav->turncoord += fmod((360 * RwD * (pcnt - scnt) / Tr), 360);

	calcpos += 2 * M_PI * RwD * (pcnt - scnt) / Tr;
	newPos.d += fmod((180 / M_PI * calcpos), 360); // convert to degrees
	newPos.x += Rw * cos(calcpos) * (pcnt + scnt) * M_PI / Tr;
	newPos.y += Rw * sin(calcpos) * (pcnt + scnt) * M_PI / Tr;
	// Reset count
	tkNav->resetEncCNT();
	return newPos;
}
void TM::turnDirInit()
{
	int turndir = (int)(tkNav->getTaskValue()
		   - tkNav->getOffGridPos().d) % 360;
	if ((turndir >= 180) || ((-180 < turndir) && (turndir < 0)))
		tkDriver->turnLeft();
	else if ((turndir < -180) || ((0 <= turndir) && (turndir < 180)))
		tkDriver->turnRight();
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
	//tkNav->zeroOGXY();
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
	if ((intsensor == LINE_ISR) && (tkNav->absEncDistance() >= 5))
	{
		tkNav->setOGPtoGrid();
		tkNav->setGrid(dirLineInc(1));
	}
}
bool TM::motionMOG::iscomplete()
{
	if (tkNav->currentGrid == tkdest)
	{
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
// ROG - Rotate on grid to position
TM::motionROG::motionROG(motions m) : Motion(m) {}
void TM::motionROG::start(int& timer)
{
	// Normalize current heading 
	//tkNav->zeroOGXY();
	turnDirInit();
}
void TM::motionROG::process()
{
	//double linediff = fmod(tkNav->getOffGridPos().d/90, 1);
	//bool islineclose = (((linediff < 0.5) ? linediff : 1 - linediff) < 0.3);


	/*
	if (islineclose == true)
		tkDriver->adjustSpeed(200);
	else
		tkDriver->adjustSpeed(255);
	*/
}
void TM::motionROG::interrupt(sensors intsensor)
{
	// Calculating 
	if (intsensor == LINE_ISR)
	{
		grid newgrid = tkNav->getGrid();
		// Round to nearest cardinal direction 
		newgrid.d =(int)(floor(tkNav->getOffGridPos().d/90 + 0.3) * 90) % 360;
		tkNav->setGrid(newgrid);
	}
}
bool TM::motionROG::iscomplete()
{
	int todiff = abs(tkNav->getOffGridPos().d - tkNav->getTaskValue());
	bool islineclose = (((todiff < 180) ? todiff : 360 - todiff) < 10);
	if ((tkNav->getGrid().d == tkNav->getTaskValue()) && islineclose)
	{
		//tkNav->setOGPtoGrid();
		return true;
	}
	return false;
}

// ================================================================ //
// RFG - rotate to position
TM::motionRFG::motionRFG(motions m) : Motion(m) {}
void TM::motionRFG::start(int& timer)
{
	turnDirInit();
	// Calculate turning direction
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
	/*
	bool greater = tkNav->turncoord > tkNav->getTaskValue();
	bool lesser = tkNav->turncoord < tkNav->getTaskValue();

	if (((tkDriver->get_status() == TURNINGLEFT) && lesser) 
		|| ((tkDriver->get_status() == TURNINGRIGHT) && greater))
	{
		return true;
	}
	*/

	int todiff = abs(tkNav->getOffGridPos().d - tkNav->getTaskValue());
	if (((todiff < 180) ? todiff : 360 - todiff) < 5)
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
// LCK

TM::motionLCK::motionLCK(motions m) : Motion(m) {}
void TM::motionLCK::start(int& timer) 
{
	if ((extLeft & extRight) == true)
	{
		return;
	}
	else if ((extRight & extLeft) == false) 
	{
		tkDriver->driveReverse();
	}
	else if ((extRight & !extLeft) == true)
	{
		tkDriver->pivotLeftReverse();
	}
	else if ((!extRight & extLeft) == true)
	{
		tkDriver->pivotRightReverse();
	}
}
void TM::motionLCK::process() 
{
	static int encCnt = 0;

	if (encCnt >= 8) 
	{
		encCnt = 0;
		tkDriver->doOpposite();
	}
	encCnt += (tkNav->encStarboardCNT + tkNav->encPortCNT) / 2;
}
void TM::motionLCK::interrupt(sensors intsensor) {}
bool TM::motionLCK::iscomplete()
{	
	if ((extLeft & extRight) == true)
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
