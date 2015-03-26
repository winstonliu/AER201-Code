#pragma once
#include <Arduino.h>
#include "nav.h"

namespace TM	// TaskManager
{
	class Motion
	{
		protected:
			motions mymotion;
			int taskval;
		public:
			Motion(motions mymotion);
			virtual void start();
			virtual void process();
			virtual void interrupt(sensors intsensor);
			virtual bool iscomplete();
			motions get_motion();
	};

	extern Motion *listofmotions[MOTIONSCOUNT];

	// Flags
	extern bool FLAG_clawextended;
	extern bool FLAG_dockedboard;
	extern bool FLAG_pause;
	extern bool FLAG_hopperleft;
	extern bool FLAG_hopperright;

	extern int wheel_pwm;
	extern int clarm_pwm;

	extern int board_now;
	extern int timeforaline;

	extern double lineSep;
	extern unsigned int lineSepTicks;

	extern double Rw; // Wheel radii
	extern double D; // Wheel separation
	extern double Tr; // ticks per rotation

	extern int offGridTicks;
	extern int predockingheading;
	extern int internalcount;

	// Stuff from nav class
	extern DriveMotor* tkDriver;
	extern motor* tkClarm;
	extern motor* tkWheel;
	extern Nav* tkNav;	
	extern grid tkdestination;
	extern drcoord departingpoint;

	void start();
	void process();
	void interrupt(sensors intsensor);
	bool iscomplete();

	double euclideanDist(int x, int y);
	grid dirLineInc(int i);
	drcoord calcOffGrid(drcoord lastPos);

	// Move on grid
	class motionMOG : public Motion 
	{
		private: 
			int linecount;
		public:
			motionMOG(motions m);
			void start();
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	// Move in reverse
	class motionMIR : public Motion 
	{
		public:
			motionMIR(motions m);
			void start();
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionPPP : public Motion 
	{
		public:
			motionPPP(motions m);
	};
	class motionMOI : public Motion 
	{
		public:
			motionMOI(motions m);
	};
	class motionNOE : public Motion 
	{
		public:
			motionNOE(motions m);
	};
}
