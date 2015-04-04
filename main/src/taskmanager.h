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
			bool FLAG_done;
		public:
			Motion(motions mymotion);
			virtual void start(int& timer);
			virtual void process();
			virtual void interrupt(sensors intsensor);
			virtual bool iscomplete();
			motions get_motion();
	};

	extern Motion *listofmotions[MOTIONSCOUNT];

	// Flags
	extern bool FLAG_pause;
	extern bool FLAG_clawextended;
	extern bool FLAG_dockedboard;
	extern bool FLAG_hopperleft;
	extern bool FLAG_hopperright;
	extern bool FLAG_ball_dropped;

	extern const int wheel_norm;
	
	extern int wheel_pwm;
	extern int clarm_pwm;

	extern int board_now;
	extern int timeforaline;

	extern double lineSep;
	extern int lineSepTicks;

	extern const double RwD; // 1.905 / 24.5
	extern const double Rw; // Wheel radii
	extern const double D; // Wheel separation
	extern const double Tr; // ticks per rotation
	extern const double Tr_TRD; // ticks per degree turning

	extern int offGridTicks;
	extern int predockingheading;
	extern double internalcount;

	extern bool extLeft;
	extern bool extRight;

	// Stuff from nav class
	extern DriveMotor* tkDriver;
	extern motor* tkClarm;
	extern motor* tkWheel;
	extern Nav* tkNav;	
	extern grid tkdest;
	extern drcoord departingpoint;

	void start(int& timer);
	void process();
	void interrupt(sensors intsensor);
	bool iscomplete();

	double euclideanDist(int x, int y);
	grid dirLineInc(int i);
	double modAbsDiff(double a, double b);
	drcoord calcOffGrid(drcoord lastPos);

	void turnDirInit(int speed = 255);

	// Move on grid
	class motionMOG : public Motion 
	{
		public:
			motionMOG(motions m);
			void start(int& timer);
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	// Move in reverse
	class motionMIR : public Motion 
	{
		public:
			motionMIR(motions m);
			void start(int& timer);
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionMTL : public Motion
	{
		public:
			motionMTL(motions m);
			void start(int& timer);
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionRTL : public Motion
	{
		public:
			motionRTL(motions m);
			void start(int& timer);
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionOGR : public Motion
	{
		public:
			motionOGR(motions m);
			void start(int& timer);
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionCEX : public Motion
	{
		public:
			motionCEX(motions m);
			void start(int& timer);
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	
	class motionCRT : public Motion
	{
		public:
			motionCRT(motions m);
			void start(int& timer);
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionROG : public Motion
	{
		public:
			motionROG(motions m);
			void start(int& timer);
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionRFG : public Motion
	{
		public:
			motionRFG(motions m);
			void start(int& timer);
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionHAL : public Motion
	{
		public:
			motionHAL(motions m);
			void start(int& timer);
			void process();
			void interrupt(sensors intsensor);
			bool iscomplete();
	};
	class motionGAL : public Motion
	{
		public:
			motionGAL(motions m);
			void start(int& timer);
			bool iscomplete();
	};
	class motionPPP : public Motion 
	{
		public:
			bool FLAG_pause;
			motionPPP(motions m);
			void start(int& timer);
			void interrupt(sensors intsensor);
			bool iscomplete();
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
