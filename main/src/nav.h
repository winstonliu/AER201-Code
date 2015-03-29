#pragma once
#include <QueueArray.h>
#include "drivemotor.h"

enum sensors
{
	LINE_ISR,
	CLAW_TOUCH,
	BOARD_TOUCH,
	HOPPER_TOUCH_LEFT,
	HOPPER_TOUCH_RIGHT,
	TIMER,
	SENSORSCOUNT
};

enum motions
{
	MOG, // Move on grid
	MIR, // Move in reverse
	GFG, // Go off grid
	OGB, // Off grid outbound
	OGR, // Off grid return
	ROG, // Rotate on grid
	RFG, // Rotate off grid
	HAL, // Hopper alignment
	GAL, // Gameboard alignment
	CEX, // Claw extend
	CRT, // Claw retract
	PPP, // Pause
	MOI, // Motion idle
	MOTIONSCOUNT,
	NOE
};

enum macro_motion
{
	mMTC, // move to corner hopper
	mCBL, // collect ball
	mMTB, // move to gameboard
	mMTH, // move to closes hopper
	mDDD, // done
	mMOTIONSCOUNT
};

struct task
{
	motions do_now;
	int value;

	task(motions a, int v) : do_now(a), value(v) {}
};

struct grid
{
	int x;	// x coordinates
	int y;  // y coordinates
	int d;	// 0 to 359, N:0, E:90, S:180, W:270

	grid() {}
	grid(int a, int b, int c) : x(a), y(b), d(c) {}

	grid& operator=(const grid& a)
	{
		x = a.x;
		y = a.y;
		d = a.d;
		return *this;
	}

	bool operator==(const grid& a) const
	{
		return (x == a.x && y == a.y && d == a.d);
	}
	bool operator>=(const grid& a) const
	{
		return (x >= a.x && y >= a.y && d >= a.d);
	}

};

// dead-reckoning coordinates, name change to avoid confusion
struct drcoord : grid 
{
	double x;
	double y;
	double z;
	double d;

	drcoord() {}
	drcoord(double a, double b, double c): x(a), y(b), d(c) {}
};

class Nav
{
	// Navigation class with event-driven interrupts
	//private:
		// TODO move stuff to private after
	public:
		// DEBUG
		long sketchyTimer;
		long currentTime;
		// DEBUG
		grid currentGrid;
		grid destination;
		int encPortCNT, encStarboardCNT;
		grid hopperEast;
		grid hopperWest;
		QueueArray <task> tasklist;
		bool check_validity(grid new_position);
		drcoord offgridpos;
		bool on_grid;

		Nav(grid start_position);
		int computeRectilinearPath(grid new_destination);
		int hopperDocking();
		int hopperUndocking();
		void advance();

		int reset(grid);
		int set_destination(grid new_destination);
		int setGrid(grid new_grid);

		void incEncPortCNT();
		void incEncStarboardCNT();
		void resetEncCNT();
		int getEncPortCNT();
		int getEncStarboardCNT();

		int absEncDistance();

		void resetOffGridToZero();
		
		int getTaskValue();
		grid getGrid();
		grid getDestination();
		motions getMotion();

		bool doneTasks();
		int countRemaining();

		long timeElapsed();
};
