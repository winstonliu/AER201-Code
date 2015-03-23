#pragma once
#include <QueueArray.h>
#include "drivemotor.h"

enum sensors
{
	LINE_ISR,
	CLAW_TOUCH,
	HOPPER_TOUCH_LEFT,
	HOPPER_TOUCH_RIGHT,
	TIMER
};

enum motions
{
	GOONGRID,
	MOVEONGRID,
	MOVEINREVERSE,
	GOOFFGRID,
	OFFGRIDOUTBOUND,
	OFFGRIDRETURN,
	ROTATEONGRID,
	ROTATEOFFGRID,
	HOPPERALIGN,
	CLAWEXTEND,
	CLAWRETRACT,
	PAUSE,
	MOTIONIDLE
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
	private:
		grid currentGrid;
		grid destination;
		unsigned int encPortCNT, encStarboardCNT;
		// TODO move stuff to private after
	public:
		grid hopperEast;
		grid hopperWest;
		QueueArray <task> tasklist;
		bool check_validity(grid new_position);
		drcoord offgridpos;
		bool on_grid;

		Nav(grid start_position);
		int computeRectilinearPath(grid new_destination);
		int hopperBerthing();
		void advance();

		int reset(grid);
		int set_destination(grid new_destination);
		int setGrid(grid new_grid);

		void incEncPortCNT();
		void incEncStarboardCNT();
		void resetEncCNT();
		unsigned int getEncPortCNT();
		unsigned int getEncStarboardCNT();

		unsigned int absEncDistance();

		void resetOffGridToZero();
		
		int getTaskValue();
		grid getGrid();
		grid getDestination();
		motions getMotion();

		bool doneTasks();
		int countRemaining();
};
