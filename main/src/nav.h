#pragma once
#include <QueueArray.h>
#include "drivemotor.h"

enum isr
{
	LINE_ISR,
	TOUCH_ISR
};

enum motions
{
	MOVEONGRID,
	MOVEOFFGRID,
	MOVEOFFGRIDREV,
	ROTATETO,
	HOPPERALIGN,
	CLAWEXTEND,
	CLAWRETRACT,
	PAUSE,
	IDLE
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

class nav
{
	// Navigation class with event-driven interrupts
	private:
		DriveMotor Driver;
		int pause_counter;
	// DEBUG make stuff to private after
	public:
		QueueArray <task> tasklist;
		int cycle_count;
		bool on_grid;
		motions currentMotion;
		grid currentGrid;
		grid destination;
		grid hopperEast;
		grid hopperWest;

		bool check_validity(grid new_position);
		grid directionalLineIncrement(int i);

		grid taskdestination;
		bool FLAG_extended;

		nav(grid start_position, DriveMotor& Driver);
		int interrupt(isr sensor_interrupt);	
		int computeRectilinearPath(grid new_destination);
		int hopperBerthing();

		void startTask();
		void processTask();
		bool checkTaskComplete();

		int reset(grid);
		int set_destination(grid new_destination);
		bool doneTasks();
		int countRemaining();
		motions getMotion();
		grid getGrid();
		grid getDestination();
};
