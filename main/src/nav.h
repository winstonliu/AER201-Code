#pragma once
#include <QueueArray.h>
#include "drivemotor.h"

enum isr
{
	LINE_ISR,
	TOUCH_ISR
};

enum action
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
	action do_now;
	int value;

	task(action a, int v) : do_now(a), value(v) {}
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
		QueueArray <task> tasklist;
		DriveMotor Driver;
	// DEBUG make stuff to private after
	public:
		bool on_grid;
		action currentAction;
		grid currentGrid;
		grid destination;
		grid hopperEast;
		grid hopperWest;

		bool check_validity(grid new_position);
		grid directionalLineIncrement(int i);

		grid taskdestination;
		bool FLAG_unpause;
		bool FLAG_extended;

		nav(grid start_position, DriveMotor& Driver);
		int interrupt(isr sensor_interrupt);	
		int computeRectilinearPath(grid new_destination);

		void startTask();
		void processTask();
		int checkTaskComplete();

		int reset(grid);
		int set_destination(grid new_destination);
		bool doneTasks();
		int countRemaining();
		action getAction();
		grid getGrid();
		grid getDestination();
};
