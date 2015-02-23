#pragma once

enum isr
{
	LINE_ISR,
	TOUCH_ISR
};

enum action
{
	MOVEFORWARD,
	ROTATETO
};

struct task
{
	action do_now;
	int value;
};

struct grid
{
	int x;	// x coordinates
	int y;  // y coordinates
	int d;	// 0 to 359, N:0, E:90, S:180, W:270
};

class nav
{
	// Navigation class with event-driven interrupts
	private:
		std::queue<task> taskMaster;
		bool on_grid;
		grid current;
		grid destination;
		grid hopperEast;
		grid hopperWest;
		bool check_validity(grid new_position);
	public:
		nav(grid);

		int interrupt(isr sensor_interrupt);	
		void computePath();
		task nextTask();
		void doneTask();
		int reset(grid);
		int set_destination(grid new_destination);
		grid get_current();
};
