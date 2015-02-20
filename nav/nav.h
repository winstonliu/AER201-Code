#pragma once

enum isr
{
	LINE_ISR,
	TOUCH_ISR
};

enum action
{
	MOVETO,
	ROTATE
};

struct task
{
	action nextTask;
	int value;
};

struct grid
{
	int x;	// x coordinates
	int y;  // y coordinates
	int d;	// 0 to 359, angle of rotation	
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
		bool check_validity(grid);
	public:
		nav(grid);
		int interrupt(isr);	
		void computePath();

		int reset(grid);
		int set_destination(grid);
		grid get_current();
};
