#ifndef NAV_H
#define NAV_H

enum isr
{
	LINE_ISR,
	TOUCH_ISR
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
		bool on_grid;
		grid current;
		grid destination;
		bool check_validity(grid);
	public:
		nav(grid);
		int interrupt(isr);	

		int reset(grid);	
		int set_destination(grid);	
		grid get_current();
};
#endif // NAV_H
