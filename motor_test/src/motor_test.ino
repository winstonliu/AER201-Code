#include "motor.h"

motor port(4,5);
motor starboard(7,6);

void setup()
{
	port.left();
	starboard.right();
}

void loop()
{	
}
