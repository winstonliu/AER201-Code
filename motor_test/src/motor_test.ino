#include "motor.h"

motor port(8,9);
motor starboard(10,11);

void setup()
{
	port.left();
	starboard.right();
}

void loop()
{
}
