#include "motor.h"

// Initialize motors (en, dir)
Motor port(3,4);
Motor starboard(5,6);

void setup()
{
	starboard.left(255);
	port.left(255);
}

void loop()
{

}
