#include "motor.h"

// Initialize motors (en, dir)
Motor port(3,2);
Motor starboard(5,4);

void setup()
{
	starboard.left(255);
	port.left(255);
}

void loop()
{

}
