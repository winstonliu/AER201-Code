// test_Feb2.ino
// Use this code to test your motor with the Arduino board:
// if you need PWM, just use the PWM outputs on the Arduino
// and instead of digitalWrite, you should use the analogWrite command

// 0 implies using optical encoder, 1 implies breakbeam

#include <Wire.h>
#include "rgb_lcd.h"
#include "motor.h"

// --------------------------------------------------------------------------- Motors
// [enable, direction]

const int START_BTN = 10;
const int ARM_SENSOR = 13;
//const int KILL_SWITCH = 2;
const int retract_time = 1000;

const int BREAKBEAM = 1;
volatile bool motor_state = true;

int print_cnt = 0;

rgb_lcd lcd;
// en, dir
motor arm(9,8);
motor wheel(11, 12);

boolean claw_dir = true;
boolean motor_on = false;
boolean engaged = false;

boolean arm_retracting = false;

// --------------------------------------------------------------------------- Setup

void set_bools()
{
	claw_dir = true;
	motor_on = false;
	engaged = false;
	arm_retracting = false;
	motor_state = true;
}

void setup() 
{
    Serial.begin(9600);
	lcd.begin(16,2);
	pinMode(START_BTN, INPUT);
	pinMode(ARM_SENSOR, INPUT);
	
	wheel.left();

	//attachInterrupt(0, kill_all, RISING);

	if (BREAKBEAM == 1)
		attachInterrupt(0, toggle_motor, RISING);
}

void loop()
{    
	if (motor_on == true)
		wheel.stop();
	else
		wheel.left();

	int buttonstate = digitalRead(START_BTN);
    //Triggers the motor
    if(buttonstate == HIGH && !motor_on)
	{
		motor_on = true;
		Serial.println("Motor starting");
		lcd.clear();	
		lcd.print("Motor Start");
	
		if (BREAKBEAM == 1)
		{
			arm.left(); 
			lcd.setCursor(0,1);
			lcd.print("BREAKBEAM");
		}
    }

    if(motor_on && BREAKBEAM == 0)
	{
		if(turn_motor(claw_dir))
		{
			motor_on = false;
			claw_dir = !claw_dir;
			if(!claw_dir)
			{
				Serial.println("Motor stopped");
				lcd.clear();	
				lcd.print("Motor Stop");
				delay(2000);
				motor_on = true;
				Serial.println("Motor Reset");
				lcd.clear();	
				lcd.print("Motor Reset");
			}
			else
			{
				Serial.println("Motor Reversing");
				lcd.clear();
				lcd.print("Motor Reversing");
				lcd.clear();	
				lcd.print("Mission");
				lcd.setCursor(0,1);
				lcd.print("Accomplished");
			}  
		}
    }    
	else if (BREAKBEAM == 1)
	{
		/*if (digitalRead(ARM_SENSOR) == true)
		{
			toggle_motor();
		}
*/

		if (arm_retracting == true && motor_on == true)
		{
			//detachInterrupt(0);
			lcd.clear();
			lcd.print("Stopped");
			delay(2000);
			lcd.clear();
			lcd.print("Retracting");
			arm.right();
			delay(1000);
			arm.stop(); 
			lcd.clear();	
			lcd.print("Mission");
			lcd.setCursor(0,1);
			lcd.print("Accomplished");
			attachInterrupt(0, toggle_motor, CHANGE);
			set_bools();
		}
	}
    delay(50);
	if (print_cnt >= 10)
	{
		print_cnt = 0;
		Serial.print("claw_dir: ");
		Serial.print(claw_dir);
		Serial.print(" motor_on: ");
		Serial.print(motor_on);
		Serial.print(" button state: ");
		Serial.print(buttonstate);
		//Serial.print(" engaged: ");
		//Serial.print(engaged);
		Serial.print(" togglestate: ");
		Serial.print(motor_state);
		Serial.print(" arm retracting: ");
		Serial.println(arm_retracting);
	}
	++print_cnt;
}


//-----Sensors
//--------------------
/* OBSOLETE
int read_val(int pin){
    int sensorValue = analogRead(pin);
    
    // print that variable over the serial connection
    //Serial.print("Pin #");
    //Serial.print(pin);
    //Serial.print(": ");
    //Serial.println(sensorValue);

    return sensorValue;
}
*/

//-----Drive
//--------------------
boolean turn_motor(boolean left)
{
	int sensorstate = digitalRead(ARM_SENSOR);

	//Runs the motor
	if(sensorstate == LOW && motor_on)
	{
		if(engaged)
		{    //Kill motor
			Serial.println("Motor Off");        
			engaged = false;
			arm.stop();
			return true;
		}
	}    
	else if (sensorstate == HIGH && motor_on)
	{
		if(!engaged)
		{
			engaged = true;
		}
	}

    //Output of FSM
    if(left)
	{
		arm.left();
    }
    else
	{
		arm.right();
    }
    
    return false;
}

void toggle_motor()
{
	if (motor_state == true)
	{
		motor_state = false;	
		arm.stop();
		//lcd.clear();
		//lcd.print("Motor Reversing");
		arm_retracting = true;	
	}
}

void kill_all()
{
	arm.stop();
	lcd.clear();
	lcd.print("Sagan is dead");
	set_bools();		
}
