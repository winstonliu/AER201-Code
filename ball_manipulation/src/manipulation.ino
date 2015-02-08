// test_Feb2.ino
// Use this code to test your motor with the Arduino board:
// if you need PWM, just use the PWM outputs on the Arduino
// and instead of digitalWrite, you should use the analogWrite command

#include <Wire.h>
#include "rgb_lcd.h"

// --------------------------------------------------------------------------- Motors
int motor_left[] = {3,2};

int hopper_sensor = 1;
int encoder_sensor = 0;
int ball_collected_sensor = 2;
int LED = 10;

rgb_lcd lcd;

boolean claw_in = true;
boolean on = false;
boolean start = true;
boolean engaged = false;

// --------------------------------------------------------------------------- Setup
void setup() 
{
    Serial.begin(9600);
	lcd.begin(16,2);
    pinMode(motor_left[0], OUTPUT);
    pinMode(motor_left[1], OUTPUT);
}

void loop()
{    
    //Triggers the motor
    if(read_val(hopper_sensor) > 600 && !on)
	{
		on = true;
		Serial.println("Motor starting");
		lcd.clear();	
		lcd.print("Motor Start");
		lcd.setCursor(0,1);
    }

    if(on)
	{
		boolean ret = turn_motor(claw_in);
		if(ret)
		{
			on = false;
			claw_in = !claw_in;
			if(!claw_in)
			{
				Serial.println("Motor stopped");
				lcd.clear();	
				lcd.print("Motor Stop");
				lcd.setCursor(0,1);
				lcd.print(analogRead(hopper_sensor));
				delay(2000);
				on = true;
				Serial.println("Motor resetting");
				lcd.clear();	
				lcd.print("Motor Reset Now");
			}
			else
			{
				Serial.println("Motor reset");
				lcd.clear();
				lcd.print("Motor Reset");
				boolean ret = ball_collected();
				if(ret)
				{
					blink_led();
					lcd.clear();	
					lcd.print("Mission");
					lcd.setCursor(0,1);
					lcd.print("Accomplished");
				}
			}  
		}
    }    
    delay(500);
	Serial.print("claw_in: ");
	Serial.print(claw_in);
	Serial.print(" on: ");
	Serial.print(on);
	Serial.print(" start: ");
	Serial.print(start);
	Serial.print(" engaged: ");
	Serial.println(engaged);
}


//-----Sensors
//--------------------
int read_val(int pin){
    int sensorValue = analogRead(pin);
    
    // print that variable over the serial connection
    //Serial.print("Pin #");
    //Serial.print(pin);
    //Serial.print(": ");
    //Serial.println(sensorValue);

    return sensorValue;
}

boolean ball_collected(){
	return true;
}  

void blink_led(){
	Serial.println("Blinking LED");
	digitalWrite(LED, HIGH);
	delay(500);
	digitalWrite(LED, LOW);
}

//-----Drive
//--------------------
boolean turn_motor(boolean left)
{
	//Runs the motor
    if(read_val(encoder_sensor) < 600 && on)
	{
		if(engaged)
		{    //Kill motor
			Serial.println("Motor Off");        
			engaged = false;
			motor_stop();
			return true;
		}
    }    
    else if (read_val(encoder_sensor) > 600 && on)
	{
		if(!engaged)
		{
			engaged = true;
		}
    }

    //Output of FSM
    if(left)
	{
		motor_l();
    }
    else
	{
		motor_r();
    }
    
    return false;
}

void motor_stop()
{
    digitalWrite(motor_left[0], LOW);
    digitalWrite(motor_left[1], LOW);    
}

void motor_r()
{
    digitalWrite(motor_left[0], HIGH);
    digitalWrite(motor_left[1], LOW);
}

void motor_l()
{
    digitalWrite(motor_left[0], HIGH);
    digitalWrite(motor_left[1], HIGH);    
}

