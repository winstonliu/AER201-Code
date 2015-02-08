// test_Feb2.ino
// Use this code to test your motor with the Arduino board:
// if you need PWM, just use the PWM outputs on the Arduino
// and instead of digitalWrite, you should use the analogWrite command

#include <Wire.h>
#include "rgb_lcd.h"

// --------------------------------------------------------------------------- Motors
const int MOTOR_LEFT[] = {5,4};

const int HOPPER_SENSOR = 3;
const int ENCODER_SENSOR = 2;
const int ball_collected_sensor = 7;
const int LED = 10;

rgb_lcd lcd;

boolean claw_dir = true;
boolean motor_on = false;
boolean engaged = false;


// --------------------------------------------------------------------------- Setup
void setup() 
{
    Serial.begin(9600);
	lcd.begin(16,2);
    pinMode(MOTOR_LEFT[0], OUTPUT);
    pinMode(MOTOR_LEFT[1], OUTPUT);
	pinMode(HOPPER_SENSOR, INPUT);
	pinMode(ENCODER_SENSOR, INPUT);
}

void loop()
{    
	int buttonstate = digitalRead(HOPPER_SENSOR);
    //Triggers the motor
    if(buttonstate == HIGH && !motor_on)
	{
		motor_on = true;
		Serial.println("Motor starting");
		lcd.clear();	
		lcd.print("Motor Start");
    }

    if(motor_on)
	{
		boolean ret = turn_motor(claw_dir);
		if(ret)
		{
			on = false;
			claw_dir = !claw_dir;
			if(!claw_dir)
			{
				Serial.println("Motor stopped");
				lcd.clear();	
				lcd.print("Motor Stop");
				delay(2000);
				motor_on = true;
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
	Serial.print("claw_dir: ");
	Serial.print(claw_dir);
	Serial.print(" motor_on: ");
	Serial.print(motor_on);
	Serial.print(" engaged: ");
	Serial.println(engaged);
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
	int sensorstate = digitalRead(ENCODER_SENSOR);
	//Runs the motor
    if(sensorstate == LOW && motor_on)
	{
		if(engaged)
		{    //Kill motor
			Serial.println("Motor Off");        
			engaged = false;
			motor_stop();
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
    digitalWrite(MOTOR_LEFT[0], LOW);
    digitalWrite(MOTOR_LEFT[1], LOW);    
}

void motor_r()
{
    digitalWrite(MOTOR_LEFT[0], HIGH);
    digitalWrite(MOTOR_LEFT[1], LOW);
}

void motor_l()
{
    digitalWrite(MOTOR_LEFT[0], HIGH);
    digitalWrite(MOTOR_LEFT[1], HIGH);    
}

