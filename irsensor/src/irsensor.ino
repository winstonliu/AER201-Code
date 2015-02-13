const int SP_1 = A0;
const int SP_2 = A1;
const int SP_3 = A2;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	Serial.print(analogRead(SP_1));
	Serial.print(" ");
	Serial.print(analogRead(SP_2));
	Serial.print(" ");
	Serial.println(analogRead(SP_3));
    delay(400);
}
