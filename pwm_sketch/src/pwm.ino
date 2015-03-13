const int MOTOPIN_EN = 11;
const int MOTOPIN_DIR = 12;

void setup()
{
    pinMode(MOTOPIN_EN, OUTPUT);
    pinMode(MOTOPIN_DIR, OUTPUT);
}

void loop()
{
	digitalWrite(MOTOPIN_DIR, LOW);
	analogWrite(MOTOPIN_EN, 100);
}
