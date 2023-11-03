/*
Adafruit Arduino - Lesson 13. DC Motor
*/


int motorPin = 5;

int detectPressure(){
    // wait for the current reading to finish
  while (digitalRead(2)) {}

  // read 24 bits
  long result = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(3, HIGH);
    digitalWrite(3, LOW);
    result = result << 1;
    if (digitalRead(2)) {
      result++;
    }
  }

  // get the 2s compliment
  result = result ^ 0x800000;

  // pulse the clock line 3 times to start the next pressure reading
  for (char i = 0; i < 3; i++) {
    digitalWrite(3, HIGH);
    digitalWrite(3, LOW);
  }
    // display pressure
  Serial.println(result);
  return result;
}


void setup() 
{ 
  pinMode(motorPin, OUTPUT);
  pinMode(2, INPUT);   // Connect HX710 OUT to Arduino pin 2
  pinMode(3, OUTPUT);  // Connect HX710 SCK to Arduino pin 3
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Speed 0 to 255");
} 
 
 
void loop() 
{   
  if (Serial.available())
  {
    int speed = Serial.parseInt();
    if (speed >= 0 && speed <= 255)
    {
      int pressure = 5;
      while(pressure != 4){
        analogWrite(motorPin, speed);
        pressure = detectPressure();
      }
    }
  }
}
