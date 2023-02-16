int Sensor1 = 12; // Enter values of pins of ESP that sensors are attached to
int Sensor2 = 13;
int Sensor3 = 14 ;
int Sensor4 = 27;
int Sensor5 = 26;
int Sensor6 = 25;

 
int val1 = 0; // Set sensor values to zero and store in seperate variables
int val2 = 0;
int val3 = 0;
int val4 = 0;
int val5 = 0;
int val6 = 0;

#include <Wire.h> // Include wire library

void setup() {

  Serial.begin(9600);  // put your setup code here, to run once:
}

 
//Read Sensor values 
void loop() {

  val1 = analogRead(Sensor1);
  val2 = analogRead(Sensor2);
  val3 = analogRead(Sensor3);
  val4 = analogRead(Sensor4);
  val5 = analogRead(Sensor5);
  val6 = analogRead(Sensor6);

  // Print sensor values
  Serial.print(val1);
  Serial.print(val2);
  Serial.print(val3);
  Serial.print(val4);
  Serial.print(val5);
  Serial.print(val6);

}
