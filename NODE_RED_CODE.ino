#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04
#include <MPU6050_tockn.h>
#include <Adafruit_NeoPixel.h>


//Global variables needed for the sensors
//HC
#define trigPin  32
#define echoPin  26
long duration;
int distance;

//LED output
const int ledPin = 32;

//Motor controls
int leftMotor_speed, rightMotor_speed, servoAngle;

//IR Sensors
int Sensor1 = 35;
int Sensor2 = 34;

int val1 = 0;
int val2 = 0;

//MPU
MPU6050 mpu6050(Wire);
int mpuX = 0;
int mpuY = 0;
int mpuZ = 0;


//LED Strip
#define LED_PIN 18 // Pin number for NeoPixel strip
#define LED_COUNT 15 // Number of LEDs in strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


//Node red connection
const char* ssid = "greedy2";                      
const char* password = "EEEEE1002";                               
const char* mqtt_server = "192.168.2.1";      

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //Needed for the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  strip.begin();
  strip.show(); 
  TransmitToArduino(0,0,90);
}

void TransmitToArduino(int leftMotor_speed, int rightMotor_speed, int servoAngle) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)(leftMotor_speed & 0x0000FF00) >> 8);
  Wire.write((byte)(leftMotor_speed & 0x000000FF));
  Wire.write((byte)(rightMotor_speed & 0x0000FF00) >> 8);
  Wire.write((byte)(rightMotor_speed & 0x000000FF));
  Wire.write((byte)(servoAngle & 0x0000FF00) >> 8);
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
 
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
 if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 255, 0));
    }
    strip.show();
    delay(10);
  }
    }
    else if(messageTemp == "off"){
    Serial.println("off");
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();  
    }
  }


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
     
      // Add your subscribe topics here
      // --
      
      client.subscribe("esp32/servoAngle");
      client.subscribe("esp32/motorSpeed");
      client.subscribe("esp32/distance");
      client.subscribe("esp32/output");
      client.subscribe("esp32/Angle_X");
      client.subscribe("esp32/Angle_Y");
      client.subscribe("esp32/Angle_Z");
      client.subscribe("esp32/Sensor1");
      client.subscribe("esp32/Sensor2");
      // --
         
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void loop() {


  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
   
    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --
    
 //HC-SR04
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);

    // Calculate the distance
    distance = duration * 0.034 / 2;
    Serial.println(distance);
    char distanceString[8];
    dtostrf(distance, 1, 2, distanceString);
    client.publish("esp32/distance", distanceString); 

  //MPU6050 
  //Angle X
    mpu6050.update();
     mpuX = mpu6050.getAngleX();
    Serial.println(mpuX);
    char mpuXString[8];
    dtostrf(mpuX, 1, 2, mpuXString);
    client.publish("esp32/Angle_X", mpuXString);
  //Angle Y
     mpuY = mpu6050.getAngleY();
    Serial.println(mpuY);
    char mpuYString[8];
    dtostrf(mpuY, 1, 2, mpuYString);
    client.publish("esp32/Angle_Y", mpuYString);
  // Angle Z
     mpuZ = mpu6050.getAngleZ();
    Serial.println(mpuZ);
    char mpuZString[8];
    dtostrf(mpuZ, 1, 2, mpuZString);
    client.publish("esp32/Angle_Z", mpuZString);
  
  // IR Sensors
    val1 = analogRead(Sensor1);
     Serial.println(val1);
    char S1String[8];
    dtostrf(val1, 1, 2, S1String);
    client.publish("esp32/Sensor1", S1String);
 
    val2 = analogRead(Sensor2);
     Serial.println(val2);
    char S2String[8];
    dtostrf(val2, 1, 2, S2String);
    client.publish("esp32/Sensor2", S2String);



    // --
  }
}
