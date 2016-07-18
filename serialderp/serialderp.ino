#include <SerialCommand.h>
SerialCommand sCmd;

#include <Servo.h>
Servo scanner;

#include "DHT.h"

#define DHTPIN A0     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
int ledPin = 3;
const int servoPin = 10;
const int pwm_speed = 6;   // Channel A speed
const int pwm_steer = 9;   // Channel B speed
const int dir_a0 = 4;  // Channel A direction 0
const int dir_a1 = 5;  // Channel A direction 1
const int dir_b0 = 7;  // Channel B direction 0
const int dir_b1 = 8;  // Channel B direction 1

const int centerPosition = 90;

int speedOut = 0;
int turnOut = 0;
int angleOut = 90;

int lastCommand = 0;

void setup() {

  Serial.begin(115200);

    analogWrite(ledPin, 0);
  dht.begin();
  scanner.attach(servoPin); // Attach servo to pin 10
  scanner.write(angleOut);

  pinMode(pwm_speed, OUTPUT);  // Set control pins to be outputs
  pinMode(pwm_steer, OUTPUT);
  pinMode(dir_a0, OUTPUT);
  pinMode(dir_a1, OUTPUT);
  pinMode(dir_b0, OUTPUT);
  pinMode(dir_b1, OUTPUT);


  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("HUM",   humCMD);          //no args
  sCmd.addCommand("LIGHT",   lightCMD);          //0 - 255
  sCmd.addCommand("TEMP",   tempCMD);          // if '1' then F, if '0' then C
  sCmd.addCommand("TURN",   turnCMD);          // sets turn -1000 to 1000
  sCmd.addCommand("SPEED",  speedCMD);         // Sets sped -1000 to 1000
  sCmd.addCommand("ANGLE",   angleCMD);          // sets angle of servo, 20 to 160
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
  Serial.println("Ready");
}

void loop() {
  sCmd.readSerial();

  drive();
  scanner.write(angleOut);

  delay(2);
  if (lastCommand > 3000) { //~3 seconds
    brake();
    lastCommand = 0;
  } else {
    lastCommand++;
  }
}

void tempCMD() {
  char *arg;
  float t;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    boolean type = atoi(arg);
    if (type)
    {
  t = dht.readTemperature(true);
    }
    else
    {
  t = dht.readTemperature();
    }
    
  Serial.print("Temperature: ");
  Serial.println(t);
    
  }
  else {
  }
  lastCommand = 0;
}

void humCMD() {
  
  Serial.print("Humidity: ");
  Serial.println(dht.readHumidity());
  lastCommand = 0;
}

void lightCMD() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    
    analogWrite(ledPin, atoi(arg));
  }
  else {
  }
  lastCommand = 0;
}



void speedCMD() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    speedOut = atoi(arg);
    Serial.println(speedOut);
  }
  else {
    speedOut = 0;
  }
  lastCommand = 0;
}



void turnCMD() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    turnOut = atoi(arg);
    Serial.println(turnOut);
  }
  else {
    turnOut = 0;
  }
  lastCommand = 0;
}

void angleCMD() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    angleOut = atoi(arg);
  }
  else {
    angleOut = centerPosition;
  }
  lastCommand = 0;
}

//This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What?");
}

void brake() {
  Serial.println("Braking...");
  digitalWrite(dir_a0, 1);
  digitalWrite(dir_a1, 1);

  digitalWrite(dir_b0, 1);
  digitalWrite(dir_b1, 1);

  speedOut = 0;
  turnOut = 0;
  angleOut = centerPosition;
}

void drive() {
  if (speedOut >= 0) {
    digitalWrite(dir_a0, 0);
    digitalWrite(dir_a1, 1);
  } else {
    digitalWrite(dir_a0, 1);
    digitalWrite(dir_a1, 0);
  }

  if (turnOut >= 0) {
    digitalWrite(dir_b0, 0);
    digitalWrite(dir_b1, 1);
  } else {
    digitalWrite(dir_b0, 1);
    digitalWrite(dir_b1, 0);
  }

  analogWrite(pwm_speed, abs(speedOut));
  analogWrite(pwm_steer, abs(turnOut));
}


