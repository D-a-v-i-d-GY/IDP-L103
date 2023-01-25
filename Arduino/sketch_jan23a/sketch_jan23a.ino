#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

void setup() {
Serial.begin (9600);
AFMS.begin();
myMotor->setSpeed(150);
myMotor->run(FORWARD);
}

void loop() {
  
}