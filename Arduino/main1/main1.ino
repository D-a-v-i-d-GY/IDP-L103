#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
bool going = true;
bool block_detected = false;

void setup() {
 AFMS.begin();
 left_motor->setSpeed(150);
 right_motor->setSpeed(150);
 pinMode(13, INPUT);
 pinMode(12, INPUT);

 pinMode(10, INPUT);
 pinMode(9, INPUT);
 Serial.begin(9600);

}

// have a way to read from ultrasound

void loop() {
  int left_sensor = digitalRead(13);
  int right_sensor = digitalRead(12);
 
  int red_sensor = digitalRead(10);
  int blue_sensor = digitalRead(9);
  bool turn_right;
  bool turn_left;




  float distance_from_block (){
    
  }

   
  // assume 1 is white
  if(right_sensor == 0 && left_sensor == 0) {
    go_straight = true;
    turn_left = false;
    turn_right = false;
  }
  else if (right_sensor == 0 && left_sensor == 1){
    go_straight = false;
    turn_left = true;
    turn_right = false;
  }
  else if (right_sensor == 1 && left_sensor == 0){
    go_straight = false;
    turn_left = false;
    turn_right = true;
  }
  else {
    // in tunnel or sommething wrong if in tunnel sound sensor
    
  }

  if ( going == true){
    if ( go_straight == true) {
      left_motor->run(FORWARD)
      right_motor->run(FORWARD)
    }
    if ( turn_left == true) {
      left_motor->setSpeed(100);
      right_motor->setSpeed(80);
    }
    if ( turn_left == true) {
      left_motor->setSpeed(80);
      right_motor->setSpeed(100);
    }
  }

  if ( red_sensor == 1 || blue_sensor == 1){
    going == false;
    block_detected == true;
  }

  if

  
  
}
