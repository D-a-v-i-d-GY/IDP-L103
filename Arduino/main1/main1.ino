#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define close_ls_left 13 // Left line sensor pin number
#define close_ls_right 12 // Right line sensor pin number

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

// Useful flags
bool going;
bool block_detected = false;
short red_sensor, blue_sensor;

void setup() {

 Serial.begin(9600);
 AFMS.begin();
 left_motor->setSpeed(150);
 right_motor->setSpeed(150);
 left_motor->run(FORWARD);
 right_motor->run(FORWARD);
 going = true;
 pinMode(close_ls_left, OUTPUT);
 pinMode(close_ls_right, OUTPUT);
}

// have a way to read from ultrasound
float distance_from_block (){
  
}

void loop() {
  int left_sensor = digitalRead(13);
  int right_sensor = digitalRead(12);

  bool turn_right;
  bool turn_left;
  bool go_straight;
  bool in_tunnel;
   
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
  else if (in_tunnel) {
    // in tunnel or sommething wrong if in tunnel sound sensor
    
  }

  if (going == true){
    if (go_straight == true) {
      // do nothing
    }
    if (turn_left == true) {
      left_motor->setSpeed(100);
      right_motor->setSpeed(80);
    }
    if (turn_left == true) {
      left_motor->setSpeed(80);
      right_motor->setSpeed(100);
    }
  }

  if (red_sensor == 1 || blue_sensor == 1){
    going == false;
    block_detected == true;
  }


  
  
}
