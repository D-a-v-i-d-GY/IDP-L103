#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#define trigPin 7
#define echoPin 8

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
bool in_tunnel = false;
float robot_width = 20;
float wall_width = 35;

float distance_from_block (void){
    unsigned long duration, distance;
    // Activate the sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // Just a delay
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Triggers the sensor
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH); // Measure the duration of the pulse
    distance = (duration/2) / 29.1;
    if (distance >= 390){distance=-999;}
    delay(250);
    return distance;
}

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
 pinMode(trigPin, OUTPUT); 
 pinMode(echoPin, INPUT);
 Serial.begin(9600);

}

void loop() {
  int left_sensor = digitalRead(close_ls_left);
  int right_sensor = digitalRead(close_ls_right);

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

  int red_sensor = digitalRead(10);
  int blue_sensor = digitalRead(9);

  float wall_distance;
  
  bool turn_right;
  bool turn_left;


  wall_distance = distance_from_block();
  if(wall_distance < (wall_width - robot_width + 3) ) {
    in_tunnel = true;
  }
  else {
    in_tunnel = false;
  }
   
  // assume 1 is white
  if( in_tunnel == false)
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
    // add code
  }

  else {
    // assume sensor on the right    
    if(wall_distance < ((tunnel_width - robot_width)/2 - 2) ){
      left_motor->setSpeed(90);
      right_motor->setSpeed(100);
    } 
    if((tunnel_width - robot_width - wall_distance) < ((tunnel_width - robot_width)/2 - 2)  ){
      left_motor->setSpeed(100);
      right_motor->setSpeed(90);
    }       
  }

  if ( going == true && in_tunnel == false){
    if ( go_straight == true) {
      left_motor->setSpeed(150);
      right_motor->setSpeed(150);
      left_motor->run(FORWARD);
      right_motor->run(FORWARD);
    }
    if (turn_left == true) {
      left_motor->setSpeed(100);
      right_motor->setSpeed(80);
      left_motor->run(FORWARD);
      right_motor->run(FORWARD);
    }
    if (turn_left == true) {
      left_motor->setSpeed(80);
      right_motor->setSpeed(100);
      left_motor->run(FORWARD);
      right_motor->run(FORWARD);
    }
  }

  if (red_sensor == 1 || blue_sensor == 1){
    going == false;
    block_detected == true;
  }
}
