#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#define trigPin 7
#define echoPin 8

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
bool going = true;
bool block_detected = false;
bool in_tunnel = false;
float robot_width = 20;
float wall_width = 35;

void setup() {
 AFMS.begin();
 left_motor->setSpeed(150);
 right_motor->setSpeed(150);
 pinMode(13, INPUT);
 pinMode(12, INPUT);

 pinMode(10, INPUT);
 pinMode(9, INPUT);
 pinMode(trigPin, OUTPUT); 
 pinMode(echoPin, INPUT);
 Serial.begin(9600);


}

// have a way to read from ultrasound

void loop() {
  int left_sensor = digitalRead(13);
  int right_sensor = digitalRead(12);
 
  int red_sensor = digitalRead(10);
  int blue_sensor = digitalRead(9);

  float wall_distance;
  
  bool turn_right;
  bool turn_left;

  




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
    
    
  }
  else {
    // assume sensor on the right    
    if(wall_distance < ((tunnel_width - robot_width)/2 - 2) ){
      left_motor->setSpeed(90);
      right_motor->setSpeed(100);
    } 
    if(  ((tunnel_width - robot_width - wall_distance) < ((tunnel_width - robot_width)/2 - 2)  ){
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
    if ( turn_left == true) {
      left_motor->setSpeed(100);
      right_motor->setSpeed(80);
      left_motor->run(FORWARD);
      right_motor->run(FORWARD);
    }
    if ( turn_left == true) {
      left_motor->setSpeed(80);
      right_motor->setSpeed(100);
      left_motor->run(FORWARD);
      right_motor->run(FORWARD);
    }
  }

  if ( red_sensor == 1 || blue_sensor == 1){
    going == false;
    block_detected == true;
  }

  if

  
  
}
