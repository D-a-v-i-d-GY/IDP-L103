#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// ALL LENGTHS ARE IN MM!!

#define ll_pin 13 // Left-most line sensor pin number
#define l_pin 12 // Left line sensor pin number
#define r_pin 11 // Right line sensor pin number
#define rr_pin 8 // Right-most line sensor pin number
#define button_pin 7 // button_pin for starting the robot
#define echoPinSide 6 // Echo Pin for the ultrasonic sensor on the side
#define trigPin 5 // Trigger Pin for the ultrasonic sensor
#define redBlockIdPin 4 // Pin for identified red block
#define blueBlockIdPin 3  // Pin for identified blue block
#define colorReadPin A0

// MOTOR INDICIES
#define LEFT_MOT 0 // Index of the left motor
#define RIGHT_MOT 1 // Index of the right motor
// #define CLAW_MOT 2 // Index for claw motor 

// GEOMETRICAL CHARACTERISTICS, ROBOT CHARACTERISTICS
#define delay_per_degree 13.05 // At 240 motor speed tested and works the delay per degree when rotating
#define stagnation_time 500 // Time allowed for a line sensor to be continuously active || TEST
#define tj_detection_interval 800 // Minimum time difference between consecutive detection of t-jucntions || TEST
#define line_crossing_delay 300 // Minimum time delay that ensures that the line sensor goes through the line without getting activated || TEST
#define pi 3.141593

#define wheel_dist 220 // Distance between the centers of the wheels
#define llrr 75 // Distance between left-most and right-most sensors
#define lr 25 // Distance between left and right sensors
#define line_width 19 // Width of the white line
#define pick_up_distance 60 // robot wasn't ready to use
#define red_area_d 745 // robot wasn't ready to use
#define greed_area_d 190 // robot wasn't ready to use
#define grab_angle 60 // robot wasn't ready to use
#define lift_angle 60 // robot wasn't ready to use
#define drop_distance 60 // the distance we initiate drop sequence robot wasn't ready to use
#define release_distance 30 // robot wasn't ready to use gotta check

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* right_motor = AFMS.getMotor(4);
Adafruit_DCMotor* left_motor = AFMS.getMotor(3);

Adafruit_DCMotor* motors[] = {left_motor, right_motor}; // Make an array with the mototrs for convenient handling


Servo grab_servo;  // attach servo to pin 9 in setup havent done yet not sure if thats how it works  robot wasn't ready
Servo lift_servo;  // attach servo to pin 10 in setup havent done yet not sure if thats how it works robot wasn't ready

// Useful flags
bool execute = false;
int delivery_stage = 1; // Counter that determines the stage of the delivery process, robot's actions are determined by the stage number
int tjc = 0; // T joint count (as seen from robot's perspective)
int temp_counter = 0; 
bool stage_start = true; // Indicator for the start of a new stage
bool rotating = false; // bool for checking if the robot is rotating
bool reached_main_loop = false; // in the first case checks if it has got out of the box and on the main loop
bool on_main_loop = false; // same use
bool block_picked_up = false; 
bool block_detected = false;
bool red_block = false;
bool green_block = false;
bool drop = false; // the following were meant to be bools for grabbing the block and dropping it but the robots mechanics wasn't ready for testing
bool grab = false; 
bool first = false;
bool second = false;
bool third = false;
bool grabbed = false;
bool in_position = true;

// Variable init
int motor_speeds[] = {0, 0};
int motor_directions[] = {0, 0};
float motor_velocities[] = {0.0, 0.0};
float orientation;
bool ll_value; // the following are values for line sensors
bool l_value;
bool r_value;
bool rr_value;
bool button_value; // the value for the start button
int ll_rr_up_flag = 0;  // flag to be used in the initial stage for turning right first
bool l_up = false; // flag to check if left sensor is on 
bool r_up = false; // flag to check if right sensor is on 
unsigned long t_l = 0; // to measure the up-time of the left sensor
unsigned long t_r = 0; // to measure the up-time of the right sensor
unsigned long t_tjc = 0; // t junction counteer time
unsigned long t0 = 0;
unsigned long t_stage_part = 0; // the following are for measuring time to add redundencys
unsigned long t_stage_st = 0;
unsigned long t_button = 0;


// function for value reading of a pin
bool sensorRead(int pin){
    bool reading;
    int out = digitalRead(pin);
    if (out == 0){reading = false;}
    else{reading = true;}

    return reading;
}

// colour identification  function called when required
void identifyColor(int analog_reading){
    if (analog_reading > 300){ // blue block
        digitalWrite(blueBlockIdPin, HIGH);
        green_block = true;
        red_block = false;
        delay(5000);
        digitalWrite(blueBlockIdPin, LOW);
    }
    else{
        digitalWrite(redBlockIdPin, HIGH); 
        red_block = true;
        green_block = false;
        delay(5000);
        digitalWrite(redBlockIdPin, LOW);
    }
}

// t junction counter helps the robot know its location
void tjCounter(){
    // Counter of T-junctions, with protection against excessive counting
    if (millis() - t_tjc > tj_detection_interval){tjc++; t_tjc = millis();}
    Serial.println("T junction encountered!"); // DEBUGGING
    Serial.println(tjc); // DEBUGGING
}

// function for ultrasonic sensor measurement robot wasn;t ready for exectution
float distance_ultrasonic (int pinNumber){
    unsigned long duration;
    float distance;
    // Activate the sensor`
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // Just a delay
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Triggers the sensor
    digitalWrite(trigPin, LOW);
    duration = pulseIn(pinNumber, HIGH); // Measure the duration of the pulse
    distance = (duration / 2) / 2.905;
    if (distance >= 3900){distance=-99;}
    return distance;
}

// TESTED
void set_motor_speed(int motor, int speed){
    // This function sets the speed of the given driving motor, 0 for left and 1 for right

    // test that the speed value is in the right 
    if (speed < 0 || speed > 255){
      return;
        Serial.println("Speed out of range"); // For debugging
    }

    if (motor == 1 || motor == 0){
        // Don't change the speed if it is the same 
        if (motor_speeds[motor] != speed){
            motors[motor]->setSpeed(speed); motor_speeds[motor] = speed;
        }
    }
    else{
        Serial.println(motor); // For debugging
        Serial.println("motor should be either 0 or 1");
    }
}

// TESTED
void set_motor_direction(int motor, int direction){
    if (motor == 1 || motor == 0){
        // This function sets the direction of the diving motors assuming straight line motion
        if (motor_directions[motor] != direction){
            // Assign the correct direction for the motors
            if (direction == 1){
                motors[motor]->run(FORWARD);
            }
            else if (direction == 0){
                motors[motor]->run(RELEASE);
            }
            else if (direction == -1){
                motors[motor]->run(BACKWARD);
            }
            else { 
                // Check for correct direction assignment
                Serial.println("Direction not defined"); // For debugging
                return;
            }
        }

        motor_directions[motor] = direction;
    }
    else{
        Serial.println(motor); // For debugging
        Serial.println("motor should be either 0 or 1");
    }
}

// TESTED rotates a given angle
void rotate_angle(float angle){
    // This function makes the robot rotate around the center of the wheel axis by the given angle, CCW is assumed positive
  
    set_motor_speed(LEFT_MOT, 240); 
    set_motor_speed(RIGHT_MOT, 240);
    // 4 seconds for full rotation at 240

    if (angle > 0){
        set_motor_direction(LEFT_MOT, -1);
        set_motor_direction(RIGHT_MOT, 1);
        delay(angle * delay_per_degree);
    }
    else if (angle < 0){
        set_motor_direction(LEFT_MOT, 1);
        set_motor_direction(RIGHT_MOT, -1);
        delay(-angle * delay_per_degree);
    }

    
    set_motor_direction(LEFT_MOT, 0);
    set_motor_direction(RIGHT_MOT, 0);
}

// TESTED rotates ccw
void rotate_ccw(int speed){
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, -1);
    set_motor_direction(RIGHT_MOT,1);
}

// TESTED rotattes cw
void rotate_cw(int speed){
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, 1);
    set_motor_direction(RIGHT_MOT, -1);
}

// TESTED function for following line
void follow_line(int forward_speed, int rotation_speed){
    // Algorithm for following the line

    // If any of the line-following sensors were on for too long then that sensor cannot go off the line, ignore that sensor's reading
    if (millis() - t_l >= stagnation_time && l_up){ // TEST THE up-time MINIMUM
        if (r_value){l_value = false;} // Force the reading of the sensor to zero
        Serial.println("Left sensor is stuck!");// DEBUGGING
    }
    else if (millis() - t_r >= stagnation_time && r_up){ // TEST THE up-time MINIMUM
        if (l_value){r_value = false;} // Force the reading of the sensor to zero
        Serial.println("Right sensor is stuck!");// DEBUGGING
    }

    if(ll_value == false && l_value == false && r_value == false && rr_value == false){ // 0000
        set_motor_speed(LEFT_MOT, forward_speed); 
        set_motor_speed(RIGHT_MOT, forward_speed); 
        set_motor_direction(LEFT_MOT, 1);
        set_motor_direction(RIGHT_MOT, 1);
    }

    else if(ll_value == false && l_value == true && r_value == false && rr_value == false){ // 0100
        rotate_ccw(rotation_speed);
    }

    else if(ll_value == false && l_value == false && r_value == true && rr_value == false){ // 0010
        rotate_cw(rotation_speed);
    }

    // MODIFY THE CODE BELOW
    else if(ll_value == true && l_value == false && r_value == false && rr_value == false){ // 1000
        rotate_ccw(rotation_speed);
    }

    else if(ll_value == false && l_value == false && r_value == false && rr_value == true){ // 0001
        rotate_cw(rotation_speed);
    }
}

// obsolete pick up function for picking up the 2nd and 3rd blocks robot wasn't ready for use
 void pick_up() {  // once in psootion pick up
    if(in_position == true) {
      set_motor_direction(LEFT_MOT, 1);
      set_motor_direction(RIGHT_MOT, 1);
      delay(300);
      set_motor_direction(LEFT_MOT, 0);
      set_motor_direction(RIGHT_MOT, 0);
      rotate_angle(90);
      delay(2000);
      set_motor_direction(LEFT_MOT, 1);
      set_motor_direction(RIGHT_MOT, 1);
      delay(2000);
      set_motor_direction(LEFT_MOT, -1);
      set_motor_direction(RIGHT_MOT, -1);
      delay(400);
      set_motor_direction(LEFT_MOT, 0);
      set_motor_direction(RIGHT_MOT, 0);
      rotate_angle(180);
      set_motor_direction(LEFT_MOT, 1);
      set_motor_direction(RIGHT_MOT, 1);
      delay(1000);
      set_motor_direction(LEFT_MOT, 0);
      set_motor_direction(RIGHT_MOT, 0);
      delay(2000);
      if(r_value == false && l_value == false){
        set_motor_direction(LEFT_MOT, -1);
        set_motor_direction(RIGHT_MOT, -1);
      }
      else { 
        set_motor_direction(LEFT_MOT, 0);
        set_motor_direction(RIGHT_MOT, 0);
      }
      delay(500);
      rotate_angle(30);
      in_position = false; 
   }
 }

// main function which takes the robot through stages
void stage_action(){
    // ADD for stage shifts, e.g. time based, encoder value based, ultrasonic sensor value based
    switch (delivery_stage) {
        case 1: // Getting out of the drop-off/start box || TEST
            if (stage_start){
                // Start going forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                Serial.println("Start"); // DEBUGGING

                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);

                stage_start = false;
                //lift_servo.write(-20); 
                }

            // Record the t-junctions until the main loop is reached
            if (!reached_main_loop){
                if (ll_value || rr_value){tjCounter();} // Record the T-junction
                if (tjc == 2) {
                    reached_main_loop = true;
                    Serial.println("reached main loop");
                }
                break;
            }

            // When the main loop is reached exectue the algorithm below to get the main loop line between the central sensors
            else if (!on_main_loop){
                // orientation of the robot is acceptable, start rotating after a short delay
                if (ll_value && rr_value){
                    delay((int) line_crossing_delay / 4);
                    on_main_loop = true;
                    Serial.println("both on");
                }
                // record which sensor reached the line first
                else if (ll_value && ll_rr_up_flag == 0){
                    ll_rr_up_flag = -1;
                }
                else if (rr_value && ll_rr_up_flag == 0){
                    ll_rr_up_flag = 1;
                }
                // if right-right sensor reached the line first, wait until the left sensor reaches and start rotating 
                else if (ll_rr_up_flag == 1 && l_value){
                    on_main_loop = true;
                    Serial.println("right first");
                }
                // if left-left sensor reached the line first, wait until the right sensor reaches and start rotating 
                else if (ll_rr_up_flag == -1 && r_value){
                    on_main_loop = true;
                    Serial.println("left first");
                    }
                break;
            }
                
            if (on_main_loop){
                Serial.println("on main loop");
                if (!rotating){
                    // Stop
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    // Start rotating
                    Serial.println("started rotating"); // DEBUGGING
                    rotate_cw(160);
                    delay((int) line_crossing_delay*1.5); // THIS DELAY IS VERY IMPORTANT, IT HAS TO BE LONG ENOUGH TO MAKE SURE THE LEFT SENSOR IS ON THE LINE
                    rotating = true;
                    break;
                }
                else if((!ll_value && !l_value && !r_value && !rr_value) || (l_value)){ // <- IMPROVE THIS
                    // Stop and go to the next stage
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    delivery_stage = 2;
                    stage_start = true;
                    break;
                }
            }
            break;
        case 2: // Getting close to the ramp || TEST
            if (stage_start){
                // Start moving forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                Serial.print("Start time: "); // DEBUGGING
                
                t_stage_st = millis();
                Serial.println(t_stage_st);

                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);

                delay(line_crossing_delay); // to get off the initial line that connects start box to the main loop || TEST THE DELAY
                
                stage_start = false;
            }

           // Give the robot enough time to leave the starting zone, then ignore the drop-off junction 
            if ((millis() - t_stage_st > 1500)){ //&& tjc != 3 removed this to test
              if(rr_value == true){
                Serial.println("Drop-off tjc");
                Serial.println(millis()); 

                tjCounter();
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                delay(line_crossing_delay); 
                
                delivery_stage = 3; 
                stage_start = true;
                break;
              }
            }  // Avoid T-junction at the green drop-off || TEST
            // tjc is equal to 3 after the drop-off junction has been passed

            follow_line(200, 200); // TEST THE SPEED
            break;
        case 3: // Going over the ramp
            if (stage_start){
                // Start moving forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                t_stage_st = millis();
                Serial.print("Start time: "); // DEBUGGING

                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200);                 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);

                delay(line_crossing_delay);

                stage_start = false;
            }

            if (millis() - t_stage_st > 5000){delivery_stage = 41; stage_start = true; break;}

            follow_line(250, 180);
            break;
        case 4: // Locating the block skipped case as ultrasonic sensor wasn't ready for use
            if (stage_start){
                // Start moving forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                t_stage_st = millis();
                Serial.print("Start time: "); // DEBUGGING

                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200);                 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);

                stage_start = false;
                //grab_servo.write(grab_angle / 3);
                //lift_servo.write(lift_angle); 
            }
            
            if (tjc == 6 && (millis() - t_stage_part < 500)){
                if (distance_ultrasonic(echoPinSide) < 150){ // TEST THE DISTANCE AND CLEARANCE FROM OTHER OBJECTS
                    Serial.println("Block located at the Third location"); // DEBUGGING
                    // Block located
                    // Stop
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    delivery_stage = 53; // block located at the second pick-up location
                    stage_start=true;
                }
                break;
            }
            
            if (tjc == 5 && (millis() - t_stage_part < line_crossing_delay)){
                follow_line(200, 200); break;
            }

            if (tjc == 4 && (millis() - t_stage_part < 500)){
                if (distance_ultrasonic(echoPinSide) < 150){ // TEST THE DISTANCE AND CLEARANCE FROM OTHER OBJECTS
                    Serial.println("Block located at the First location"); // DEBUGGING
                    // Block located
                    // Stop
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    delivery_stage = 51; // block located at the second pick-up location
                    stage_start=true;
                }
                break;
            }

            if (rr_value || ll_value){
                t_stage_part = millis();
                tjCounter();
                set_motor_speed(LEFT_MOT, 60); 
                set_motor_speed(RIGHT_MOT, 60); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1); 
                break;
            }

            follow_line(200, 200);

            break;
        case 41: // Picking-up the central block and identifying it
            if (stage_start){
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                t_stage_st = millis();
                stage_start = false;
                Serial.println("Start"); // DEBUGGING
                tjc = 3;
            }

            if (tjc == 6){ // Arrived at the starting position
                set_motor_direction(LEFT_MOT, 0);
                set_motor_direction(RIGHT_MOT, 0);
                delivery_stage = 50; stage_start = true; break;
            }

            /*if (tjc == 5){ // Arrived at the starting position      
                set_motor_direction(LEFT_MOT, 0);
                set_motor_direction(RIGHT_MOT, 0);
                delay(500);
                //identifyColor(analogRead(colorReadPin));
                block_detected = true;
                //delay(500);
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);  // added these not sure if necessary
                break;
            }*/
            
            if (rr_value || ll_value){
                tjCounter();
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1); 
                delay(500); break;
            }
            
            follow_line(200, 200);
            break;
        case 50: // out of the stright identfying the block after the turn going in the tunnel and placing block
            if (stage_start){
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                t_stage_st = millis();
                Serial.println("Start"); // DEBUGGING

                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);

                stage_start = false;
                break;
            }

            if(millis() - t_stage_st > 8500 && !green_block && !red_block ) {
              set_motor_direction(LEFT_MOT, 0);
              set_motor_direction(RIGHT_MOT, 0);
              delay(500);
              identifyColor(analogRead(colorReadPin));
              delay(500);
              set_motor_direction(LEFT_MOT, 1);
              set_motor_direction(RIGHT_MOT, 1);  // added these not sure if necessary
              break;
            }

            if (green_block){
                if (tjc == 7){ // arrived at the green drop-off location
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);

                    rotate_angle(270);
                    delay(1000);

                    set_motor_direction(LEFT_MOT, 1);
                    set_motor_direction(RIGHT_MOT, 1);
                    delay(line_crossing_delay * 7);   

                    set_motor_direction(LEFT_MOT, -1);
                    set_motor_direction(RIGHT_MOT, -1);
                    delay(line_crossing_delay * 7);

                    rotate_angle(90);
                    delay(1000);

                    set_motor_direction(LEFT_MOT, 1);
                    set_motor_direction(RIGHT_MOT, 1);
                    delay(line_crossing_delay);

                    delivery_stage = 60; stage_start = true; break;
                }
            }

            if (red_block){
                if (tjc == 9){ // arrived at the green drop-off location
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);

                    rotate_angle(270);
                    delay(1000);

                    set_motor_direction(LEFT_MOT, 1);
                    set_motor_direction(RIGHT_MOT, 1);
                    delay(line_crossing_delay * 7);   

                    set_motor_direction(LEFT_MOT, -1);
                    set_motor_direction(RIGHT_MOT, -1);
                    delay(line_crossing_delay * 7);

                    rotate_angle(-90);
                    delay(1000);

                    set_motor_direction(LEFT_MOT, 1);
                    set_motor_direction(RIGHT_MOT, 1);
                    delay(line_crossing_delay);

                    delivery_stage = 60; stage_start = true; break;
                }
            }

            if (rr_value || ll_value){  
                tjCounter();
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1); 
                delay(500); break;
            }

            follow_line(200, 200);
            break;
        case 60: // parking
            if (red_block){
                if (tjc == 10){ // arrived at the green drop-off location
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);

                    rotate_angle(90);
                    delay(1000);

                    set_motor_direction(LEFT_MOT, 1);
                    set_motor_direction(RIGHT_MOT, 1);
                    delay(line_crossing_delay * 15);

                    set_motor_direction(LEFT_MOT, -1);
                    set_motor_direction(RIGHT_MOT, -1);
                    delay(line_crossing_delay * 5);
                    break; 
                }
            }

            if (green_block){
                if (tjc == 8){ // arrived at the green drop-off location
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);

                    rotate_angle(-90);
                    delay(1000);

                    set_motor_direction(LEFT_MOT, 1);
                    set_motor_direction(RIGHT_MOT, 1);
                    delay(line_crossing_delay * 15);

                    set_motor_direction(LEFT_MOT, -1);
                    set_motor_direction(RIGHT_MOT, -1);
                    delay(line_crossing_delay * 5);
                    break; 
                }
            }

            if (rr_value || ll_value){  
                tjCounter();
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1); 
                delay(500); break;
            }

            follow_line(200, 200);
            break;
        
        /* // obsolete cases which would have picked up multiple blocks not tested as robot wasn't ready
        case 51: // second pick-up location. Picking up the block and getting outside the pick-up zone
            if (stage_start){
                if(second == true) {
                  stage_start = false;
                  Serial.println("Start"); // DEBUGGING
                }
                else {
                  stage_start = true;
                }                
            }
            follow_line(200,145);
            if(tjc == 3) {
              grab = true;
              grab_block();
              stage_start = true;
            }
            follow_line(170,145);
            break;
        case 52: // first pick-up location.
            if (stage_start){
                if(first == true) {
                  stage_start = false;
                  Serial.println("Start"); // DEBUGGING
                }
                else {
                  stage_start = true;
                }                
            }
            if(distance_ultrasonic(echoPinSide) < 300) {
              in_position = true;
              pick_up();
              stage_start = true;
            }
            follow_line(170, 150);
            break;
        case 53: // Third pick-up location.
            if (stage_start){
                if(third == true) {
                  stage_start = false;
                  Serial.println("Start"); // DEBUGGING
                }
                else {
                  stage_start = true;
                }                
            }
            if(distance_ultrasonic(echoPinSide) < 300) {
              in_position = true;
              pick_up();
              stage_start = true;
            }
            follow_line(170, 150);
            break;
        case 54: // get through tunnel
            if (stage_start){
              stage_start = false; 
              set_motor_speed(LEFT_MOT, 200); 
              set_motor_speed(RIGHT_MOT, 200); 
              set_motor_direction(LEFT_MOT, 1);
              set_motor_direction(RIGHT_MOT, 1);
              t_stage_st = millis();           
            }
            if (millis() - t_stage_st < 7000) {
              stage_start = true;  // out of the tunnel?
            }
            break;*/
    }
}



        /*  // obsolete cases which would have picked up multiple blocks not tested as robot wasn't ready
        case 60: //locating the right drop off location
            if(stage_start) {
              stage_start = false;
            }
            follow_line(170,145);
            if (red_block && tjc == 9) {
              
              set_motor_direction(LEFT_MOT, 0);
              set_motor_direction(RIGHT_MOT, 0);
              //initiate droppin sequence 
              stage_start = true;
            }
            if (green_block && tjc == 7) {
              set_motor_direction(LEFT_MOT, 0);
              set_motor_direction(RIGHT_MOT, 0);
              //initiate droppin sequence turn right soon when rr_value = 1  !
              stage_start = true;
            }
            break;
        
        case 70: // get to the right place drop the box drop the box
            if (stage_start){
                // Start turning right
                rotate_cw(145);
                delay(line_crossing_delay * 2);
                t_stage_st = millis();
                stage_start = false;
            }
            follow_line(120,120);        
            if(millis() - t_stage_st < 4000 ) {
              set_motor_direction(LEFT_MOT, 0);
              set_motor_direction(RIGHT_MOT, 0);
              stage_start = true; //  stage drop it
            
            }
            break;
        case 80:
          if (stage_start){
            drop = true; // added this bool but probably a better way to do it check!! also adding a function for drop
            drop_block();
            stage_start = true;
          }
          break;
        case 90: // getting back on the line
          if (stage_start) {
            rotate_angle(180); // testtt might not work
            stage_start = false;
          }
          if(l_value == false && r_value == false) {
            set_motor_direction(LEFT_MOT, 1);
            set_motor_direction(RIGHT_MOT, 1);
            // go straiht until some line detected
          }
          else{
            stage_start = true;
          }
          break;
        case 100: // on the line turnn the right way
          if(stage_start){
            stage_start = false;
          }
          follow_line(170,145);
          if(ll_value == true || rr_value == true){
            if(red_block == true) {
              rotate_ccw(90);
            }
            if(blue_block == true) {
              rotate_cw(90);
            }
            delay(500);
            stage_start = true; //hopefully facing the right way 
          }   
          break;
        case 110: // find the starting point   testtttt
          if(stage_start) {
            stage_start = false;             
          }
          if(red_block){
            if(ll_sensor == false) {
              follow_line(170,145);
            }
            else {
              rotate_ccw(90);
              delay(500);
              stage_start = true;
            }
          }
          if(blue_block) {
            if(ll_sensor == false) {
              follow_line(170,145);
            }
            else {
              rotate_cw(90);
              delay(500);
              stage_start = true;
            }
          }
          break;
        case 120: //final case we stop
          if(stage_start){
            stage_start = false;
            t_stage_st = millis()
          }
          
          if (millis() - t_stage_st < 3000){
            follow_line(170,145); // hopefully its straight by now            
          }
          else{
             set_motor_direction(LEFT_MOT, 0);
             set_motor_direction(RIGHT_MOT, 0);
          }  // we have stopped
*/

void setup() {
    // test(); // To make initial tests, like run the motors, claw etc.
    // Motor setup
    Serial.begin(9600);
    AFMS.begin();
    pinMode(ll_pin, INPUT);
    pinMode(l_pin, INPUT);
    pinMode(r_pin, INPUT);
    pinMode(rr_pin, INPUT);
    pinMode(echoPinSide, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(button_pin, INPUT);
    pinMode(colorReadPin, INPUT);
    pinMode(redBlockIdPin, OUTPUT);
    pinMode(blueBlockIdPin, OUTPUT);

    set_motor_direction(LEFT_MOT, 0);
    set_motor_direction(RIGHT_MOT, 0);
    t_stage_st = millis();
    
}

void loop() {
    //grab_servo.write(40);
    //lift_servo.write(40);
    //delay(250);
  // MODIFY THIS CODE AFTER TESTS
    ll_value = sensorRead(ll_pin);
    l_value = sensorRead(l_pin);
    r_value = sensorRead(r_pin);
    rr_value = sensorRead(rr_pin);                
    button_value = sensorRead(button_pin);
    if (l_value && !l_up){t_l = millis(); l_up = true;} 
    if (r_value && !r_up){t_r = millis(); r_up = true;} 
    if (!l_value){l_up = false;}
    if (!r_value){r_up = false;} 

    if (button_value) {   // start the robot once button is pressed
      delay(3000);
      if (!execute){
        execute = true;
        delay(1000);    
      }
      else {
        execute = false;
      }
    }
    if (execute){
      stage_action();
    }
}
