#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// ALL LENGTHS ARE IN MM!!

#define ll_pin 13 // Left-most line sensor pin number
#define l_pin 12 // Left line sensor pin number
#define r_pin 11 // Right line sensor pin number
#define rr_pin 8 // Right-most line sensor pin number
#define echoPinFront 7 // Echo Pin for the ultrasonic sensor in the front
#define echoPinSide 6 // Echo Pin for the ultrasonic sensor on the side
#define trigPin 5 // Trigger Pin for the ultrasonic sensor

// MOTOR INDICIES
#define LEFT_MOT 0 // Index of the left motor
#define RIGHT_MOT 1 // Index of the right motor
#define CLAW_MOT 2 // Index for claw motor

// GEOMETRICAL CHARACTERISTICS, ROBOT CHARACTERISTICS
#define delay_per_degree 12.9 // At 150 motor speed (NEED TO MEASURE) (DO NOT USE?)
#define stagnation_time 500 // Time allowed for a line sensor to be continuously active || TEST
#define tj_detection_interval 200 // Minimum time difference between consecutive detection of t-jucntions || TEST
#define line_crossing_delay 100 // Minimum time delay that ensures that the line sensor goes through the line without getting activated || TEST
#define wheel_dist 220 // Distance between the centers of the wheels, NEED TO MEASURE
#define llrr 75 // Distance between left-most and right-most sensors, NEED TO VERIFY
#define lr 25 // Distance between left and right sensors, NEED TO VERIFY
#define line_width 19 // Width of the white line, NEED TO VERIFY
#define pick_up_distance 60
#define pi 3.141593
#define red_area_d 745
#define greed_area_d 190
#define lift_angle 60
#define drop_distance 60 // random guess gotta check tghis the distance we initiate drop sequence
#define release_distance 30 // again gotta check when we drop it

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* right_motor = AFMS.getMotor(1);
Adafruit_DCMotor* left_motor = AFMS.getMotor(2);
Adafruit_DCMotor* claw_motor = AFMS.getMotor(3);
Adafruit_DCMotor* motors[] = {left_motor, right_motor, claw_motor}; // Make an array with the mototrs for convenient handling



Servo lift_servo;  // attach servo to pin 9 in setup havent done yet not sure if thats how it works

// Useful flags
int delivery_stage = 1; // Counter that determines the stage of the delivery process, robot's actions are determined by the stage number
int tjc = 0; // T joint count (as seen from robot's perspective)
int temp_counter = 0;
bool stage_start = true; // Indicator for the start of a new stage
bool rotating = false;
bool reached_main_loop = false;
bool on_main_loop = false;
bool block_picked_up = false;
bool block_detected = false;
bool red_block = false;
bool green_block = false;
bool drop = false;
bool grab = false;

// Variable init
int motor_speeds[] = {0, 0};
int motor_directions[] = {0, 0};
float motor_velocities[] = {0.0, 0.0};
float orientation;
bool ll_value;
bool l_value;
bool r_value;
bool rr_value;
int ll_rr_up_flag = 0;
bool l_up = false; // flag to check if left sensor is on 
bool r_up = false; // flag to check if right sensor is on 
unsigned long t_l = 0; // to measure the up-time of the left sensor
unsigned long t_r = 0; // to measure the up-time of the right sensor
unsigned long t_tjc = 0;
unsigned long t0 = 0;

void tjCounter(){
    // Counter of T-junctions, with protection against excessive counting
    if (millis() - t_tjc > tj_detection_interval){tjc++; t_tjc = millis();}
    Serial.println("T junction encountered!"); // DEBUGGING
    Serial.println(tjc); // DEBUGGING
}

// TEST THIS!!!! (TRY NOT TO USE!!!)
float velocity(int motor){
    // Return velocity based on the speed of the motor
    // CHECK THIS
    return 106.4 * ((float) motor_speeds[motor] / (float) 255) * (float) motor_directions[motor];
}

// NEEDS TEST (TRY NOT TO USE!!!)
void orientaion_change(){
    if (t0 == 0){
        t0 = millis();
        return;
    }
    float dt = (millis() - t0) / 1000;
    float omega = (velocity(RIGHT_MOT) - velocity(LEFT_MOT)) / wheel_dist;
    orientation += omega * dt * (180 / pi);
    if (orientation >= 360){
      orientation -= 360;
    }
    if (orientation <= -360){
      orientation += 360;
    }
}

// TESTED A LITTLE, MORE TESTS
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
            motors[motor]->setSpeed(speed); motor_speeds[motor] = speed; motor_velocities[motor] = velocity(motor);
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

        motor_directions[motor] = direction; motor_velocities[motor] = velocity(motor);
    }
    else{
        Serial.println(motor); // For debugging
        Serial.println("motor should be either 0 or 1");
    }
}

// NEEDS TEST (TRY NOT TO USE!!!)
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

// NEEDS TEST
void rotate_ccw(int speed){
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, -1);
    set_motor_direction(RIGHT_MOT, 1);
}

// NEEDS TEST
void rotate_cw(int speed){
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, 1);
    set_motor_direction(RIGHT_MOT, -1);
}

// DONE? NEEDS A LOT OF TEST!!!
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

    else if(ll_value == true && l_value == true && r_value == false && rr_value == false){ // 1100
        // Left T-junction
        Serial.println("LEFT T-Junction DETECTED"); // DEBUGGING
        tjCounter();
        rotate_ccw(rotation_speed);
    }

    else if(ll_value == false && l_value == false && r_value == true && rr_value == true){ // 0011
        // Right T-junction
        Serial.println("RIGHT T-Junction DETECTED"); // DEBUGGING
        tjCounter();
        rotate_cw(rotation_speed);
    }

    else if(ll_value == true && l_value == true && r_value == true && rr_value == true){ // 1111
        // T_junction
        // t_junction_function();
        tjCounter();
    }
}

void grab_block() {
  if(grab == true) {
    set_motor_speed(CLAW_MOT, 100);
    set_motor_direction(CLAW_MOT, 1);
    grab == false;
    Serial.println("block grabbed");
  }
}

void drop_block() {
  if(drop == true) {
    lift_servo.write (lift_angle);
    delay(1000);
    if(distance_ultrasonic(echoPinFront) > release_distance) {
      set_motor_direction(LEFT_MOT, 1);
      set_motor_direction(RIGHT_MOT, 1);
    }
    else {
      set_motor_direction(LEFT_MOT, 1);
      set_motor_direction(RIGHT_MOT, 1);
      delay(500);
      set_motor_direction(CLAW_MOT, 0);
      Serial.println("block released");
    }
    drop = false;
  }
}


int stage_action(int delivery_stage){
    // ADD for stage shifts, e.g. time based, encoder value based, ultrasonic sensor value based
    switch (delivery_stage) {
        case 1: // Getting out of the drop-off/start box || TEST
            if (stage_start){
                // Start going forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                set_motor_speed(LEFT_MOT, 140); 
                set_motor_speed(RIGHT_MOT, 140); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                stage_start = false;
                Serial.println("Start"); // DEBUGGING
                }

            if (!reached_main_loop){
                if (ll_value || rr_value){tjCounter();} // Record the T-junction
                if (tjc == 2) {
                    reached_main_loop = true;
                    Serial.println("reached main loop");
                }
                break;
            }

            else if (!on_main_loop){
                if (ll_value && rr_value){
                    delay((int) line_crossing_delay / 3);
                    on_main_loop = true;
                    Serial.println("both on");
                    break;
                }
                else if (ll_value){
                    if (ll_rr_up_flag == 1){
                        on_main_loop = true;
                        Serial.println("right first");
                    }
                    ll_rr_up_flag = -1;
                    break;
                }
                else if (rr_value){
                    if (ll_rr_up_flag == -1){
                        on_main_loop = true;
                        Serial.println("left first");
                    }
                    ll_rr_up_flag = 1;
                    break;
                }
            }
                
            if (on_main_loop){
                Serial.println("on main loop");
                if (!rotating){
                    // Stop
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    // Start rotating
                    Serial.println("started rotating"); // DEBUGGING
                    rotate_ccw(145);
                    delay(line_crossing_delay * 2);
                    rotating = true;
                    break;
                }
                else if((!ll_value && !l_value && !r_value && !rr_value) || ll_value){
                    // Stop and go to the next stage
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    delivery_stage++;
                    stage_start = true;
                    break;
                }
            }
            break;
        case 2: // Getting to the tunnel || TEST
            if (stage_start){
                // Start moving forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                Serial.println("Start"); // DEBUGGING
                set_motor_speed(LEFT_MOT, 170); 
                set_motor_speed(RIGHT_MOT, 170); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                delay(line_crossing_delay); // to get off the initial line that connects start box to the main loop || TEST THE DELAY
                stage_start = false;
            }

            if (distance_ultrasonic(echoPinSide) <= 60){ // TEST THE DISTANCE
                // Ultrasonic sensor sees the wall of the tunnel, change the stage
                Serial.println("Reached the tunnel"); // DEBUGGING
                delivery_stage = 3;
                stage_start=true;
                break;
            }
            
            // Ignoring the drop-off junction 
            if(ll_value == true){tjCounter(); break;} // Avoid T-junction at the green drop-off || TEST
            
            follow_line(170, 150); // TEST THE SPEED
            break;
        case 3: // Going through the tunnel
            if (stage_start){
                // Start moving forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                set_motor_speed(LEFT_MOT, 170); 
                set_motor_speed(RIGHT_MOT, 170); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                stage_start = false;
                Serial.println("Start"); // DEBUGGING
            }

            if (distance_ultrasonic(echoPinSide) > 60){ // TEST THE DISTANCE
                // Ultrasonic sensor does not see the wall of the tunnel anymore, change the stage
                Serial.println("Out of the tunnel"); // DEBUGGING
                delivery_stage = 4;
                stage_start=true;
                break;
            }
            // Maybe add an additional check for the distance from the front wall
            break;
        case 4: // Locating the block and getting close to it
            if (stage_start){
                // Some kind of wiggly motion to find the line?
                stage_start = false;
                Serial.println("Start"); // DEBUGGING
            }
            
            if (tjc == 4){
                if (distance_ultrasonic(echoPinFront) < 150){ // TEST THE DISTANCE AND CLEARANCE FROM OTHER OBJECTS
                    Serial.println("Block located at the second location"); // DEBUGGING
                    // Block located
                    // Stop
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    delivery_stage = 52; // block located at the second pick-up location
                    stage_start=true;
                    break;
                }
            }
        
            if (rr_value){
                tjCounter();
                if (distance_ultrasonic(echoPinSide) < 150){ // TEST THE DISTANCE AND CLEARANCE FROM OTHER OBJECTS
                    // Block located and the robot is at the right position
                    // Stop
                    set_motor_direction(LEFT_MOT, 0);
                    set_motor_direction(RIGHT_MOT, 0);
                    if (tjc == 4){delivery_stage = 51; Serial.println("Block located at the first location");} // DEBUGGING // block located at the first pick-up location
                    else if (tjc == 6){delivery_stage = 53; Serial.println("Block located at the first location");} // DEBUGGING // block located at the third pick-up location
                    stage_start=true;
                    break;
                }
                else{
                    // An empty junction is reached, wait untill the sensor goes off the line and continue.
                    Serial.println("Skipping the junction");
                    delay(line_crossing_delay);
                    break;
                }
            }

            follow_line(170, 150);

            break;
        case 51: // First pick-up location. Picking up the block and getting outside the pick-up zone
            if (stage_start){
                // Some kind of wiggly motion to find the line?
                stage_start = false;
                Serial.println("Start"); // DEBUGGING
                rotate_cw(100);
            }
           
            if (distance_ultrasonic(echoPinFront) < 150){
                set_motor_direction(LEFT_MOT, 0);
                set_motor_direction(RIGHT_MOT, 0);
            }

            if (distance_ultrasonic(echoPinFront) > pick_up_distance){
                set_motor_direction(LEFT_MOT, 0);
                set_motor_direction(RIGHT_MOT, 0);
            }
            grab = true;
            

            break;
        case 52: // Second pick-up location. Picking up the block and getting outside the pick-up zone
        
            break;
        case 53: // Third pick-up location. Picking up the block and getting outside the pick-up zone
        
            break;
        /* 
        case 60:: locating the right drop off location
            //red area distance from wall = 745
            //green area distance from wall = 190  make variables for these!

            if (red_block && distance_ultrasonic(echoPinFront) < 760) {
              Serial.println("in range for red stop");
              set_motor_direction(LEFT_MOT, 0);
              set_motor_direction(RIGHT_MOT, 0);
              //initiate droppin sequence turn left soon when ll_value = 1  !
              stage_start = true;
            }
            if (green_block && distance_ultrasonic(echoPinFront) < 190) {
              Serial.println("in range for green stop");
              set_motor_direction(LEFT_MOT, 0);
              set_motor_direction(RIGHT_MOT, 0);
              //initiate droppin sequence turn left soon when ll_value = 1  !
              stage_start = true;
            }
            break
        
        case 70: // get to the right place drop the box drop the box
            if (stage_start){
                // Start turning left
                set_motor_speed(LEFT_MOT, 100); 
                set_motor_speed(RIGHT_MOT, 100); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                
                stage_start = false;
            }        
            if(distance_ultrasonic(echoPinFront) < drop_distance) {
              set_motor_direction(LEFT_MOT, 0);
              set_motor_direction(RIGHT_MOT, 0);
              stage_start = true; // final stage drop it

            

            }
        case 80:
          if (stage_start){
            drop = true: // added this bool but probably a better way to do it check!! also adding a function for drop
          }


        */
      

    }
}

void setup() {
    // test(); // To make initial tests, like run the motors, claw etc.
    // Motor setup
    Serial.begin(9600);
    AFMS.begin();
    pinMode(ll_pin, INPUT);
    pinMode(l_pin, INPUT);
    pinMode(r_pin, INPUT);
    pinMode(rr_pin, INPUT);
    pinMode(echoPinFront, INPUT);
    pinMode(echoPinSide, INPUT);
    pinMode(trigPin, OUTPUT);

    delay(15000);
    //set_motor_speed(LEFT_MOT, 170); 
    //set_motor_speed(RIGHT_MOT, 170); 
    //set_motor_direction(LEFT_MOT, -1);
    //set_motor_direction(RIGHT_MOT, -1);
    //delay(3000);
    set_motor_direction(LEFT_MOT, 0);
    set_motor_direction(RIGHT_MOT, 0);

    lift_servo.attach (9);
}

void loop() {
  // MODIFY THIS CODE AFTER TESTS
    ll_value = digitalRead(ll_pin);
    l_value = digitalRead(l_pin);
    r_value = digitalRead(r_pin);
    rr_value = digitalRead(rr_pin);
    if (l_value && !l_up){t_l = millis(); l_up = true;} 
    if (r_value && !r_up){t_r = millis(); r_up = true;} 
    if (!l_value){l_up = false;}
    if (!r_value){r_up = false;} 
    // test
    //follow_line(170, 145);
    // drop_block;
    stage_action(delivery_stage);
    Serial.println(delivery_stage);
} 
