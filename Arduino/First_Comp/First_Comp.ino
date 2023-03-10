#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// ALL LENGTHS ARE IN MM!!

// PINS
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

// GEOMETRICAL CHARACTERISTICS, ROBOT CHARACTERISTICS
#define delay_per_degree 12.9 // At X motor speed (NEED TO MEASURE) (DO NOT USE?)
#define stagnation_time 500 // Time allowed for a line sensor to be continuously active || TEST
#define tj_detection_interval 600 // Minimum time difference between consecutive detection of t-jucntions || TEST
#define line_crossing_delay 600 // Minimum time delay that ensures that the line sensor goes through the line without getting activated || TEST
#define wheel_dist 220 // Distance between the centers of the wheels, NEED TO MEASURE
#define llrr 75 // Distance between left-most and right-most sensors, NEED TO VERIFY
#define lr 25 // Distance between left and right sensors, NEED TO VERIFY
#define line_width 19 // Width of the white line, NEED TO VERIFY
#define pick_up_distance 60
#define pi 3.141593
#define red_area_d 745
#define greed_area_d 190
#define grab_angle 60
#define drop_distance 60 // random guess gotta check tghis the distance we initiate drop sequence
#define release_distance 30 // again gotta check when we drop it
#define tunnel_line_distance 100

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* right_motor = AFMS.getMotor(1);
Adafruit_DCMotor* left_motor = AFMS.getMotor(2);

Adafruit_DCMotor* motors[] = {left_motor, right_motor}; // Make an array with the mototrs for convenient handling


Servo grab_servo;  // attach servo to pin 9 in setup havent done yet not sure if thats how it works

// Useful flags
int delivery_stage = 1; // Counter that determines the stage of the delivery process, robot's actions are determined by the stage number
int tjc = 0; // T joint count (as seen from robot's perspective)
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
float tunnel_wall_distance = 0;
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
unsigned long t_stage_st = 0;

void tjCounter(){
    // Counter of T-junctions, with protection against excessive counting
    if (millis() - t_tjc > tj_detection_interval){tjc++; t_tjc = millis();}
    Serial.println("T junction encountered!"); // DEBUGGING
    Serial.println(tjc); // DEBUGGING
}

bool sensorRead(int pin){
    bool reading;
    int out = digitalRead(pin);
    if (out == 0){reading = false;}
    else{reading = true;}

    return reading;
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

// TEST IF IT IS BETTER TO HAVE ONE ECHO PIN OR ONE TRIG PIN
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
    Serial.println(digitalRead(pinNumber));
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

// NEEDS PRECISE TESTS
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

// TESTED
void rotate_ccw(int speed){
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, 1);
    set_motor_direction(RIGHT_MOT, -1);
}

// TESTED
void rotate_cw(int speed){
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, -1);
    set_motor_direction(RIGHT_MOT, 1);
}

// TESTED (EXCEPT STAGNATION!!!), WORKS FINE
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
    grab == false;
    grab_servo.write(grab_angle);
    delay(1000);
    Serial.println("block grabbed");
  }
}

void drop_block() {
  if(drop == true) {
    drop == false;
    grab_servo.write(0);
    delay(1000);
    Serial.println("block dropped");
  }

}


void stage_action_first_comp(){
    switch(delivery_stage){
    // ADD for stage shifts, e.g. time based, encoder value based, ultrasonic sensor value based
        case 1: // Getting out of the drop-off/start box || TEST
            if (stage_start){
                // Start going forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                stage_start = false;
                Serial.println("Start"); // DEBUGGING
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
                Serial.println("Start"); // DEBUGGING
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
            if ((millis() - t_stage_st > 1500)){
              if(rr_value == true){
                Serial.println(millis()); 
                tjCounter(); 
                delay(line_crossing_delay); 
                Serial.println("Drop-off tjc"); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                delivery_stage = 3; 
                stage_start = true;
                break;
              }
            } // Avoid T-junction at the green drop-off || TEST
            // tjc is equal to 3 after the drop-off junction has been passed

            follow_line(200, 200); // TEST THE SPEED
            break;
        case 3: // Going over the ramp
            if (stage_start){
                // Start moving forward
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                stage_start = false;
                t_stage_st = millis();
                Serial.println("Start"); // DEBUGGINGx
                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200);                 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                delay(line_crossing_delay);
            }

            if (millis() - t_stage_st > 5000){delivery_stage = 4; stage_start = true; break;}

            follow_line(250, 180);
            break;
        case 4: // Going through pick-up locations and thunnel
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
            }

            if (tjc == 8){ // Arrived at the starting position
                set_motor_direction(LEFT_MOT, 0);
                set_motor_direction(RIGHT_MOT, 0);
                delivery_stage = 5; delay(500); stage_start = true; break;
            }
            
            if (rr_value || ll_value){
              tjCounter();
              set_motor_direction(LEFT_MOT, 1);
              set_motor_direction(RIGHT_MOT, 1); 
              delay(500); break;}
            
            follow_line(200, 200);

            break;
        case 5: // First pick-up location. Picking up the block and getting outside the pick-up zone
            if (stage_start){
                // Some kind of wiggly motion to find the line?
                Serial.print("Current Stage: "); // DEBUGGING
                Serial.println(delivery_stage); // DEBUGGING
                stage_start = false;
                Serial.println("Start"); // DEBUGGING
                rotate_cw(200);
                delay((int) line_crossing_delay * 2);
                // Stop and go to the next stage
                set_motor_speed(LEFT_MOT, 200); 
                set_motor_speed(RIGHT_MOT, 200); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                delay(3000);
                set_motor_direction(LEFT_MOT, 0);
                set_motor_direction(RIGHT_MOT, 0);
                break;
            }
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
    grab_servo.attach(9);

    set_motor_direction(LEFT_MOT, 0);
    set_motor_direction(RIGHT_MOT, 0);
    delay(6000);
}

void loop() {
  // MODIFY THIS CODE AFTER TESTS
    ll_value = sensorRead(ll_pin);
    l_value = sensorRead(l_pin);
    r_value = sensorRead(r_pin);
    rr_value = sensorRead(rr_pin);
    if (l_value && !l_up){t_l = millis(); l_up = true;} 
    if (r_value && !r_up){t_r = millis(); r_up = true;} 
    if (!l_value){l_up = false;}
    if (!r_value){r_up = false;}
    //Serial.print(ll_value);
    //Serial.print(l_value);
    //Serial.print(r_value);
    //Serial.println(rr_value);
    //delay(200);

    stage_action_first_comp();
    //follow_line(200, 180);
}
