#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define left_ls_pin 13; // Left line sensor pin number
#define central_ls_pin 12; // Central line sensor pin number
#define right_ls_pin 11; // Right line sensor pin number
#define echoPin 8; // Echo Pin for the ultrasonic sensor
#define trigPin 7; // Trigger Pin for the ultrasonic sensor

// IMPORTANT CONSTANTS
#define delay_per_degree 20 // Needs to be measured
#define LEFT_MOT 0 // Index of the left motor
#define RIGHT_MOT 1 // Index of the right motor

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor motors[] = {left_motor, right_motor} // Make an array with the mototrs for convenient handling

// Useful flags
int stage = 0; // Counter that determines the stage of the delivery process, robot's actions are determined by the stage number
int tjc = 0; // T joint count (as seen from robot's perspective)
int ljc = 0; // L joint count (as seen from robot's perspective)

// Variable init
int motor_speeds[] = {0, 0}
int motor_directions[] = {0, 0}


void set_motor_speed(motor, speed){
    // This function sets the speed of the given driving motor, 0 for left and 1 for right

    // test that the speed value is in the right 
    if (speed < 0 || speed > 255){
        break;
        Serial.println("Speed out of range")
    }

    if (motor = 1 || motor == 0){
        // Don't change the speed if it is the same 
        if (motors[motor] != speed){
            motors[motor]->setSpeed(speed); motor_speeds[motor] = speed;
        }
    }

    else{
        Serial.println("motor should be either 0 or 1")
    }
}

void set_motor_direction(motor, direction){
    if (motor = 1 || motor == 0){
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
                Serial.println("Direction not defined")
                break;
            }
        }

        motor_directions[motor] = direction;
        else{
            Serial.println("motor should be either 0 or 1")
        }
    }
}

void rotate(angle){
    // This function makes the robot rotate around the center of the wheel axis by the given angle, CCW is assumed positive

    set_motor_speed(LEFT_MOT, 150); 
    set_motor_speed(RIGHT_MOT, 150); 

    set_motor_direction(LEFT_MOT, -1)
    set_motor_direction(RIGHT_MOT, 1)

    delay(angle * delay_per_degree);
    
    set_motor_direction(LEFT_MOT, 0)
    set_motor_direction(RIGHT_MOT, 0)
}

void setup() {
    // test(); // To make initial tests, like run the motors, claw etc.
    // Motor setup
    set_motor_speed(LEFT_MOT, 150); 
    set_motor_speed(RIGHT_MOT, 150); 
    set_motor_direction(LEFT_MOT, -1)
    set_motor_direction(RIGHT_MOT, -1)
    delay(3000);
}

void loop() {
    bool left_ls_value = digitalRead(left_ls);
    bool central_ls_value = digitalRead(central_ls);
    bool right_ls_value = digitalRead(right_ls);
    stage = stage_action(stage);
} 

int stage_action(stage){
    switch (stage) {
        case 0: // Getting out of the drop-off/start box
            set_motor_direction(LEFT_MOT, 1)
            set_motor_direction(RIGHT_MOT, 1)
            // ==========================================================
            // ask Tim Love if it is better to call a function each time or to write a separate code

            if (left_ls_value && central_ls_value && right_ls_value){tjc++;} // Record the T joint
            if (tjc == 2) {
                set_motor_direction(LEFT_MOT, 0)
                set_motor_direction(RIGHT_MOT, 0)
                rotate(90);
                stage++;
            }
            break;
        case 1:
            if (left_motor_direction != 1 || right_motor_direction != 1){
                set_motor_speed(LEFT_MOT, 250); 
                set_motor_speed(RIGHT_MOT, 250); 
                set_motor_direction(LEFT_MOT, 1)
                set_motor_direction(RIGHT_MOT, 1)
            }

            follow_line(left_ls_value, central_ls_value, right_ls_value);

            if ((left_ls_value && central_ls_value) || (right_ls_value && central_ls_value))

    }
}