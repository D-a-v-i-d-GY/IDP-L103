#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

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

// GEOMETRICAL CHARACTERISTICS, ROBOT CHARACTERISTICS
#define delay_per_degree 12.9 // At 150 motor speed (NEED TO MEASURE) (DO NOT USE?)
#define wheel_dist 220 // Distance between the centers of the wheels, NEED TO MEASURE
#define llrr 75 // Distance between left-most and right-most sensors, NEED TO VERIFY
#define lr 25 // Distance between left and right sensors, NEED TO VERIFY
#define line_width 19 // Width of the white line, NEED TO VERIFY
#define pi 3.141593

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* right_motor = AFMS.getMotor(1);
Adafruit_DCMotor* left_motor = AFMS.getMotor(2);
Adafruit_DCMotor* motors[] = {left_motor, right_motor}; // Make an array with the mototrs for convenient handling

// Useful flags
int stage = 0; // Counter that determines the stage of the delivery process, robot's actions are determined by the stage number
int tjc = 0; // T joint count (as seen from robot's perspective)
bool stage_start = true; // Indicator for the start of a new stage

// Variable init
int motor_speeds[] = {0, 0};
int motor_directions[] = {0, 0};
float motor_velocities[] = {0.0, 0.0};
float orientation;
bool ll_value;
bool l_value;
bool r_value;
bool rr_value;
unsigned long t0 = 0;
unsigned long t = 0;

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
    t = millis();
    float dt = (t - t0) / 1000;
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
            Serial.println("Speed set!"); // For debugging
        }
        else{Serial.println("Speed is already set!")}; // For debugging}
    }

    else{
        Serial.println("motor should be either 0 or 1"); // For debugging
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
                Serial.println("Direction Set!"); // For debugging
            }
            else if (direction == 0){
                motors[motor]->run(RELEASE);
                Serial.println("Direction Set!"); // For debugging
            }
            else if (direction == -1){
                motors[motor]->run(BACKWARD);
                Serial.println("Direction Set!"); // For debugging
            }
            else { 
                // Check for correct direction assignment
                Serial.println("Direction not defined"); // For debugging
                return;
            }
        }
        else{
          Serial.println("Direction is already set"); // For debugging
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
  Serial.println("Rotating CCW"); // DEBUGGING
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, -1);
    set_motor_direction(RIGHT_MOT, 1);
}

// NEEDS TEST
void rotate_cw(int speed){
  Serial.println("Rotating CW"); // DEBUGGING
    set_motor_speed(LEFT_MOT, speed); 
    set_motor_speed(RIGHT_MOT, speed); 

    set_motor_direction(LEFT_MOT, 1);
    set_motor_direction(RIGHT_MOT, -1);
}

// DONE? NEEDS A LOT OF TEST!!!
void follow_line(int forward_speed, int rotation_speed){
    if(ll_value == false && l_value == false && r_value == false && rr_value == false){
        Serial.println("Moving straight");// DEBUGGING
        set_motor_speed(LEFT_MOT, forward_speed); 
        set_motor_speed(RIGHT_MOT, forward_speed); 
        set_motor_direction(LEFT_MOT, 1);
        set_motor_direction(RIGHT_MOT, 1);
    }

    else if(ll_value == false && l_value == true && r_value == false && rr_value == false){
        rotate_ccw(rotation_speed);
    }

    else if(ll_value == false && l_value == false && r_value == true && rr_value == false){
        rotate_cw(rotation_speed);
    }

    else if(ll_value == true && l_value == false && r_value == false && rr_value == false){
        rotate_ccw(rotation_speed);
    }

    else if(ll_value == false && l_value == false && r_value == false && rr_value == true){
        rotate_cw(rotation_speed);
    }

    else if(ll_value == true && l_value == true && r_value == false && rr_value == false){
        // Left T-junction
        Serial.println("LEFT T-Junction DETECTED"); // DEBUGGING
        tjc++;
        rotate_ccw(rotation_speed);
    }

    else if(ll_value == false && l_value == false && r_value == true && rr_value == true){
        // Right T-junction
        Serial.println("RIGHT T-Junction DETECTED"); // DEBUGGING
        tjc++;
        rotate_cw(rotation_speed);
    }

    else if(ll_value == true && l_value == true && r_value == true && rr_value == true){
        // T_junction
        // t_junction_function();
    }
}

int stage_action(int stage){
  Serial.print("Current Stage: ");
  Serial.println(stage);
    // ADD for stage shifts, e.g. time based, encoder value based, ultrasonic sensor value based
    switch (stage) {
        case 0: // Getting out of the drop-off/start box
            if (stage_start){
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                stage_start = false;
                }
            // ==========================================================

            if (ll_value && l_value && r_value && rr_value){tjc++;} // Record the T joint
            if (tjc == 2) {
                set_motor_direction(LEFT_MOT, 0);
                set_motor_direction(RIGHT_MOT, 0);
                rotate_ccw();
                stage++;
                stage_start = true;
            }
            break;
        case 1: // Getting to the tunnel
            if (stage_start){
                set_motor_speed(LEFT_MOT, 250); 
                set_motor_speed(RIGHT_MOT, 250); 
                set_motor_direction(LEFT_MOT, 1);
                set_motor_direction(RIGHT_MOT, 1);
                delay(20);
            }

            if (-10 < orientation < 10){
                rotate_angle(-orientation);
            }
            if (distance_ultrasonic("side") <= 60){
                stage++;
                stage_start=true;
                break;
            }
            if(ll_value == true && l_value == true && r_value == false && rr_value == false){break;}
            follow_line();
            break;
        case 2: // Going through the tunnel
            break;
    }
  return stage;
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

    set_motor_speed(LEFT_MOT, 240); 
    set_motor_speed(RIGHT_MOT, 240); 
    set_motor_direction(LEFT_MOT, 0);
    set_motor_direction(RIGHT_MOT, 0);
    delay(3000);
}

void loop() {
  // MODIFY THIS CODE AFTER TESTS
    // ll_value = digitalRead(ll_pin);
    // l_value = digitalRead(l_pin);
    // r_value = digitalRead(r_pin);
    // rr_value = digitalRead(rr_pin);

    // follow_line();
    // stage = stage_action(stage);
} 
