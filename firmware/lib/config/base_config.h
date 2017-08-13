#ifndef BASE_CONFIG_H
#define BASE_CONFIG_H

#define DEBUG 0

float K_P = 6.0f; // P constant
float K_I = 0.5f; // I constant
float K_D = 0.0f; // D constant

//define your motors' specs here

const int MAX_RPM = 150; //motor's maximum RPM
const int COUNTS_PER_REV = 4480; //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
const float WHEEL_DIAMETER = 0.120; //wheel's diameter in meters

#define BASE_WIDTH 0.20 // width of the plate you are using

//PIEZO BEEPER PIN
#define PIEZO 11

//battery sensor pin
#define BATT_PIN A0
#define BATT_COEF 0.01937f


//ENCODER PINS
// left side encoders pins
#define MOTOR1_ENCODER_A 2 //front_A
#define MOTOR1_ENCODER_B 3 //front_B

// right side encoders pins
#define MOTOR2_ENCODER_A 7 //front_A
#define MOTOR2_ENCODER_B 6 //front_B

//don't change this if you followed the schematic diagram
//MOTOR PINS
//left side motor pins
#define MOTOR1_PWM 23
#define MOTOR1_IN_A 17
#define MOTOR1_IN_B 16

//right side motor pins
#define MOTOR2_PWM 21
#define MOTOR2_IN_A 13
#define MOTOR2_IN_B 12

#endif
