/*
 Copyright (c) 2016, Juan Jimeno

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of  nor the names of its contributors may be used to
 endorse or promote products derived from this software without specific
 prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#define ENCODER_OPTIMIZE_INTERRUPTS

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 100 //hz
#define BATTERY_PUBLISH_RATE 1 //hz
#define DEBUG_RATE 2
#define BEEP_DURATION 200 //ms

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include "base_config.h"
#include "Encoder.h"
#include "Motor.h"

#include <ros.h>
//TODO: change to specific message type
//header file for publishing Odom Velocities
#include <geometry_msgs/Vector3Stamped.h>

//header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Twist.h>

//header files for imu
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

#include <ros/time.h>

#include <cbbot_msgs/cfgPID.h>
#include <cbbot_msgs/CBService.h>
#include <sensor_msgs/BatteryState.h>

//MPU instance
MPU6050 accelgyro;
//copied from imu includes:
geometry_msgs::Vector3 raw_acceleration;
geometry_msgs::Vector3 raw_rotation;
geometry_msgs::Vector3 raw_magnetic_field;
//temp vars for imu
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

bool is_first = true;
boolean imu_enabled = false;

//battery variables
double voltage = 0;
long batt_measure_counter = 0;

//left side motors
Motor motor1(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B); //front
//right side motors
Motor motor2(MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); // front

//left side encoders
Encoder motor1_encoder(MOTOR1_ENCODER_A,MOTOR1_ENCODER_B); //front
//right side encoders
Encoder motor2_encoder(MOTOR2_ENCODER_A,MOTOR2_ENCODER_B); //front

float Motor::Kp = K_P;
float Motor::Kd = K_D;
float Motor::Ki = K_I;

int Motor::max_rpm = MAX_RPM;
int Motor::counts_per_rev = COUNTS_PER_REV;
float Motor::wheel_diameter = WHEEL_DIAMETER;

int left_pwm = 0;
int right_pwm = 0;

unsigned int cmd_timeout = 400;

double required_angular_vel = 0;
double required_linear_vel = 0;
unsigned long previous_command_time = 0;
unsigned long previous_control_time = 0;
unsigned long publish_vel_time = 0;
unsigned long previous_imu_time = 0;
unsigned long previous_debug_time = 0;
unsigned long beep_start_time = 0;
unsigned long previous_batt_time = 0;
char buffer[50];

//callback function prototypes
void command_callback( const geometry_msgs::Twist& cmd_msg);
void pid_callback( const cbbot_msgs::cfgPID& pid);
void srv_cmd_callback( const cbbot_msgs::CBService& srv_cmd);

ros::NodeHandle nh;
//TODO: make topics configurable
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<cbbot_msgs::cfgPID> pid_sub("pid", pid_callback);
ros::Subscriber<cbbot_msgs::CBService> srv_cmd_sub("srv_cmd", srv_cmd_callback);

ros_arduino_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

geometry_msgs::Vector3Stamped raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

sensor_msgs::BatteryState battery_state_msg;
ros::Publisher batt_pub("battery_state", &battery_state_msg);

void setup(){
  analogWriteFrequency(MOTOR1_PWM, 1000);//increase pwm frequency
  analogWriteFrequency(MOTOR2_PWM, 1000);
  pinMode(PIEZO, OUTPUT);
  beep_off();//quiet
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.subscribe(srv_cmd_sub);
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);
  nh.advertise(batt_pub);

  while (!nh.connected()){
    nh.spinOnce();
  }
  nh.loginfo("CBBASE CONNECTED");

  init_battery_msg();

  Wire.begin();
  accelgyro.initialize();
  accelgyro.setRate(7);
  accelgyro.setDLPFMode(4);
  accelgyro.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec
  accelgyro.setFullScaleAccelRange(0); // set accelerometer to 2 g range
  accelgyro.setIntDataReadyEnabled(true);
  delay(5);
}

void loop(){
  //this block drives the robot based on defined rate
  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
    do_kinematics();
    move_base();
    previous_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - previous_command_time) >= cmd_timeout){
    stop_base();
  }

  //this block publishes velocity based on defined rate
  if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
    publish_linear_velocity();
    publish_vel_time = millis();
  }

  //check battery state
  if ((millis() - previous_batt_time) >= (1000 / BATTERY_PUBLISH_RATE)){
    publish_battery_v();
    previous_batt_time = millis();
  } else if ((millis() - previous_batt_time) >= 100){
    check_battery_v();
  }

  if (is_first){
    check_imu();
  }else if(imu_enabled){
    get_imu();
  }

  //TODO: Disable this block if imu not initialized
  //this block publishes the IMU data based on defined rate
  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
    publish_imu();
    previous_imu_time = millis();
  }

  if(beep_start_time != 0 && ((millis() - beep_start_time)>=BEEP_DURATION)){
    beep_off();
  }

  //Change DEBUG to 0 if you don't want to display info
  if(DEBUG){
    if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)){
      print_debug();
      previous_debug_time = millis();
    }
  }

  //call all the callbacks waiting to be called
  nh.spinOnce();
}

void pid_callback( const cbbot_msgs::cfgPID& pid){
  Motor::Kp = pid.p;
  Motor::Kd = pid.d;
  Motor::Ki = pid.i;
}

void command_callback( const geometry_msgs::Twist& cmd_msg){
  required_linear_vel = cmd_msg.linear.x;
  required_angular_vel = cmd_msg.angular.z;

  previous_command_time = millis();
}

void srv_cmd_callback( const cbbot_msgs::CBService& srv_cmd){
  uint8_t cmd = srv_cmd.cmd;
  switch (cmd) {
  case 21:
    beep_on();
    break;
  default:
    sprintf(buffer, "Unknown command: %d", cmd);
    nh.logwarn(buffer);
    break;
  }
}

void do_kinematics(){
  //convert m/s to m/min
  double linear_vel_mins = required_linear_vel * 60;
  //convert rad/s to rad/min
  double angular_vel_mins = required_angular_vel * 60;
  //calculate the wheel's circumference
  double circumference = PI * WHEEL_DIAMETER;
  //calculate the tangential velocity of the wheel if the robot's rotating where Vt = ω * radius
  double tangential_vel = angular_vel_mins * BASE_WIDTH;

  //calculate and assign desired RPM for left and right motor
  motor1.required_rpm = (linear_vel_mins / circumference) - (tangential_vel / circumference);
  motor2.required_rpm = (linear_vel_mins / circumference) + (tangential_vel / circumference);
}

void move_base(){
  motor1.calculate_rpm(motor1_encoder.read());
  motor2.calculate_rpm(motor2_encoder.read());

  left_pwm = motor1.calculate_pwm();
  right_pwm = motor2.calculate_pwm();

  motor1.spin(left_pwm);
  motor2.spin(right_pwm);
}

void stop_base(){
  required_linear_vel = 0;
  required_angular_vel = 0;
}

void publish_linear_velocity(){
  //calculate the average RPM
  double average_rpm = (motor1.current_rpm + motor2.current_rpm) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  double average_rps = average_rpm / 60; // RPS
  //calculate linear speed
  double linear_velocity = (average_rps * (WHEEL_DIAMETER * PI)); // m/s

  //fill in the object
  raw_vel_msg.header.stamp = nh.now();
  raw_vel_msg.vector.x = linear_velocity;
  raw_vel_msg.vector.y = 0.00;
  raw_vel_msg.vector.z = 0.00;

  //publish raw_vel_msg object to ROS
  raw_vel_pub.publish(&raw_vel_msg);
}

//this function checks if IMU is present
void check_imu(){
  imu_enabled = accelgyro.testConnection();

  raw_imu_msg.accelerometer = imu_enabled;
  raw_imu_msg.gyroscope = imu_enabled;
  raw_imu_msg.magnetometer = imu_enabled;

  if (!imu_enabled){
    nh.logerror("IMU NOT FOUND!");
  }

  is_first = false;
}

// get raw data from IMU sensor
void get_imu(){
  if(imu_enabled && accelgyro.getIntDataReadyStatus() == 1) { // wait for data ready status register to update all data registers
      accelgyro.getAcceleration( &ax, &ay, &az );
      raw_acceleration.x = ax*2.0f/32768.0f; // 2 g full range for accelerometer
      raw_acceleration.y = ay*2.0f/32768.0f;
      raw_acceleration.z = az*2.0f/32768.0f;

      accelgyro.getRotation( &gx, &gy, &gz );
      raw_rotation.x = (gx*250.0f/32768.0f)*M_PI/180.0f; // 250 deg/s full range for gyroscope
      raw_rotation.y = (gy*250.0f/32768.0f)*M_PI/180.0f;
      raw_rotation.z = (gz*250.0f/32768.0f)*M_PI/180.0f;

      accelgyro.getMag( &mx, &my, &mz );
      raw_magnetic_field.y = mx*10.0f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
      raw_magnetic_field.x = my*10.0f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
      raw_magnetic_field.z = mz*10.0f*1229.0f/4096.0f + 270.0f;

      raw_imu_msg.raw_linear_acceleration = raw_acceleration;
      raw_imu_msg.raw_angular_velocity = raw_rotation;
      raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
  }
}

//publish raw_imu_msg object to ROS
void publish_imu(){
  if (raw_imu_msg.accelerometer){
    //this function publishes raw IMU reading
    raw_imu_msg.header.stamp = nh.now();
    raw_imu_msg.header.frame_id = "imu_link";
    raw_imu_pub.publish(&raw_imu_msg);
  }
}

//piezo beep sound
void beep_on(){
  beep_start_time = millis();
  digitalWrite(PIEZO, HIGH);
}

void beep_off(){
  digitalWrite(PIEZO, LOW);
  beep_start_time = 0;
}

//battery state
void init_battery_msg(){
  // Initialize Battery States
  battery_state_msg.current         = NAN;
  battery_state_msg.charge          = NAN;
  battery_state_msg.capacity        = 5800; //mAh
  battery_state_msg.design_capacity = 6000; //mAh
  battery_state_msg.percentage = NAN;
  battery_state_msg.power_supply_status = 2; //discharging
  battery_state_msg.power_supply_health = 0; //unknown
  battery_state_msg.power_supply_technology = 3; //LiPo
  battery_state_msg.present = 1;
}

void check_battery_v(){
  voltage += analogRead(BATT_PIN) * BATT_COEF;
  batt_measure_counter++;
}

void publish_battery_v(){
  float v = (float)voltage/(float)batt_measure_counter;
  battery_state_msg.voltage = v;
  battery_state_msg.header.stamp = nh.now();
  voltage = 0;
  batt_measure_counter = 0;
  //publish battery_state_msg object to ROS
  batt_pub.publish(&battery_state_msg);
}

// prints RPM and encoder debug info
void print_debug(){
  sprintf (buffer, "Encoder L: %ld | R: %ld", motor1_encoder.read(), motor2_encoder.read());
  nh.loginfo(buffer);
  char str_m1[6];
  char str_m2[6];
  dtostrf(motor1.required_rpm, 4, 2, str_m1);
  dtostrf(motor2.required_rpm, 4, 2, str_m2);
  sprintf (buffer, "REQ RPM L: %s | R: %s", str_m1, str_m2);
  nh.loginfo(buffer);
  dtostrf(motor1.current_rpm, 4, 2, str_m1);
  dtostrf(motor2.current_rpm, 4, 2, str_m2);
  sprintf (buffer, "CUR RPM L: %s | R: %s", str_m1, str_m2);
  nh.loginfo(buffer);
  sprintf (buffer, "CUR PWM L: %d | R: %d", left_pwm, right_pwm);
  nh.loginfo(buffer);
}
