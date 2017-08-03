#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int pwm_pin, int motor_pinA, int motor_pinB)
{
    pinMode(pwm_pin, OUTPUT);
    pinMode(motor_pinA, OUTPUT);
    pinMode(motor_pinB, OUTPUT);

    _pwm_pin = pwm_pin;
    _motor_pinA = motor_pinA;
    _motor_pinB = motor_pinB;
}

void Motor::calculate_rpm(long current_encoder_ticks)
{
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - _previous_rpm_time;
    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000;
    //calculate change in number of ticks from the encoder
    double delta_ticks = current_encoder_ticks - _previous_encoder_ticks;
    //calculate wheel's speed (RPM)
    current_rpm = (delta_ticks / double(counts_per_rev))/ dtm;
    _previous_encoder_ticks = current_encoder_ticks;
    _previous_rpm_time = current_time;
}

int Motor::calculate_pwm()
{
  // this function implements a PID controller and calculates the PWM required
  // to drive the motor
  double error;
  double pwm;
  //double rpm_signal;

  //required_rpm is constrained to max_rpm to prevent pid from having too much error
  error = constrain(required_rpm, -max_rpm, max_rpm) - current_rpm;
  _total_pid_error += error;

  //if(required_rpm == 0 || error == 0) //experimental
  if(error == 0) //experimental
  {
    _total_pid_error = 0;
  }

  pwm = (Kp * error) + (Ki * _total_pid_error) + (Kd * (error - _previous_pid_error));
  _previous_pid_error = error;

  //pwm = (((double) pwm / (double) max_rpm) * 255);
  //make sure calculated pwm value is within PWM range
  return pwm; //constrain(pwm, -255, 255);
}

void Motor::spin(int pwm)
{
    //this function spins the motor based on calculated PWM
    if(pwm > 0)
    {
        digitalWrite(_motor_pinA , HIGH);
        digitalWrite(_motor_pinB , LOW);
    }
    else if(pwm < 0)
    {
        digitalWrite(_motor_pinA , LOW);
        digitalWrite(_motor_pinB , HIGH);
    }
    analogWrite(_pwm_pin , abs(pwm));
}
