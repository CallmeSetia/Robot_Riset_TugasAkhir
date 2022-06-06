#include "Motor.h"

void Motor::pin(int _pin1, int _pin2){
  this->pin1 = _pin1;
  this->pin2 = _pin2;
}

void Motor::enc(int _enc1, int _enc2){
  this->enc1 = _enc1;
  this->enc2 = _enc2;  
}

void Motor::pwm_ch(int pwm_pin1, int pwm_pin2){
  this->pwm_ch1 = pwm_pin1;
  this->pwm_ch2 = pwm_pin2;
}
