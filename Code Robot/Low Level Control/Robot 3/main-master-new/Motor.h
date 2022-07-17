#ifndef MOTOR_H
#define MOTOR_H

#define RESOLUSI_10_BIT 10

class Motor {


  public:
    int pin1;
    int pin2;

    int enc1;
    int enc2;

    int pwm_ch1;
    int pwm_ch2;
    
    void pin(int _pin1, int _pin2);
    void enc(int _enc1, int _enc2);
    void pwm_ch(int pwm_pin1, int pwm_pin2);
};

#endif
