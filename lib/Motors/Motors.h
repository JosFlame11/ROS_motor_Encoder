#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

class Motor{
    public:

    Motor(int PWM, int IN1, int IN2, int pwmChannel); //For TB6612FNG like drivers that requires 1 pwm, and 2 inputs
    Motor(int PWM, int IN1, int IN2, int pwmChannel, int res, int freq); //For TB6612FNG like drivers that requires 1 pwm, and 2 inputs
    Motor(int Dir, int PWM, int pwmChannel); //For drivers that only need a Direction pin and a PWM pin
    Motor(int Dir, int PWM, int pwmChannel, int res, int freq); //For drivers that only need a Direction pin and a PWM pin
    void setMotorSpeed(int vel);
    void setMaxSpeed(int max_speed);


    private:

    int _pwmPin;
    int _dirPin;
    int _in1Pin;
    int _in2Pin;
    int _vel;

    int _resolution = 8;
    int _frequency = 1000;
    int _pwmChannel = 0;

    int MAX_VEL = 255;
};








#endif