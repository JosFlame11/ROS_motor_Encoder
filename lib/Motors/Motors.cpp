#include "Motors.h"
#include <Arduino.h>

Motor::Motor(int PWM, int IN1, int IN2, int pwmChannel){

    _pwmPin = PWM;
    _in1Pin = IN1;
    _in2Pin = IN2;
    _pwmChannel = pwmChannel;

    pinMode(_pwmPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);

    ledcSetup(_pwmChannel, _frequency, _resolution);
    ledcAttachPin(_pwmPin, _pwmChannel);

}
Motor::Motor(int PWM, int IN1, int IN2, int pwmChannel, int res, int freq){

    _pwmPin = PWM;
    _in1Pin = IN1;
    _in2Pin = IN2;
    _pwmChannel = pwmChannel;
    _resolution = res;
    _frequency = freq;


    pinMode(_pwmPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);

    ledcSetup(_pwmChannel, _frequency, _resolution);
    ledcAttachPin(_pwmPin, _pwmChannel);
}

Motor::Motor(int Dir, int PWM, int pwmChannel){

    _pwmPin = PWM;
    _dirPin = Dir;
    _pwmChannel = pwmChannel;

    #define isDir 1

    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    ledcSetup(_pwmChannel, _frequency, _resolution);
    ledcAttachPin(_pwmPin, _pwmChannel);
}

Motor::Motor(int Dir, int PWM, int pwmChannel, int res, int freq){

    _pwmPin = PWM;
    _dirPin = Dir;
    _pwmChannel = pwmChannel;
    _resolution = res;
    _frequency = freq;

    #define isDir 1

    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    ledcSetup(_pwmChannel, _frequency, _resolution);
    ledcAttachPin(_pwmPin, _pwmChannel);
}

void Motor::setMotorSpeed(int vel){

    _vel = constrain(vel, -MAX_VEL, MAX_VEL);
    
    #if defined(isDir)
        (_vel > 0) ? digitalWrite(_dirPin, HIGH) : digitalWrite(_dirPin, LOW);

    #else
    if(_vel > 0){
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
    }
    else{
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
    }
    #endif

    ledcWrite(_pwmChannel, _vel);
}

void Motor::setMaxSpeed(int max_speed){
    MAX_VEL = max_speed;
}