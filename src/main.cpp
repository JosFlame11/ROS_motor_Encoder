#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID.h>
// #include <Motors.h>
#include <filters.h>
#include <BasicLinearAlgebra.h>
#include "StateFeedbackController.h"




//Left motor pins
#define EnA 2  // 36
#define In1 13  //35
#define In2 12  //34
// Define pins for the encoder of Left motor
const int encLeftA = 7;  //7
const int encLeftB = 3;  //6
// Motor pwmChannel
const int MC0 = 0;

//Rigth motor pins
#define In3 14   //33
#define In4 21   //47
#define Enb 1   //48
// Define pins for the encoder of Right motor
const int encRightA = 10;   //5
const int encRightB = 11;   //4
// Motor pwmChannel
const int MC1 = 1;

// Create an Encoder object
ESP32Encoder LeftEnc;
ESP32Encoder RightEnc;

// Create PID gains struct
struct PIDgains{
  double Kp;
  double Ki;
  double Kd;
  PIDgains(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {}
};

PIDgains leftGains(1.331, 66.56, 0.0);
PIDgains rightGains(2.683, 134.17, 0.0);

// Pulses 
double leftRPM;
double rightRPM;

double leftVel_rad;
double rightVel_rad;

// Desire velocity
double leftPID_output;
double rightPID_output;

// Velocity changing overtime
double leftSpeed = 0;
double rightSpeed = 0;

int vel = 0;
// Create PID objects for each motor
// PID objects
PID leftPID(&leftVel_rad, &leftPID_output, &leftSpeed, leftGains.Kp, leftGains.Ki, leftGains.Kd);
PID rightPID(&leftVel_rad, &rightPID_output, &rightSpeed, rightGains.Kp, rightGains.Ki, rightGains.Kd);


// State-space equation
// double A_d[3][3] = {{-204.9, -798.2, -171.8}, {512, 0, 0}, {0, 256, 0}};
// double B_d[3] = {4, 0, 0};
// double C_d[3] = {0.9374, -0.3582, 2.908};
// double D_d = 0;
// double K[3] = {98.7750 -153.6563  -31.3202}; // State feedback gain
// double Ki_L = -82.9183;    // Integral gain
// double L[3] = {1178.1,200.8,-404.8}; // Observer gain

using namespace BLA;

BLA::Matrix<3, 3, double> A_L = { -204.9, -798.2, -171.8,
                         512, 0, 0,
                         0, 256, 0 };
BLA::Matrix<3, 1, double> B_L = { 4, 0, 0 };
BLA::Matrix<1, 3, double> C_L = { 0.9374, -0.3582, 2.908 };
double D_L = -0.0006;
BLA::Matrix<1, 3, double> Kx_L = {23.7750, -186.8543, -40.4584};
double Ki_L = -11.6604;
BLA::Matrix<3, 1, double> L = {1.3643, -0.3678, -0.4111};
// State and observer variables
BLA::Matrix<3, 1, double> x_hat_l = { 0, 0, 0 };  // Estimated state
BLA::Matrix<3, 1, double> x_dot_l = { 0, 0, 0 };  // State derivative
double xi_L = 0;                          // Integral state
double y_L = 0;                            // Output
double y_hat_L = 0;                        // Estimated output
double u_L = 0;                            // Control input
double r_L = 5.0;                            // Reference input (desired speed)
BLA::Matrix<3, 3, double> A_R = {-231.6, -805.5, -179.6,
                         512, 0, 0,
                         0, 256, 0 };
BLA::Matrix<3, 1, double> B_R = { 4, 0, 0 };
BLA::Matrix<1, 3, double> C_R = {0.8804,-1.331,2.885};
double D_R = 0.004519;
BLA::Matrix<1, 3, double> Kx_R = {17.1000,-188.6782,-42.3636};
double Ki_R = -11.7534;
// State and observer variables
BLA::Matrix<3, 1, double> x_hat_R = { 0, 0, 0 };  // Estimated state
BLA::Matrix<3, 1, double> x_dot_R = { 0, 0, 0 };  // State derivative
float xi_R = 0;                          // Integral state
double y_R = 0;                            // Output
double y_hat_R = 0;                        // Estimated output
double u_R = 0;                            // Control input
double r_R = 5.0;                            // Reference input (desired speeddouble

// Sampling time
const float Ts = 0.005;

StateFeedbackController leftMotorController(A_L, B_L, C_L, D_L, Kx_L, Ki_L, L, Ts, &leftVel_rad, &r_L, &u_L);
StateFeedbackController rightMotorController(A_R, B_R, C_R, D_R, Kx_R, Ki_R, L, Ts, &rightVel_rad, &r_R, &u_R);

const int res = 8;
const int freq = 30000;


// Encoder specifications
const double encoderCPR = 12.0; // Pulses per revolution (PPR) of the encoder
const double gearRatio = 35.0 / 1.0; // Gear ratio (1:32)
// const double L = 163 / 1000; // Distance between wheels in meters (it is in mm)

// Time interval for RPM calculation
unsigned long previousMillis = 0; // Stores the last time the RPM was calculated

long previousLeftPosition = 0; // Stores the previous position of the left encoder
long previousRightPosition = 0; // Stores the previos position of the right encoder

//-------------Filter Variables--------------
const float cutoff_freq = 15.0;
const float sampling_time = 5000 / 1e6;
IIR::ORDER order = IIR::ORDER::OD2;

Filter fl(cutoff_freq, sampling_time, order);
Filter fr(cutoff_freq, sampling_time, order);


// Function to calculate RPM
double calculateRPM(long pulses, double encoderCPR, double gearRatio, double timeIntervalInSeconds) {
  double effectivePPR = 2 * encoderCPR * gearRatio;
  double revolutions = static_cast<double>(pulses) / effectivePPR;
  double rps = revolutions / timeIntervalInSeconds;
  double rpm = (rps * 60);
  return rpm;
}
double calculateRadPerSec(int64_t pulseDifference, double periodSeconds) {
  // Calculate the velocity in radians per second
  double revolutions = pulseDifference / (2 * encoderCPR * gearRatio);
  double velocityRadPerSec = revolutions * (2.0 * PI) / periodSeconds;
  return velocityRadPerSec;
}
//Function for motor speed base on PWM
void motorSpeed(float Lvel, float Rvel){
  // //Set the speed of the motors
  Lvel = constrain(Lvel, -255, 255);
  Rvel = constrain(Rvel, -255, 255);

  // Determine rotation
  if (Lvel > 0){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  }
  else if (Lvel < 0){
      digitalWrite(In1, LOW);
      digitalWrite(In2, HIGH);
  }
  if (Rvel < 0){
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  }
  else if (Rvel > 0){
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
  }
  // (Lvel > 0) ? digitalWrite(In1, HIGH), digitalWrite(In2, LOW) : digitalWrite(In1, LOW), digitalWrite(In2, HIGH);
  // (Rvel > 0) ? digitalWrite(In3, HIGH), digitalWrite(In4, LOW) : digitalWrite(In3, LOW), digitalWrite(In4, HIGH);

  //Set absolute value of the speed
  ledcWrite(MC0, fabs(Lvel));
  ledcWrite(MC1, fabs(Rvel));

}


void setup() {
  Serial.begin(115200);
  //Initialize enconders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  LeftEnc.attachHalfQuad(encLeftA, encLeftB);
  RightEnc.attachHalfQuad(encRightA, encRightB);
  LeftEnc.clearCount();
  RightEnc.clearCount();

  //Initialize motors
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(Enb, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

  ledcSetup(MC0, freq, res);
  ledcSetup(MC1, freq, res);
  ledcAttachPin(EnA, MC0);
  ledcAttachPin(Enb, MC1);
  // left_RPM_value.data = 0.0;

}


void loop() {
  unsigned long currentMillis = micros(); // Get the current time
  if (currentMillis - previousMillis >= 5000){
    float dt = (currentMillis - previousMillis) / 1e6;

    if (dt == 0) {
      dt = 0.001;
    }
    previousMillis = currentMillis;
    // Get the current position of the encoder
    long currentLeftPosition = LeftEnc.getCount();
    long currentRightPosition = RightEnc.getCount();

    // Calculate the number of Leftpulses in the time interval
    long Leftpulses = currentLeftPosition - previousLeftPosition;
    long Rightpulses = currentRightPosition - previousRightPosition;

    previousLeftPosition = currentLeftPosition; // Update the previous position
    previousRightPosition = currentRightPosition; // Update the previous position

    // Calculate rad/s
    leftVel_rad = fl.filterIn(calculateRadPerSec(Leftpulses, dt));
    rightVel_rad = fr.filterIn(calculateRadPerSec(Rightpulses, dt));

    leftMotorController.update();
    rightMotorController.update();

    int dutyL = constrain(u_L, -255, 255);
    int dutyR = constrain(u_R, -255, 255);

    motorSpeed(dutyL, dutyR);
    // Print for debugging
  }
}

