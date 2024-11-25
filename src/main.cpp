#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID.h>
// #include <Motors.h>
#include <filters.h>
#include <BasicLinearAlgebra.h>




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
// double K_i = -82.9183;    // Integral gain
// double L[3] = {1178.1,200.8,-404.8}; // Observer gain

using namespace BLA;

BLA::Matrix<3, 3> A = { -204.9, -798.2, -171.8,
                         512, 0, 0,
                         0, 256, 0 };
BLA::Matrix<3, 1> B = { 4, 0, 0 };
BLA::Matrix<1, 3> C = { 0.9374, -0.3582, 2.908 };
float D = -0.0006;
BLA::Matrix<1, 3> K_x = {92.7750, -155.3811, -26.7866};
float K_i = -74.6265;
BLA::Matrix<3, 1> L = {1.581, 0.2632, -0.4043};
// State and observer variables
BLA::Matrix<3, 1> x_hat = { 0, 0, 0 };  // Estimated state
BLA::Matrix<3, 1> x_dot = { 0, 0, 0 };  // State derivative
float x_i = 0;                          // Integral state
float y = 0;                            // Output
float y_hat = 0;                        // Estimated output
float u = 0;                            // Control input
float r = 15.0;                            // Reference input (desired speed)

// Sampling time
const float Ts = 0.005; // 1ms (adjust based on system needs)


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
    leftVel_rad = calculateRadPerSec(Leftpulses, dt);
    rightVel_rad = calculateRadPerSec(Rightpulses, dt);

    y = static_cast<float>(leftVel_rad);

    float error = r - y;

    // Update integral state
    x_i += error * Ts;

    // Compute estimated output
    y_hat = (C * x_hat)(0, 0) + D * u;

    // Observer update: dx_hat = A * x_hat + B * u + L * (y - y_hat)
    BLA::Matrix<3, 1> observer_correction = L * (y - y_hat);
    x_dot = A * x_hat + B * u + observer_correction;

    // Update estimated states using Euler integration
    x_hat = x_hat + x_dot * Ts;

    // Compute control input: u = -K_x * x_hat - K_i * x_i
    u = -(K_x * x_hat)(0, 0) - K_i * x_i;

    // Simulate output (if needed for testing): y = C * x + D * u
    // y = (C * x_hat)(0, 0) + D * u;

    int duty = constrain(u, -255, 255);

    // Print for debugging
    Serial.print("Control Input (u): "); Serial.print(u);
    Serial.print(" Integral State (x_i): "); Serial.print(x_i);
    Serial.print(" Output (y): "); Serial.println(y);
    motorSpeed(duty, 0);
  }
}

