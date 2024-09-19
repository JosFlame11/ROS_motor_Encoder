#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID.h>
// #include <Motors.h>



//Left motor pins
#define EnA 2  // 36
#define In1 13  //35
#define In2 12  //34
// Define pins for the encoder of Left motor
const int encLeftA = 9;  //7
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

// Create PID objects for each motor
// PID objects
PID leftPID(&leftVel_rad, &leftPID_output, &leftSpeed, leftGains.Kp, leftGains.Ki, leftGains.Kd);
PID rightPID(&leftVel_rad, &rightPID_output, &rightSpeed, rightGains.Kp, rightGains.Ki, rightGains.Kd);


const int res = 8;
const int freq = 30000;


// Encoder specifications
const double encoderCPR = 12.0; // Pulses per revolution (PPR) of the encoder
const double gearRatio = 35.0 / 1.0; // Gear ratio (1:32)
const double L = 163 / 1000; // Distance between wheels in meters (it is in mm)

// Time interval for RPM calculation
unsigned long previousMillis = 0; // Stores the last time the RPM was calculated

long previousLeftPosition = 0; // Stores the previous position of the left encoder
long previousRightPosition = 0; // Stores the previos position of the right encoder

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
void motorSpeed(int Lvel, int Rvel){
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
  if (Rvel > 0){
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  }
  else if (Rvel < 0){
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
  }
  // (Lvel > 0) ? digitalWrite(In1, HIGH), digitalWrite(In2, LOW) : digitalWrite(In1, LOW), digitalWrite(In2, HIGH);
  // (Rvel > 0) ? digitalWrite(In3, HIGH), digitalWrite(In4, LOW) : digitalWrite(In3, LOW), digitalWrite(In4, HIGH);

  //Set absolute value of the speed
  ledcWrite(MC0, Lvel);
  ledcWrite(MC1, Rvel);

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
  // Check if the time interval has passed
  if (currentMillis - previousMillis >= 100000) {
    double dt = (currentMillis - previousMillis) / 1e6;
    // Get the current position of the encoder
    long currentLeftPosition = LeftEnc.getCount();
    long currentRightPosition = RightEnc.getCount();
    
    // Calculate the number of Leftpulses in the time interval
    long Leftpulses = currentLeftPosition - previousLeftPosition;
    long Rightpulses = currentRightPosition - previousRightPosition;

    previousLeftPosition = currentLeftPosition; // Update the previous position
    previousRightPosition = currentRightPosition; // Update the previous position
    
    // Calculate RPM
    // leftRPM = calculateRPM(Leftpulses, encoderCPR, gearRatio, dt);
    // rightRPM = calculateRPM(Rightpulses, encoderCPR, gearRatio, dt);

    // Calculate rad/s
    leftVel_rad = calculateRadPerSec(Leftpulses, dt);
    rightVel_rad = calculateRadPerSec(Rightpulses, dt);

    Serial.print(leftVel_rad);
    Serial.print(" ");
    Serial.println(rightVel_rad);
    // leftPID.calculate();
    // rightPID.calculate();

    previousMillis = currentMillis; // Update the time
  }
  // motorSpeed(leftPID_output,rightPID_output);
  motorSpeed(255, 255);
 
}

