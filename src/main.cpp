#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID.h>
#include <Motors.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

///////ROS node///////
double Rx = 0;
double Rz = 0;

ros::NodeHandle nh;

//Left motor pins
#define EnA 36
#define In1 35
#define In2 34
// Define pins for the encoder of Left motor
const int encLeftA = 7;
const int encLeftB = 6;
// Motor pwmChannel
const int MC0 = 0;

//Rigth motor pins
#define In3 33
#define In4 47
#define Enb 48
// Define pins for the encoder of Right motor
const int encRightA = 5;
const int encRightB = 4;
// Motor pwmChannel
const int MC1 = 1;

// Create motor objects
Motor leftMotor(EnA, In1, In2, MC0);
Motor rightMotor(Enb, In3, In4, MC1);

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

PIDgains leftGains(7.0, 5.0, 0.0);
PIDgains rightGains(7.0, 5.0, 0.0);

// Pulses 
double leftRPM;
double rightRPM;

// Desire velocity
double leftPID_output;
double rightPID_output;

// Create PID objects for each motor
// PID objects
PID leftPID(&leftRPM, &leftPID_output, 0, leftGains.Kp, leftGains.Ki, leftGains.Kd);
PID rightPID(&rightRPM, &rightPID_output, 0, rightGains.Kp, rightGains.Ki, rightGains.Kd);


// const int res = 8;
// const int freq = 1000;

int16_t sub_Lvel = 0;
int16_t sub_Rvel = 0;

// Encoder specifications
const double encoderPPR = 341.2; // Pulses per revolution (PPR) of the encoder
const double gearRatio = 32.0 / 1.0; // Gear ratio (1:32)

// Time interval for RPM calculation
unsigned long previousMillis = 0; // Stores the last time the RPM was calculated

long previousLeftPosition = 0; // Stores the previous position of the left encoder
long previousRightPosition = 0; // Stores the previos position of the right encoder

// Function to calculate RPM
double calculateRPM(long pulses, double encoderPPR, double gearRatio, double timeIntervalInSeconds) {
  double effectivePPR = 4.0 * encoderPPR;
  double revolutions = static_cast<double>(pulses) / effectivePPR;
  double rps = revolutions / timeIntervalInSeconds;
  double rpm = (rps * 60);
  return rpm;
}
//Function for motor speed base on PWM
void motorSpeed(int Lvel, int Rvel){
  // //Set the speed of the motors
  // Lvel = constrain(Lvel, -255, 255);
  // Rvel = constrain(Rvel, -255, 255);

  //Determine rotation
  // if (Lvel > 0){
  //   digitalWrite(In1, HIGH);
  //   digitalWrite(In2, LOW);
  // }
  // else if (Lvel < 0){
  //     digitalWrite(In1, LOW);
  //     digitalWrite(In2, HIGH);
  // }
  // if (Rvel > 0){
  //   digitalWrite(In3, HIGH);
  //   digitalWrite(In4, LOW);
  // }
  // else if (Rvel < 0){
  //   digitalWrite(In3, LOW);
  //   digitalWrite(In4, HIGH);
  // }
  // // (Lvel > 0) ? digitalWrite(In1, HIGH), digitalWrite(In2, LOW) : digitalWrite(In1, LOW), digitalWrite(In2, HIGH);
  // // (Rvel > 0) ? digitalWrite(In3, HIGH), digitalWrite(In4, LOW) : digitalWrite(In3, LOW), digitalWrite(In4, HIGH);

  // //Set absolute value of the speed
  // ledcWrite(MC0, Lvel);
  // ledcWrite(MC1, Rvel);

  leftMotor.setMotorSpeed(Lvel);
  rightMotor.setMotorSpeed(Rvel);
}
//callbackFunction for the teleop_key
void messageCb(const geometry_msgs::Twist& msg){
  Rx = msg.linear.x;
  Rz = msg.angular.z;

  if (Rx >= 1.0 && Rz == 0) sub_Lvel = 255, sub_Rvel = 255; //avanzar (fix)
  else if (Rx <= 0 && Rz == 0) sub_Lvel = -100, sub_Rvel = -100; //Retroseder (fix)
  else if (Rz >= 1.0 && Rx == 0) sub_Lvel = 200, sub_Rvel = -200; //Derecha
  else if (Rz <= 0 && Rx == 0) sub_Lvel = -200, sub_Rvel = 200; //Izq
}
//Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", messageCb);

//Publisher
std_msgs::Float32 left_RPM_value;
std_msgs::Float32 right_RPM_value;
ros::Publisher RPM_left_val("RPM_left_val", &left_RPM_value);
ros::Publisher RPM_right_val("RPM_right_val", &right_RPM_value);
// char MSG[12] = "RPM value: ";


void setup() {
  //Initialize Ros node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(RPM_left_val);
  nh.advertise(RPM_right_val);

  //Initialize enconders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  LeftEnc.attachFullQuad(encLeftA, encLeftB);
  RightEnc.attachFullQuad(encRightA, encRightB);
  LeftEnc.clearCount();
  RightEnc.clearCount();

  //Initialize motors
  // pinMode(EnA, OUTPUT);
  // pinMode(In1, OUTPUT);
  // pinMode(In2, OUTPUT);
  // pinMode(Enb, OUTPUT);
  // pinMode(In3, OUTPUT);
  // pinMode(In4, OUTPUT);

  // ledcSetup(MC0, freq, res);
  // ledcSetup(MC1, freq, res);
  // ledcAttachPin(EnA, MC0);
  // ledcAttachPin(Enb, MC1);
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
    double leftRPM = calculateRPM(Leftpulses, encoderPPR, gearRatio, dt);
    double rightRPM = calculateRPM(Rightpulses, encoderPPR, gearRatio, dt);

    leftPID.setSetpoint(sub_Lvel);
    rightPID.setSetpoint(sub_Rvel);

    leftPID.calculate();
    rightPID.calculate();
    left_RPM_value.data = static_cast<float>(leftRPM);
    RPM_left_val.publish(&left_RPM_value);
    nh.spinOnce();

    previousMillis = currentMillis; // Update the time
  }
  motorSpeed(sub_Lvel, sub_Rvel);
  // if (currentMillis - previousMillis >= 1000000){
  // }
}

