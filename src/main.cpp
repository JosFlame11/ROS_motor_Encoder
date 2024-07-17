#include <Arduino.h>
#include <ESP32Encoder.h>
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

//Rigth motor pins
#define In3 33
#define In4 47
#define Enb 48

//Motor Setup PWM
const int MC0 = 0;
const int MC1 = 1;

const int res = 8;
const int freq = 1000;



// Define pins for the encoder of Left motor
const int encLeftA = 7;
const int encLeftB = 6;

// Define pins for the encoder of Right motor
const int encRightA = 5;
const int encRightB = 4;

// Create an Encoder object
ESP32Encoder LeftEnc;
ESP32Encoder RightEnc;

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
  //Set the speed of the motors
  Lvel = constrain(Lvel, -255, 255);
  Rvel = constrain(Rvel, -255, 255);

  //Determine rotation
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
//callbackFunction for the teleop_key
void messageCb(const geometry_msgs::Twist& msg){
  Rx = msg.linear.x;
  Rz = msg.linear.z;

  if (Rx >= 2.0) motorSpeed(255, 255);
  if (Rx <= 0) motorSpeed(0, 0);
  if (Rz >= 2.0) motorSpeed(100, -100);
  if (Rz <= 0) motorSpeed(-100, 100);
}
//Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", messageCb);

//Publisher
std_msgs::Float32 rpm_value_float32;
ros::Publisher RPM_val("RPM_val", &rpm_value_float32);
char MSG[12] = "RPM value: ";


void setup() {
  //Initialize Ros node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(RPM_val);

  //Initialize enconders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  LeftEnc.attachFullQuad(encLeftA, encLeftB);
  RightEnc.attachFullQuad(encRightA, encRightB);
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
  rpm_value_float32.data = 0.0;
}

void loop() {

  unsigned long currentMillis = micros(); // Get the current time
  motorSpeed(250, 250);
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

  rpm_value_float32.data = static_cast<float>(leftRPM);

  }
  if (currentMillis - previousMillis >= 500000){
    RPM_val.publish(&rpm_value_float32);
    nh.spinOnce();
  }
}

