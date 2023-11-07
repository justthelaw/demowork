// Author: Ryan Hagler

// How many motors
const int NMOTORS = 2;
// How many ultrasonics
const int NULTRA = 3;

// Pins
const int enca[] = {19,18};
const int encb[] = {23,27};
const int pwm[] = {2,3};
const int in1[] = {25,29};
const int in2[] = {22,26};
const int stopButton = 21;
const int trig[] = {4, 5, 6};
const int echo[] = {30, 31, 32};

#include "route_planner.h"

// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral, wprev; // Storage
    int wallCheck; 
  public:
  // Constructor w/ initializer list
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), wallCheck(0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir, int motorSide);
};

extern volatile int posi[];

int getDistance(int trigPin, int echoPin);

bool fwdSensor();
bool rightSensor();
bool leftSensor();

void fwd();
void backup();
void turnRight();
void turnLeft();
void rotate();

void iter();

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);

void killMotor();
