// Author: Ryan Hagler

#include <util/atomic.h>
#include <cmath>
#include "motor_driver.h"

int delta_error = 0;
int state_match = 0;
bool flag = true, turn_flag = false;
int switch_count = 0;

long prevT = 0;
long duration;
int distance;
int prevDistances[2];
int target[NMOTORS];
//bool wallExistence[3]; //Right = 0, Front = 1, Left = 2


#define center_dist 5 //Should be 4
#define max_wall_dev 7
#define min_wall_dev 3
#define wall_pwr_wgt 44
#define no_wall_thrshld 19
#define target_thrshld 160

// PID class instances
SimplePID pid[NMOTORS];

void SimplePID::setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
  kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn, wallCheck = 0;
}

void SimplePID::evalu(int value, int target, float deltaT, int &pwr, int &dir, int motorSide) {
  // checks for walls in front

    int e = target - value;

    float wall_assist;
    if ( ( abs(value) < ( abs(target) - 14 ) ) && ( abs(target) > target_thrshld ) ) {
      int oppSide;
      if (motorSide) //Selecting ultrasonics based on motor perspective
      {
        oppSide = 0;
        motorSide = 2; //Left motor to left ultrasonic
      }
      else
      {
        oppSide = 2;
      }

      int wallDist = getDistance(trig[motorSide], echo[motorSide]);
      int oppWallDist = getDistance(trig[oppSide], echo[oppSide]);

      if ((abs(target) > target_thrshld) && (getDistance(trig[1], echo[1]) <= 6))
      {
        value = target;
        posi[0] = target; // Change encoder value to stop and continue with iterations
        posi[1] = target;
        wallCheck = 0;
      }
      // mouse leaving this side wall, there's a wall on this side, forward move
      // Leaning away from motor side wall
      else if ((oppWallDist < no_wall_thrshld) && (wallDist > center_dist) && (wallDist < no_wall_thrshld))
      {
        wallCheck = 0;
        Serial.print("Activate Lane Keep - BOTH WALL - ");
        if (motorSide == 2)
        {
          Serial.print("Left Motor");
        }
        else
        {
          Serial.print("Right Motor");
        }
        wall_assist = (((wallDist - center_dist) / (max_wall_dev - center_dist)) * wall_pwr_wgt * 1); //Decrease motor power
        Serial.print(" DECREASE POWER by: ");
        Serial.println(wall_assist);
      }
      // opposite side has no wall, mouse is heading towards this side wall
      // Leaning towards motor side wall
      else if ((oppWallDist > no_wall_thrshld) && (wallDist < center_dist))
      {
        wallCheck = 0;
        Serial.print("Activate Lane Keep ");
        if (motorSide == 2)
        {
          Serial.print("- LEFT WALL - Left Motor ");
        }
        else
        {
          Serial.print("- RIGHT WALL - Right Motor ");
        }
        wall_assist = (((wallDist - center_dist) / (center_dist - min_wall_dev)) * wall_pwr_wgt * 0.6); //Increase motor power
        Serial.print(" INCREASE POWER by: ");
        Serial.println(wall_assist);
      }
      // opposite side has no wall, mouse leacing this side wall, there's a wall on this side, forward move
      // Leaning away motor side wall
      else if ((oppWallDist > no_wall_thrshld) && (wallDist > center_dist) && (wallDist < no_wall_thrshld))
      {
        wallCheck = 0;
        wall_assist = (((wallDist - center_dist) / (center_dist - min_wall_dev)) * wall_pwr_wgt * 0.6); //Decreases motor power
        Serial.print(" DECREASE POWER by: ");
        Serial.println(wall_assist);
      }
      //Both walls, leaning towards motor side wall [second conditional implies a wall exists]
      else if ((oppWallDist > no_wall_thrshld) && (wallDist > no_wall_thrshld))
      {
        if (wallCheck > 5)
          wall_assist = ((( ( wallDist % 4 ) - center_dist) / (max_wall_dev - center_dist)) * wall_pwr_wgt * .05);
        wallCheck ++;
      }
      else
        wall_assist = 0;
    }
    else {
      wall_assist = 0;
    }
    // derivative
    float dedt = (e - eprev) / (deltaT);
    float dwdt = (wall_assist - wprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // motor power
    pwr = (int)fabs(u);
    if (pwr > umax)
    {
      pwr = umax;
    }

    else if (pwr > 0 && pwr <= 90) {
      pwr = 90;
    }
    Serial.print("Wall Delta: ");
    Serial.println(dwdt);
    pwr -= (wall_assist) ;

    // motor direction
    dir = 1;
    if (u < 0) {
      dir = -1;
    }

    // store previous error
    eprev = e;
    wprev = wall_assist;
  //}
}

int getDistance(int trigPin, int echoPin) {
  // sending the sound wave
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // capturing the sound wave
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 29 / 2);
  //Serial.println(distance);
  return distance;
}

bool fwdSensor() {
  bool noWall = false;
  for (int i = 0; i < 3; i++)
  {
    if ( !(getDistance(trig[1], echo[1]) < 12) )
      noWall = true;
  }
  return noWall;
}

bool rightSensor() {
  bool noWall = false;
  for (int i = 0; i < 3; i++)
  {
    if ( !( getDistance(trig[0], echo[0]) < 12) )
      noWall = true;
  }
    return noWall;
}

bool leftSensor() {
  bool noWall = false;
  for (int i = 0; i < 3; i++)
  {
    if ( !(getDistance(trig[2], echo[2]) < 12) )
      noWall = true;
  }
  return noWall;
}

void fwd() {


  //Take intial readings of ultrasonic, PID will update at 1/4 target intervals


  if (turn_flag)
    backup();
  // 19 cm PID
   if (target[0] == 146) {
     target[0] = -188;
     target[1] = 190;
   }
   else {
     target[0] = -200;
     target[1] = 202;
   }
  flag = true;
  iter();
}

void backup() {
  if (target[0] == 146){
    target[0] = 27;
    target[1] = -27;
  }
  else{
  target[0] = 55;
  target[1] = -55;
  }
  flag = false;
  turn_flag = false;
  iter();
}

void turnRight() {
  target[0] = 00;
  target[1] = 146;
  flag = false;
  turn_flag = true;
  iter();
}

void turnLeft() {
  target[0] = -146;
  target[1] = 00;
  flag = false;
  turn_flag = true;
  iter();
}

void rotate() {
  if (getDistance(trig[0], echo[0]) >= getDistance(trig[2], echo[2]) ) {
    target[0] = 146;
    target[1] = 146;
  }
  else {
    target[0] = -146;
    target[1] = -146;
  }
  flag = false;
  turn_flag = true;
  iter();
}

void iter() {
  while (1) {
    if ( switch_count = 500 ) {
      switch_count = 0;
    }
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      for (int k = 0; k < NMOTORS; k++) {
        pos[k] = posi[k];
      }
    }

    for (int k = 0; k < NMOTORS; k++) {
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
    Serial.println();

    if (state_match == 1) {
      if (target[1] > 0) {
        Serial.println("State Match: ");
        Serial.println(delta_error);
        Serial.println(posi[0]);
        Serial.println(posi[1]);
        posi[1] += delta_error;
        Serial.println("After adding error: ");
        Serial.println(posi[1]);

      }
      else {
        posi[1] -= delta_error;
        Serial.println(posi[1]);
      }
      delta_error = 0;
      state_match = 0;
    }
    else if (state_match == 2) {
      if (target[0] > 0) {
        Serial.println("State Match: ");
        Serial.println(delta_error);
        Serial.println(posi[0]);
        Serial.println(posi[1]);
        posi[0] += delta_error;
        Serial.println("After adding error: ");
        Serial.println(posi[0]);
      }
      else {
        posi[0] -= delta_error;
        Serial.println(posi[0]);
      }
      delta_error = 0;
      state_match = 0;
    }

    // error correction
    if (flag && ((target[0] == pos[0]) || (target[1] == pos[1]))) {
      if (target[0] == pos[0] && target[1] != pos[1]) {
        if (target[1] > 0 && pos[1] > 0) {
          delta_error = -1 * (target[1] - pos[1]);
          Serial.println("Pos Error recorded in 1:");
          Serial.println(delta_error);
          Serial.println(target[1]);
          Serial.println(pos[1]);
        }
        else if (target[1] < 0 && pos[1] < 0) {
          delta_error = target[1] - pos[1];
          Serial.println("Neg Error recorded in 1:");
          Serial.println(delta_error);
          Serial.println(target[1]);
          Serial.println(pos[1]);
        }
        else {
          Serial.println("Discrepancy in signs!");
        }
        state_match = 1;
      }
      else if (target[1] == pos[1] && target[0] != pos[0]) {
        if (target[0] > 0 && pos[0] > 0) {
          delta_error = -1 * (target[0] - pos[0]);
          Serial.println("Pos Error recorded in 0:");
          Serial.println(delta_error);
          Serial.println(target[0]);
          Serial.println(pos[0]);
        }
        else if (target[0] < 0 && pos[0] < 0) {
          delta_error = target[0] - pos[0];
          Serial.println("Neg Error recorded in 0:");
          Serial.println(delta_error);
          Serial.println(target[0]);
          Serial.println(pos[0]);
        }
        else {
          Serial.println("Discrepancy in signs!");
        }

        state_match = 2;
      }
      else {
        Serial.println("Perfect match!");
      }
      killMotor();
      delay(100);
      posi[0] = 0;
      posi[1] = 0;
      return;
    }


    if (!flag && ((target[0] == pos[0]) && (target[1] == pos[1]))) {
      killMotor();
      delay(100);
      posi[0] = 0;
      posi[1] = 0;
      return;
    }

    int dir_0 = 0, pwr_0 = 0, dir_1 = 0, pwr_1 = 0;
    int k_0 = 0, k_1 = 0;
    if (switch_count % 2 ) {
      k_1 = 1;
    }
    else {
      k_0 = 1;
    }

    pid[k_0].evalu(pos[k_0], target[k_0], deltaT, pwr_0, dir_0, k_0); // Evaluating power for right motor
    pid[k_1].evalu(pos[k_1], target[k_1], deltaT, pwr_1, dir_1, k_1);   // Evaluating for left motor

    Serial.print("Left Motor Power: ");
    Serial.print(pwr_0);
    Serial.print(" Right Motor Power: ");
    Serial.println(pwr_1);

    //Activate motors
    setMotor(dir_0, pwr_0, pwm[k_0], in1[k_0], in2[k_0]);
    setMotor(dir_1, pwr_1, pwm[k_1], in1[k_1], in2[k_1]);

    switch_count ++;
  }

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == -1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void killMotor() {
  digitalWrite(in1[0], LOW);
  digitalWrite(in1[1], LOW);
  digitalWrite(in2[0], LOW);
  digitalWrite(in2[1], LOW);
}
