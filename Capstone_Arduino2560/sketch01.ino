// Author: Ryan Hagler

#include "route_planner.h"
#include "motor_driver.h"

#define bounceDelay 20
#define minButtonPress 3

volatile int posi[] = {0,0};
const int VEC_DIM = 11;

float pulsesPerTurn = 3 * 52;
float wheelCirc = 0.013823;
float targetDist = 0.7747;
float pulsesPerMeter = pulsesPerTurn * 72.3431559508;

const int buttonPin = 20; // the number of the pushbutton pin
int masterCount = 1;


void setup(){
  //Serial.begin(9600);
  extern SimplePID pid[NMOTORS];
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

    //pid[k].setParams(1,0.01,.65,225);
    pid[k].setParams(1,0.05,0,120);
  }

  for (int i = 0; i < NULTRA; i++){
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  } 

  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(buttonPin), read_button, FALLING);


  // virtual map setup
  std::vector<std::vector<State>> grid(VEC_DIM, std::vector<State>(VEC_DIM, State::kUndefined));

  if (grid.empty())
    Serial.println("Failed to construct virtual map");
  
  std::vector<int> start{0,0}, finish{10,10};
  while ( masterCount == 1){
    delay(150);
  }
 // Serial.println("Starting");
 RoutePlanner route_planner(start, finish);
 route_planner.AStarSearch(grid);
  //rotate();
}

void loop(void){
  }

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }  
}

void read_button(){
  while(digitalRead(buttonPin)) //Debounce
  {
  delay(150);
  }
  if(masterCount == 0)
  {
    killMotor();
    while(1)
    {
      //do nothing
    }
  }
  masterCount = !masterCount;
}
