// import functions
#include "communication.h"
//#include "motorcontrol.h"
#include "movement.h"

// global variables
#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

const int timeout = 1000/50;
const float pi = 3.14159;

// the setup function runs once when you press reset or power the board
void setup() {
  analogWrite(PWM_R, 0); 
  analogWrite(PWM_L, 0);
  digitalWrite(EN_L_FWD, LOW);
  digitalWrite(EN_L_BWD, LOW);
  digitalWrite(EN_R_FWD, LOW);
  digitalWrite(EN_R_BWD, LOW);

  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(EN_L_FWD, OUTPUT);
  pinMode(EN_L_BWD, OUTPUT);
  pinMode(EN_R_FWD, OUTPUT);
  pinMode(EN_R_BWD, OUTPUT);
  
  Serial.begin(9600);
  Serial.setTimeout(timeout);
}

void loop() 
{ 
// initialize variables
    
  // recieve rpi information
  int mov_type = -1;
  float x_d = 0.0;
  float y_d = 1.0;
  float turn_deg = 0;
  float stop_sec = 0;  
  
  // position(outer) control loop (pcontrol_freq)
  while(1)
  {    
    // receive rpi information   
    rpi2arduino(mov_type, x_d, y_d, turn_deg, stop_sec); 
    //Serial.println(x_d);
    //Serial.println(y_d);

    // act according to selected movement type
    // if movement type is drive between lines (mov_type = 0) 
    // if movement type is turn (mov_type = 4)
    // if movement type is stop (mov_type = 5) 
    movement(mov_type, x_d, y_d, turn_deg, stop_sec);    
  } 
}




