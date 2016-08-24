// import libraries
#include <Arduino.h>
#include "motorcontrol.h"


// functions
void pcontrol(float phi_d, float phi_a, float &omega, float &v, float &e_old, float &E)
{  
  // PID controller for omega (rotation speed)
  const int Kp = 1;
  const int Ki = 1;
  const int Kd = 1; 
  
  float e = phi_d - phi_a;
  float e_dot = e - e_old;
  E = E + e;
  e_old = e;
  
  //omega = Kp*e + Ki*E + Kd*e_dot;
  omega = Kp*e;    
  v = 0.0;            // base speed [m/s]
}

float scontrol(float theta_dot_d, float theta_dot_a, float &e_old, float &E)
{
  // state space controller     
//  const int K[2] = {0.3101, -0.0688 };
//  const int F = 0;
//  float r = theta_dot_d;
//  float x1 = theta_dot_a;  
//  float x2 = dc_current_a;
//  float c = K[0]*x1 + K[1]*x2 + F*r
          
  // PID controller for c (duty cycle)
  const int Kp = 1;
  const int Ki = 1;
  const int Kd = 1;
 
  float e = theta_dot_d - theta_dot_a;
  float e_dot = e - e_old;
  E = E + e;
  e_old = e;
  
  float c = Kp*e + Kd*e_dot;  // Kp*e + Ki*E + Kd*e_dot;
  return c;
}


void setMotor(const unsigned char cucPWM, const unsigned char cucFWD , const unsigned char cucBWD, const int ciSpeed)
{
  if (ciSpeed < 0)
  {
    digitalWrite(cucFWD, LOW);
    digitalWrite(cucBWD, LOW); 
    digitalWrite(cucFWD, LOW);
    digitalWrite(cucBWD, HIGH); 
  }
  else
  {
    digitalWrite(cucFWD, LOW);
    digitalWrite(cucBWD, LOW); 
    digitalWrite(cucFWD, HIGH);
    digitalWrite(cucBWD, LOW); 
  }

  analogWrite(cucPWM, abs(ciSpeed));  
}


