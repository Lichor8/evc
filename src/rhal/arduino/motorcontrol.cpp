// import libraries
#include <Arduino.h>
#include "motorcontrol.h"


// functions
void pcontrol(float phi_d, float phi_a, float &omega, float &v, float &e_old, float &E)
{  
  const int Kp = 1;
  const int Ki = 1;
  const int Kd = 1; 
  
  float e = phi_d - phi_a;
  float e_dot = e - e_old;
  E = E + e;
  //omega = Kp*e + Ki*E + Kd*e_dot;
  omega = Kp*e;
  e_old = e;
  
  v = 0.0;            // base speed [m/s]
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


