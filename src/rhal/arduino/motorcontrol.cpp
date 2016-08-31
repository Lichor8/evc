// import libraries
#include <Arduino.h>
#include "motorcontrol.h"


// functions
void pcontrol(float phi_d, float phi_a, float &omega, float &v, float &e_old, float &E)
{  
  // PID controller for omega (rotation speed)
  const float Kp = 0.1;
  const float Ki = 0.0;
  const float Kd = 0.0; 
  
  float e = phi_d - phi_a;
  float e_dot = e - e_old;
  E = E + e;
  e_old = e;
  
  omega = Kp*e + Ki*E + Kd*e_dot;    
  v = 0.25;            // base speed [m/s]
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
  const float Kp = 0.1;
  const float Ki = 0.01;
  const float Kd = 0.02;
 
  float e = theta_dot_d - theta_dot_a;
  float e_dot = e - e_old;
  E = E + e;
  e_old = e;
  
  float c = Kp*e + Ki*E + Kd*e_dot;
  if ( c > 1)
  {
    c = 1;
  }
  else if (c < -1)
  {
    c = -1;
  }
  return c;
}

//float scontrol_l(float theta_dot_d, float theta_dot_a, float &e_old_l, float &E_l)
//{          
//  // PID controller for c (duty cycle)
//  const float Kp = 1.0;
//  const float Ki = 0.0;
//  const float Kd = 0.0;
// 
//  float e = theta_dot_d - theta_dot_a;
//  float e_dot_l = e - e_old_l;
//  E_l = E_l + e;
//  e_old_l = e;
//  
//  float c = Kp*e + Ki*E_l + Kd*e_dot_l;
//  if ( c > 1)
//  {
//    c = 1;
//  }
//  else if (c < -1)
//  {
//    c = -1;
//  }
//  return c;
//}
//
//float scontrol_r(float theta_dot_d, float theta_dot_a, float &e_old_r, float &E_r)
//{          
//  // PID controller for c (duty cycle)
//  const float Kp = 1.0;
//  const float Ki = 0.0;
//  const float Kd = 0.0;
// 
//  float e = theta_dot_d - theta_dot_a;
//  float e_dot_r = e - e_old_r;
//  E_r = E_r + e;
//  e_old_r = e;
//  
//  float c = Kp*e + Ki*E_r + Kd*e_dot_r;
//  if ( c > 1)
//  {
//    c = 1;
//  }
//  else if (c < -1)
//  {
//    c = -1;
//  }
//  return c;
//}


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



//        // PID controller for c (duty cycle)
//        const float Kp = 2.0;
//        const float Ki = 0.0;
//        const float Kd = 0.0;
//       
//        float e = theta_dot_d_l - theta_dot_a_l;    
//        float e_dot_l = e - e_old_l;
//        E_l = E_l + e;
//        e_old_l = e;    
//        float c_l = Kp*e + Ki*E_l + Kd*e_dot_l;
//        
//        e = theta_dot_d_r - theta_dot_a_r;\
//        float e_dot_r = e - e_old_r;
//        E_r = E_r + e;
//        e_old_r = e;        
//        float c_r = Kp*e + Ki*E_r + Kd*e_dot_r;

