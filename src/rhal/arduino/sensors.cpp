// import libraries
#include <Arduino.h>
#include "sensors.h"

// global variables
const float pi = 3.14159;

// functions
// angular velocity from motor encoders
float avelocity(float motor_encoder_srtime_previous, int pulse_count, int pulse_count_old)
{
  // end read time of encoder
  float motor_encoder_ertime = micros(); 
  
  // calculate differences
  float dt = (motor_encoder_ertime - motor_encoder_srtime_previous)/1000000; // [s] dt is always the same value if the loops run at their specified speeds? not always exactly (5 or 6 ms)
  float dp = pulse_count - pulse_count_old;  // cast into float or divisions might result in 0
  
  // pulse2rad (12 magnets in sensor, gear ratio 34:1)
  float ds = dp*2*pi/(465);   // [rad] //ds = dp*2*pi/(12*34);
  
  // calculate speed
  float theta_dot_a = ds/dt;        // [rad/s]
  
  // debug statements
  //Serial.println(j*5);
  //if(dp_l != 0)
  //{
  //  Serial.println(dp_l);
  //}
  //if(dp_r != 0)
  //{
  //  Serial.println(dp_r);
  //}
  //Serial.println(dt_l);
  //Serial.println(dt_r);
  
  return theta_dot_a;
}
