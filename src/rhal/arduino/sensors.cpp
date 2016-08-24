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
  float motor_encoder_ertime = millis(); 
  
  // calculate differences
  float dt = (motor_encoder_ertime - motor_encoder_srtime_previous)/1000; //[s]
  float dp = pulse_count - pulse_count_old;
  
  // pulse2rad (12 magnets in sensor, gear ratio 34:1)
  float ds = dp*2*pi/(12*34);   // [rad]
  
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
