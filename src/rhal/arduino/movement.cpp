// act according to selected movement type
// if movement type is drive between lines (mov_type = 0) 
// if movement type is turn left (mov_type = 1)
// if movement type is drive (mov_type = 2) then drive the recieved amount of meters
// if movement type is turn right (mov_type = 3)
// if movement type is turn (mov_type = 4) then turn the recieved amount of degrees
// if movement type is stop (mov_type = 5)

// import libraries
#include <Arduino.h>
#include "movement.h"
#include "motorcontrol.h"
#include "sensors.h"

// global variables
#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

const float pi = 3.14159;

// initialize variables
  // recieve rpi information
  int mov_type = -1;
  float x_d = 0.0;
  float y_d = 1.0;
  float turn_deg = 0;
  float stop_sec = 0; 
  
  // location2angle (robot frame: x=right, y=upwards)
  float phi_a = pi/2; // using coordinate frame on the robot (relative) so actual is always pi/2
  float x_a   = 0.0;  // using coordinate frame on the robot (relative) so actual is always 0
  float y_a   = 0.0;  // using coordinate frame on the robot (relative) so actual is always 0
  float phi_d = 0.0;  // initialize
  
  // position control loop
  float e_old_p = 0.0;
  float E_p = 0.0;
  float omega = 0.0;
  float v     = 0.0;
  float stop_timer = 0.0;
  int pulse_count_start = 0;
  int execute_start = 1;
  
  // encoder loop  
  int motor_encoder_l_old = 0;
  int motor_encoder_r_old = 0;
  int pulse_count_l = 0;
  int pulse_count_l_old = 0;
  int pulse_count_r = 0;
  int pulse_count_r_old = 0;
  
  // velocity control loop
  float e_old_l = 0.0;
  float e_old_r = 0.0;
  float E_l = 0.0;  
  float E_r = 0.0;
  float motor_encoder_srtime_l = 0.0;
  float motor_encoder_srtime_r = 0.0;
  float motor_encoder_srtime_l_previous = 0.0;
  float motor_encoder_srtime_r_previous = 0.0;
  //float theta_dot_a_l = 0.0;
  //float theta_dot_a_r = 0.0;
  //float c_l = 0.0;
  //float c_r = 0.0;
  
  // set inner/outer loop frequencies
  const float pcontrol_freq = 1.0;               // position controol loop frequency [1/s]
  const float scontrol_freq = 200.0;             // speed control loop frequency     [1/s]
  const float sensor_freq   = 10000.0;            // sensor sample loop frequency     [1/s]
  const float sensor_time   = 1000/sensor_freq;  // sensor sample time               [ms]
  
  // set debug frequency
  const float debug_freq = 10;  // [1/s]
  
// functions
void movement(int mov_type, float x_d, float y_d, float turn_r, float drive_m, float turn_deg, float stop_sec)
{  
  // inverse kinematics
  const float R = 0.07;         // [m]
  const float L = 0.45;         // [m]
  float theta_dot_d_l = 0.0;  
  float theta_dot_d_r = 0.0;    
  
  // act according to selected movement type
  // if movement type is drive between lines (mov_type = 0) 
  if (mov_type == 0)
  {      
    Serial.println(x_d);
    Serial.println(y_d);
    // location2angle (use when actual location is available)
    //phi_a = pi;   
    //x_a   = 0.0;    
    //y_a   = 0.0;    
    phi_d = atan2(y_d - y_a, x_d - x_a);
      
    // position(outer) control loop (pcontrol_freq)
    pcontrol(phi_d, phi_a, omega, v, e_old_p, E_p);  
    //Serial.println(omega);
    
    // inverse kinematics
    theta_dot_d_r = (2*v + omega*L)/(2*R);  // [rad/s]
    theta_dot_d_l = (2*v - omega*L)/(2*R);  // [rad/s]
    //Serial.println(theta_dot_d_r);
    
    Serial.println("execute0");
  }    
  // if movement type is turn left (mov_type = 1)
  if (mov_type == 1)
  {    
    Serial.println(turn_r);
    if (execute_start = 1)
    {
      pulse_count_start = pulse_count_r;
      execute_start = 0;      
    }
    omega = pi/4;               // turning speed [rad/s] 
    float r1 = turn_r;          // inner turn radius [m]
    float r2 = r1 + L;          // outer turn radius [m]
    theta_dot_d_r = omega*r2;   // [rad/s]
    theta_dot_d_l = omega*r1;   // [rad/s]   
    
    // pulse2meters (12 magnets in sensor, gear ratio 34:1)
    float dp = pulse_count_r - pulse_count_start;
    float ds = R*dp*2*pi/(12*34);   // [m]
    
    // if traveled distance is half a turn
    if (ds >= pi/2)
    {
      Serial.println("done1");
      execute_start = 1;
    }
    else
    {
      Serial.println("execute1");
    }
  }
  // if movement type is drive (mov_type = 2)
  if (mov_type == 2)
  {
    Serial.println(drive_m);
    if (execute_start = 1)
    {
      pulse_count_start = pulse_count_l;
      execute_start = 0;      
    }
    theta_dot_d_r = v;  // [rad/s]
    theta_dot_d_l = v;  // [rad/s]
    
    // pulse2meters (12 magnets in sensor, gear ratio 34:1)
    float dp = pulse_count_l - pulse_count_start;
    float ds = R*dp*2*pi/(12*34);   // [m]
    
    // if traveled distance is drive_m then ask for next command
    if (ds >=  drive_m)
    {
      Serial.println("done2");
      execute_start = 1;
    }
    else
    {
      Serial.println("execute2");
    }
  }
  // if movement type is turn right (mov_type = 3)
  if (mov_type == 3)
  {
    Serial.println(turn_r);
    if (execute_start = 1)
    {
      pulse_count_start = pulse_count_l;
      execute_start = 0;      
    }
    omega = pi/4;               // turning speed [rad/s]
    float r1 = turn_r;          // inner turn radius [m]
    float r2 = r1 + L;          // outer turn radius [m]
    theta_dot_d_r = omega*r1;  // [rad/s]
    theta_dot_d_l = omega*r2;  // [rad/s]
    
    // pulse2meters (12 magnets in sensor, gear ratio 34:1)
    float dp = pulse_count_l - pulse_count_start;
    float ds = R*dp*2*pi/(12*34);   // [m]
    
    // if traveled distance is half a turn then ask for next command
    if (ds >= pi/2)
    {
      Serial.println("done3");
      execute_start = 1;
    }
    else
    {
      Serial.println("execute3");
    }
  }    
  // if movement type is turn (mov_type = 4)
  if (mov_type == 4)
  {
    Serial.println(turn_deg);
    if (execute_start = 1)
    {
      pulse_count_start = pulse_count_l;
      execute_start = 0;      
    }
    theta_dot_d_r = pi/2;              // [rad/s]
    theta_dot_d_l = -theta_dot_d_r;    // [rad/s]
    
    // pulse2meters (12 magnets in sensor, gear ratio 34:1)
    float dp = pulse_count_l - pulse_count_start;
    float ds = R*dp*2*pi/(12*34);               // [m]
    float turn = L*turn_deg*pi/180;             // [m]
    
    // if traveled distance is turn_deg then ask for next command
    if (ds >= turn)
    {
      Serial.println("done4");
      execute_start = 1;
    }
    else
    {
      Serial.println("execute4");
    }
  }    
  // if movement type is stop (mov_type = 5)
  if (mov_type == 5)
  {   
    Serial.println(stop_sec);
    if (execute_start = 1)
    {
      // start stop timer
      stop_timer = millis()/1000;  // [s]
      execute_start = 0;      
    }
    theta_dot_d_r = 0;    // [rad/s]
    theta_dot_d_l = 0;    // [rad/s]    
    
    // wait stop_sec amount of seconds then ask for next command
   if (stop_timer >= stop_sec)
   {
     Serial.println("done5");
   }
   else
    {
      Serial.println("execute5");
    }
  } 
  
  // sensor sample loop (sensor_freq)
  int i = 0;
  int j = 1;
  int k = 1;
  while(i < sensor_freq/pcontrol_freq)
  {
    float start_sensor_timer = millis();
  
    // if i = 0, 5, 10, 15, ... then set start time
    if(i == sensor_freq/(pcontrol_freq*scontrol_freq)*j)
    {
      //Serial.println(i);
      motor_encoder_srtime_l = millis();    
      motor_encoder_srtime_r = millis();
    }   
    
    int motor_encoder_l = digitalRead(2);
    int motor_encoder_r = digitalRead(3);
    
    if(motor_encoder_l == 1 && motor_encoder_l_old == 0) 
    {
      pulse_count_l++;      
    }
    motor_encoder_l_old = motor_encoder_l;       
     
    if(motor_encoder_r == 1 && motor_encoder_r_old == 0) 
    {
      pulse_count_r++;      
    }
    motor_encoder_r_old = motor_encoder_r;   
    
    // speed control loop (scontrol_freq)
    // if i = 0, 5, 10, 15, ...
    if(i == sensor_freq/(pcontrol_freq*scontrol_freq)*j)
      {
        // calculate speed from motor encoders
        float theta_dot_a_l = avelocity(motor_encoder_srtime_l_previous, pulse_count_l, pulse_count_l_old);
        float theta_dot_a_r = avelocity(motor_encoder_srtime_r_previous, pulse_count_r, pulse_count_r_old);
        
        // int to float problem?
        float dp = pulse_count_r - pulse_count_r_old;  
        float ds = dp*2*pi/(12*34);   // [rad]

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
        
        // calculate c (duty cycle) using scontrol() controller and transform to arduino motor voltages        
        float c_l = scontrol(theta_dot_d_l, theta_dot_a_l, e_old_l, E_l);        
        int MotorL = 255*c_l;
        
        float c_r = scontrol(theta_dot_d_r, theta_dot_a_r, e_old_r, E_r);
        int MotorR = 255*c_r;         
       
        // turn motors completely off if a stop is issued
        if (mov_type == 5) 
        {
          MotorL = 0;
          MotorR = 0;
        }
        
        //Serial.println(c_l);
        
        // set motor voltages (minimum needed is 150?)
        setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorL);
        setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorR);
       
        // debug statements (debug_freq)
        if (i == sensor_freq/(pcontrol_freq*debug_freq)*k)
        {
          Serial.println(dp);   // sample frequency increases dp?
          k++; 
        }      
        //if(theta_dot_a_l != 0)
        //{
        // Serial.println(theta_dot_a_l);
        //}
        //if(theta_dot_a_r != 0)
        //{
        //  Serial.println(theta_dot_a_r);
        //} 
        
        pulse_count_l_old = pulse_count_l;
        pulse_count_r_old = pulse_count_r;
        j++;
      }
    
    // use the read start time of one step before, the previous step
    motor_encoder_srtime_l_previous = motor_encoder_srtime_l;
    motor_encoder_srtime_r_previous = motor_encoder_srtime_r;
      
    i++;
    sleep(sensor_time, start_sensor_timer);
  }   
  
  // debug statements 1 Hz
  //Serial.println(pulse_count_l);    //a whole turn is 12*34 = 408 pulses
  //Serial.println(pulse_count_r);
  //Serial.println(theta_dot_a_r);
  //Serial.println(theta_dot_a_l);
  //Serial.println(c_l);
  //Serial.println(c_r);
}  
    
void sleep(float sensor_time, float ts) 
{
  float te = millis();  
  while (abs(te-ts) < sensor_time) 
  {    
    te = millis();
  } 
}
