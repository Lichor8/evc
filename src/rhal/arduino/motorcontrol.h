#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

// import libraries
#include <Arduino.h>

//functions
void pcontrol(float phi_d, float phi_a, float &omega, float &v, float &e_old, float &E);
float scontrol(float theta_dot_d, float theta_dot_a, float &e_old, float &E);
float scontrol_l(float theta_dot_d, float theta_dot_a, float &e_old_l, float &E_l);
float scontrol_r(float theta_dot_d, float theta_dot_a, float &e_old_r, float &E_r);
void setMotor(const unsigned char cucPWM, const unsigned char cucFWD , const unsigned char cucBWD, const int ciSpeed);

#endif
