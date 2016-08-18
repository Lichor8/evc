#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

// import libraries
#include <Arduino.h>

//functions
void setMotor(const unsigned char cucPWM, const unsigned char cucFWD , const unsigned char cucBWD, const int ciSpeed);
void pcontrol(float phi_d, float phi_a, float &omega, float &v, float &e_old, float &E);

#endif
