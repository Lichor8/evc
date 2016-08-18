#ifndef MOVEMENT_H
#define MOVEMENT_H

// import libraries
#include <Arduino.h>

// functions
void movement(int mov_type, float x_d, float y_d, float turn_deg, float stop_sec);
void sleep(float sensor_time, float ts);

#endif
