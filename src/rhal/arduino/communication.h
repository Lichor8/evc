#ifndef COMMUNICATION_H
#define COMMUNICATION_H

// import libraries
#include <Arduino.h>

// functions
void rpi2arduino(int &mov_type, float &x_d, float &y_d, float &turn_deg, float &stop_sec);
float read_data(int begin_index, String rpiData, int &end_index);

#endif
