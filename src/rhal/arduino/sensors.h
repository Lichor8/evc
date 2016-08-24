#ifndef SENSORS_H
#define SENSORS_H

// import libraries
#include <Arduino.h>

//functions
float avelocity(float motor_encoder_srtime_previous, int pulse_count, int pulse_count_old);

#endif
