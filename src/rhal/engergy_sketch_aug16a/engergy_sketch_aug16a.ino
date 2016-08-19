#include <Arduino.h>
#include "energy.h"
#include "Timer.h"

#define PIN_ENERGY_SOLAR  1
#define PIN_ENERGY_PI     0
#define ENERGY_SOLAR_INTERVAL 1000 // interval in ms
#define ENERGY_SOLAR_NUMREADINGS 7 // number of readings for averaging (2^n!!!)

#define ENERGY_PI_INTERVAL 1000 // interval in ms
#define ENERGY_PI_NUMREADINGS 7 // number of readings for averaging (2^n!!!)

energy energySolar(PIN_ENERGY_SOLAR  ,ENERGY_SOLAR_NUMREADINGS, ENERGY_SOLAR_INTERVAL,0);
//          energy(int sensorPin,int numReadings, int interval, int sensorNum);
energy energyPi(PIN_ENERGY_PI  ,ENERGY_SOLAR_NUMREADINGS, ENERGY_SOLAR_INTERVAL,1);
Timer t;


//to enable the motor
#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1


void setup()
{
  //to enable motor
  analogWrite(PWM_R, 0); 
  analogWrite(PWM_L, 0);
  digitalWrite(EN_L_FWD, LOW);
  digitalWrite(EN_L_BWD, LOW);
  digitalWrite(EN_R_FWD, LOW);
  digitalWrite(EN_R_BWD, LOW);

  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(EN_L_FWD, OUTPUT);
  pinMode(EN_L_BWD, OUTPUT);
  pinMode(EN_R_FWD, OUTPUT);
  pinMode(EN_R_BWD, OUTPUT);
  Serial.begin(9600);
}

// to enable motor
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





void loop()
{
  int MotorL = 230;
  int MotorR = 250;

  setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorL);
  setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorR);
  
  
  callbackReportEnergy0();
//SENSOR number: 0 for energy solar, 1 for energy pi 
//AVG: sumReadings if sumReadings < 0, sumReading=0
//dT: time difference between reading
//add E: added energy
//sumEnergy:

// below code from energy.cpp
//  if(sumReadings < 0)
//    sumReadings = 0;
//  unsigned long timeNow = millis();
//  Serial.println("SENSOR: "+String(_sensorNum)+" AVG: "+String(sumReadings)+" dT: " +String(timeNow-_lastReading)+ " add E: " + String((long)((sumReadings * (timeNow-_lastReading))>>10)));
//  _sumEnergy += (long)((sumReadings * (timeNow-_lastReading))>>10); // A milisecond
//  Serial.println("sumEnergy:"+String(_sumEnergy));
//  
//  _lastReading = timeNow;
//   ("Current current:" + String(sumReadings) + "total E: "+String(_sumEnergy));



}
