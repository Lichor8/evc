#include "energy.h"
//#include "Common.h"
#include "Arduino.h"
#include "Timer.h"
//#include "Definitions.cpp"


#define INFO_ENERGY(message) Serial.println(message)
extern energy energySolar;
extern energy energyPi;
extern Timer t;

void callbackReportEnergy0(void){
    
    energySolar.readProcedure();
    energyPi.readProcedure();
  };

energy::energy(int sensorPin,int numReadings, int interval,int sensorNum){
  INFO_ENERGY("REGISTER SENSOR");
  
  _sensorPin = sensorPin;
  _numReadings = numReadings;
  _interval = interval;
  _sensorNum = sensorNum;
  _sumEnergy = 0;

  if(sensorNum ==0){
     t.every(_interval, callbackReportEnergy0) ;//call the function every second
     
  }
  
  };

void energy::readProcedure(void){//read the input from _sensorpin using analogRead
  
  long sumReadings = 0;
  for(int reading = 0;reading< (1<<_numReadings);reading++)
  {
    sumReadings += analogRead(_sensorPin);
    delay(1);
   
  }
  
  sumReadings = (sumReadings >> _numReadings); // average current current & remove offset
  sumReadings -=512;
  if(sumReadings > 189)
	  sumReadings = 189;
  if(sumReadings < -189)
	  sumReadings = -189;
  
  sumReadings = map(sumReadings,-189,189,-5000,5000);//map(value: the number to map, fromLow: the lower bound of the value's current range,fromHigh: the upper bound of the value's current range, toLow: the lower bound of the value's target range, toHigh: the upper bound of the value's target range )
  sumReadings = (sumReadings << 2) ;
  sumReadings -= (sumReadings * 0.06) ;
  
  if(sumReadings < 0)
    sumReadings = 0;
  unsigned long timeNow = millis();
  Serial.println("SENSOR: "+String(_sensorNum)+" AVG: "+String(sumReadings)+" dT: " +String(timeNow-_lastReading)+ " add E: " + String((long)((sumReadings * (timeNow-_lastReading))>>10)));
  _sumEnergy += (long)((sumReadings * (timeNow-_lastReading))>>10); // A milisecond
  Serial.println("sumEnergy:"+String(_sumEnergy));
  
  _lastReading = timeNow;
   //Serial.println("Current current:" + String(sumReadings) + "total E: "+String(_sumEnergy));
  
  };

