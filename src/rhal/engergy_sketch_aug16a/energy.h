#ifndef ENERGY_H
#define ENERGY_H

void callbackReportEnergy0(void);

class energy{

  public:
  energy(int sensorPin,int numReadings, int interval, int sensorNum);
  void readProcedure(void);


  private:

  unsigned int _sensorPin;

  unsigned int _numReadings; // a two power, the bitshift amount 

  unsigned int _interval;

  unsigned int _sensorNum;

           long _sumEnergy;

  unsigned long _lastReading;

  

  };





#endif


