// import libraries
#include <Arduino.h>
#include "communication.h"


// functions
void rpi2arduino(int &mov_type, float &x_d, float &y_d, float &turn_deg, float &stop_sec)
{
  // initialize rpiData
  String rpiData; 
  
  if(Serial.available()) // Don't read unless
  {
    // read serial data from raspberry pi
    //rpiData = Serial.readStringUntil('e');
    rpiData = Serial.readString();
    Serial.println(rpiData);
              
    mov_type = rpiData[0] - '0';  // convert the character '1'-'9' to decimal 1-9
    int end_index = 0;
    int begin_index = 2;    
        
    // if movement type is drive between lines (mov_type = 0) then read x and y position of goal point
    if(mov_type == 0 && rpiData[1] == 'x')//rpiData[1] == 'x'  //mov_type == 0
    {
      x_d = read_data(begin_index, rpiData, end_index);
      //Serial.println(x_d);
      digitalWrite(13, LOW);
          
      if(mov_type == 0 && rpiData[1 + end_index] == 'y')
      {
        y_d = read_data(begin_index + end_index, rpiData, end_index);
        //Serial.println(y_d);
      }
    }
    
    // if movement type is turn left (mov_type = 1) then read omega
//    if(mov_type == 1 && rpiData[1] == 'w')
//    {
//      turn_deg = read_data(begin_index, rpiData, end_index);
//      //Serial.println(turn_deg);
//    }
    
    // if movement type is turn (mov_type = 4) then read degrees
    if(mov_type == 4 && rpiData[1] == 'd')
    {
      turn_deg = read_data(begin_index, rpiData, end_index);
      //Serial.println(turn_deg);
    }
        
    // if movement type is stop (mov_type = 5) then read stop time
    if(mov_type == 5 && rpiData[1] == 't')
    {
      stop_sec = read_data(begin_index, rpiData, end_index);
      //Serial.println(stop_sec);
    }
  }
}


float read_data(int begin_index, String rpiData, int &end_index)
{
  int k = 0;
  int data[5];
  float data_num = 0;
    
  while(rpiData[begin_index + k] != '|')
  {
    data[k] = rpiData[begin_index + k]  - '0';
    //Serial.println(data[k]);
    k++;    
  }
  end_index = begin_index + k; //3
  //Serial.println(end_index);
    
  int l = 0;
  while(l < k)
  {
    data_num = data_num + data[k-1-l]*pow(10,l);
    //Serial.println(data_num);
    l++;
  }
  //Serial.println(data_num);
  return data_num;
}
