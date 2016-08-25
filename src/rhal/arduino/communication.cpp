// import libraries
#include <Arduino.h>
#include "communication.h"


// functions
void rpi2arduino(int &mov_type, float &x_d, float &y_d, float &turn_r, float &drive_m, float &turn_deg, float &stop_sec)
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
      Serial.println(x_d);
          
      if(mov_type == 0 && rpiData[1 + end_index] == 'y')
      {
        y_d = read_data(begin_index + end_index, rpiData, end_index);
        Serial.println(y_d);
      }
    }
    
    // if movement type is turn left (mov_type = 1) then read turn radius
    if(mov_type == 1 && rpiData[1] == 'r')
    {
      turn_r = read_data(begin_index, rpiData, end_index);
      //Serial.println(turn_r);
    }
    
    // if movement type is drive (mov_type = 2) then read distance
    if(mov_type == 2 && rpiData[1] == 'd')
    {
      drive_m = read_data(begin_index, rpiData, end_index);
    }
    
    // if movement type is turn left (mov_type = 3) then read turn radius
    if(mov_type == 3 && rpiData[1] == 'r')
    {
      turn_r = read_data(begin_index, rpiData, end_index);
    }
    
    // if movement type is turn (mov_type = 4) then read degrees
    if(mov_type == 4 && rpiData[1] == 'a')
    {
      turn_deg = read_data(begin_index, rpiData, end_index);
    }
        
    // if movement type is stop (mov_type = 5) then read stop time
    if(mov_type == 5 && rpiData[1] == 't')
    {
      stop_sec = read_data(begin_index, rpiData, end_index);
    }
  }
}


float read_data(int begin_index, String rpiData, int &end_index)
{
  int k = 0;
  int sign = 1;
  int commals = 0;
  int commal = 0;
  float comma = 1.0;      // float is necessary, because pow() gives a float: then turning into 'int' causes rounding/trunancation errors
  int data[10];
  float data_num = 0.0;
  
  // to proces negative numbers
  if (rpiData[begin_index] == '-')
  {
    sign = -1;
    begin_index++;    
  }
  
  // turn number string into integers
  while(rpiData[begin_index + k + commals] != '|')
  {
    // to proces floats
    if (rpiData[begin_index + k] == '.')
    {
      commals = 1;     // a comma is present in the number
      commal = k;      // comma location
    }
  
    data[k] = rpiData[begin_index + k + commals]  - '0';
    //Serial.println(data[k]);
    k++;    
  }
  end_index = begin_index + k + commals;
  //Serial.println(end_index); 
  
  // concatenate integer numbers saved in data array
  int l = 0;
  while(l < k)
  {
    data_num = data_num + data[k-1-l]*pow(10,l);
    //Serial.println(data_num);
    l++;
  }
  
  // transform data_num into positive or negative float
  if (commals)
  {
    //Serial.println(end_index);
    //Serial.println(end_index - 1 - (begin_index + commal));
    comma = pow(10, (end_index - 1 - (begin_index + commal)));    
  }
  //Serial.println(comma);
  data_num = sign*data_num/comma;
  //Serial.println(data_num);
  return data_num;
}
