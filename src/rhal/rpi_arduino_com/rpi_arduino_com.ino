const int timeout = 1000/50;
String rpiData; //"0x127|y135|";

void setup() {
    Serial.begin(9600);
    //Serial.setTimeout(timeout); 
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

void loop()
{
  int mov_type = -1;
  int x_d = 0;
  int y_d = 0;
  int turn_deg = 0;
  int stop_sec = 0;
  
  rpi2arduino(mov_type, x_d, y_d, turn_deg, stop_sec);
}

void rpi2arduino(int &mov_type, int &x_d, int &y_d, int &turn_deg, int &stop_sec)
{
  if(Serial.available()) // Don't read unless
  {
    // read serial data from raspberry pi
    //rpiData = Serial.readStringUntil('e');
    rpiData = Serial.readString();
              
    mov_type = rpiData[0] - '0';  // convert the character '1'-'9' to decimal 1-9
    int end_index = 0;
    int begin_index = 2;   
        
    // if movement type is drive between lines (mov_type = 0) then read x and y position of goal point
    if(rpiData[1] == 'x')//rpiData[1] == 'x'  //mov_type == 0
    {
      //x_d = read_data(begin_index, rpiData, end_index);
      Serial.println(5);
      digitalWrite(13, LOW);
          
      //if(mov_type == 0 && strcmp(rpiData[1 + end_index], 'y') == 0)
      //{
      //  y_d = read_data(begin_index + end_index, rpiData, end_index);
      //}
    }
    
    // if movement type is turn (mov_type = 4) then read degrees
    //if(mov_type == 4 && strcmp(rpiData[1], 'd') == 0)
    //{
    //  turn_deg = read_data(begin_index, rpiData, end_index);
    //}
        
    // if movement type is stop (mov_type = 5) then read stop time
    //if(mov_type == 5 && strcmp(rpiData[1], 't') == 0)
    //{
    //  stop_sec = read_data(begin_index, rpiData, end_index);
    //}
  }
}


int read_data(int begin_index, String rpiData, int &end_index)
{
  int k = 0;
  int data[5];
  int data_num;
  
  while(rpiData[begin_index + k] != '|')
  {
    data[k] = rpiData[k+2]  - '0';
    k++;
  }
  end_index = k;
    
  int l = 0;
  while(k >= 0)
  {
    data_num = data_num + data[k-l]*10^l;
    l++;
  }
  return data_num;
}
