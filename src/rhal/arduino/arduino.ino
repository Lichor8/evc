// global variables
#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

const int timeout = 1000/50;
const float pi = 3.14159;
const int ledPin = 13;
String rpiData; //"0x127|y135|";

// the setup function runs once when you press reset or power the board
void setup() {
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
  
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(9600);
  Serial.setTimeout(timeout);
}

// position control loop (pcontrol_freq)
void loop() 
{ 
// initialize variables
    
  // recieve rpi information
  int mov_type = -1;
  float x_d = 0.0;
  float y_d = 0.0;
  float turn_deg = 0;
  float stop_sec = 0;
  
  //location2angle
  float phi_a = pi/2; // using coordinate frame on the robot (relative) so actual is always pi/2
  float x_a   = 0.0;  // using coordinate frame on the robot (relative) so actual is always 0
  float y_a   = 0.0;  // using coordinate frame on the robot (relative) so actual is always 0
  float phi_d = 0.0;  // initialize
  
  // outer loop control
  float e_old = 0.0;
  float E = 0.0;
  float omega = 0.0;
  float v     = 0.0;
  
  // encoder loop  
  int motor_encoder_l_old = 0;
  int motor_encoder_r_old = 0;
  int pulse_count_l = 0;
  int pulse_count_l_old = 0;
  int pulse_count_r = 0;
  int pulse_count_r_old = 0;
  
  // velocity control loop
  float motor_encoder_srtime_l = 0.0;
  float motor_encoder_srtime_r = 0.0;
  float motor_encoder_srtime_l_previous = 0.0;
  float motor_encoder_srtime_r_previous = 0.0;
  float theta_dot_a_l = 0.0;
  float theta_dot_a_r = 0.0;
  
  while(1)
  {
    // set inner/outer loop frequencies
    const float pcontrol_freq = 1.0;               // position controol loop frequency [1/s]
    const float scontrol_freq = 200.0;             // speed control loop frequency     [1/s]
    const float sensor_freq   = 1000.0;            // sensor sample loop frequency     [1/s]
    const float sensor_time   = 1000/sensor_freq;  // sensor sample time               [ms]
    
    // receive rpi information   
    rpi2arduino(mov_type, x_d, y_d, turn_deg, stop_sec);
    //char inData[20];   // Allocate some space for the string
    //char inChar = -1;  // Where to store the character read
    //byte index  = 0;   // Index into array; where to store the character

    //if (Serial.available())
    //{
    // Serial.read();
    //}
    
    // receive rpi information test
    //if (Serial.available())  {
    //  blink(Serial.read() - '0');  // convert the character '1'-'9' to decimal 1-9
    //}
    //delay(500);
    
    x_d = 0;    // [m]
    y_d = 1;    // [m]  
    
    // location2angle
    //phi_a = pi;   
    //x_a   = 0.0;    
    //y_a   = 0.0;    
    //phi_d = 0.0;
    
    phi_d = atan2(y_d - y_a, x_d - x_a);
        
    // outer loop control (pcontrol_freq)
    pcontrol(phi_d, phi_a, omega, v, e_old, E);  
    Serial.println(omega);
    
    // inverse kinematics
    const float R = 0.07;         // [m]
    const float L = 0.45;         // [m]
    float theta_dot_d_l = 0.0;  
    float theta_dot_d_r = 0.0;    
    
    theta_dot_d_r = (2*v + omega*L)/(2*R);  // [rad/s]
    theta_dot_d_l = (2*v - omega*L)/(2*R);  // [rad/s]
    
    // sensor sample loop (sensor_freq)
    int MotorL = 0;
    int MotorR = 0;  
    //Serial.println(MotorL);
    //Serial.println(MotorR);
    
    //Serial.println(pulse_count_l);
    
    int i = 0;
    int j = 1;
    while(i < sensor_freq/pcontrol_freq) 
    {
      float start_sensor_timer = millis();
      
      // read sensors
      float dc_current_a_l = 0.0;
      float dc_current_a_r = 0.0;    
      //dc_current_a_l = 
      //dc_current_a_r = 

      // if i = 1, 6, 11, 16, ... then set start time
      // x(j-1) + 1 = xj - (x-1) = 5j - 4 = 1, 6, 11, 16, ...
      //if(i == (sensor_freq/(pcontrol_freq*scontrol_freq))*(j-1) + 1 )
      //{
      //  Serial.println(i);
      //  motor_encoder_srtime_l = millis();    
      //  motor_encoder_srtime_r = millis();
      //}    
    
      // if i = 0, 5, 10, 15, ... then set start time
      // 
      if(i == sensor_freq/(pcontrol_freq*scontrol_freq)*j)
      {
        //Serial.println(i);
        motor_encoder_srtime_l = millis();    
        motor_encoder_srtime_r = millis();
      }   
      
      int motor_encoder_l = digitalRead(2);
      int motor_encoder_r = digitalRead(3);
      
      if(motor_encoder_l == 1 && motor_encoder_l_old == 0) 
      {
        pulse_count_l++;      
      }
      motor_encoder_l_old = motor_encoder_l;       
       
      if(motor_encoder_r == 1 && motor_encoder_r_old == 0) 
      {
        pulse_count_r++;      
      }
      motor_encoder_r_old = motor_encoder_r;   
      
      // speed control loop (scontrol_freq)
      if(i == sensor_freq/(pcontrol_freq*scontrol_freq)*j) // i mod x, gives 0 when x fits (multiple times) into the bigger number i
        {
          //// left motor 
          // end read time of encoder
          float motor_encoder_ertime_l = millis(); 
          
          // calculate differences
          float dt_l = (motor_encoder_ertime_l - motor_encoder_srtime_l_previous)/1000; //[s]
          float dp_l = pulse_count_l - pulse_count_l_old;
          
          // pulse2rad (12 magnets in sensor, gear ratio 34:1)
          float ds_l = dp_l*2*pi/(12*34);   // [rad]
          
          // calculate speed
          theta_dot_a_l = ds_l/dt_l;        // [rad/s]
          
          //// right motor 
          // end read time of encoder       
          float motor_encoder_ertime_r = millis();  
          
          // calculate differences
          float dt_r = (motor_encoder_ertime_r - motor_encoder_srtime_r_previous)/1000; // [s]
          float dp_r = pulse_count_r - pulse_count_r_old;
          
          // pulse2rad (12 magnets in sensor, gear ratio 34:1)
          float ds_r = dp_r*2*pi/(12*34);    // [rad]
          
          // calculate speed
          theta_dot_a_r = ds_r/dt_r;  // [rad/s]
              
          // static controller gains
          const int K[2] = {0.3101, -0.0688};
          const int F = 0;
              
          //Serial.println(j*5);
          //if(dp_l != 0)
          //{
          //  Serial.println(dp_l);
          //}
          //if(dp_r != 0)
          //{
          //  Serial.println(dp_r);
          //}
          //Serial.println(dt_l);
          //Serial.println(dt_r);
          //if(theta_dot_a_l != 0)
          //{
          // Serial.println(theta_dot_a_l);
          //}
          //if(theta_dot_a_r != 0)
          //{
          //  Serial.println(theta_dot_a_r);
          //}
          
          float r_l = theta_dot_d_l;
          float x1_l = theta_dot_a_l;
          float x2_l = dc_current_a_l;
          //float c_l = K[0]*x1_l + K[1]*x2_l + F*r_l;
          float c_l = (theta_dot_d_l - theta_dot_a_l)*1;
          MotorL = 255*c_l;
          
          float r_r = theta_dot_d_r;
          float x1_r = theta_dot_a_r;
          float x2_r = dc_current_a_r;
          //float c_r = K[0]*x1_r + K[1]*x2_r + F*r_r;
          float c_r = (theta_dot_d_r - theta_dot_a_r)*1;
          MotorR = 255*c_r;
          
          //Serial.println(c_l);
          
          // motor control (minimum needed is 150?)
          setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorL);
          setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorR); 
          
          pulse_count_l_old = pulse_count_l;
          pulse_count_r_old = pulse_count_r;
          j++;
        }
      
      // use the read start time of 1 step before, the previous step
      motor_encoder_srtime_l_previous = motor_encoder_srtime_l;
      motor_encoder_srtime_r_previous = motor_encoder_srtime_r;
        
      i++;
      sleep(sensor_time, start_sensor_timer);
    }     
  } 
}

void blink(int numberOfTimes){
  for (int i = 0; i < numberOfTimes; i++)  {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

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

void pcontrol(float phi_d, float phi_a, float &omega, float &v, float &e_old, float &E)
{  
  const int Kp = 1;
  const int Ki = 1;
  const int Kd = 1; 
  
  float e = phi_d - phi_a;
  float e_dot = e - e_old;
  E = E + e;
  //omega = Kp*e + Ki*E + Kd*e_dot;
  omega = Kp*e;
  e_old = e;
  
  v = 0.0;            // [m/s]
}

void sleep(float sensor_time, float ts) {
  float te = millis();  
  while (abs(te-ts) < sensor_time) {    
    te = millis();
  } 
}

void rpi2arduino(int &mov_type, float &x_d, float &y_d, float &turn_deg, float &stop_sec)
{
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
      digitalWrite(13, LOW);
          
      if(mov_type == 0 && rpiData[1 + end_index] == 'y')
      {
        y_d = read_data(begin_index + end_index, rpiData, end_index);
        Serial.println(y_d);
      }
    }
    
    // if movement type is turn (mov_type = 4) then read degrees
    if(mov_type == 4 && rpiData[1] == 'd')
    {
      turn_deg = read_data(begin_index, rpiData, end_index);
      Serial.println(turn_deg);
    }
        
    // if movement type is stop (mov_type = 5) then read stop time
    if(mov_type == 5 && rpiData[1] == 't')
    {
      stop_sec = read_data(begin_index, rpiData, end_index);
      Serial.println(stop_sec);
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
