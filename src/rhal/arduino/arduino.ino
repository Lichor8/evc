// global variables
#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

const float pi = 3.14159;
const int ledPin = 13;

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
}

// position control loop (pcontrol_freq)
void loop() 
{ 
// initialize variables
    
  // recieve rpi information
  float x_d = 0.0;
  float y_d = 0.0;
  
  //location2angle
  float phi_a = pi;   // using coordinate frame on the robot (relative) so actual is always pi
  float x_a   = 0.0;  // using coordinate frame on the robot (relative) so actual is always 0
  float y_a   = 0.0;  // using coordinate frame on the robot (relative) so actual is always 0
  float phi_d = 0.0;  // initialize
  
  // outer loop control
  float e_old = 0.0;
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
    
    // recieve rpi information    
    
    // recieve rpi information test
    //if (Serial.available())  {
    //  blink(Serial.read() - '0');  // convert the character '1'-'9' to decimal 1-9
    //}
    //delay(500);
    
    x_d = 10;    // [m]
    y_d = 10;    // [m]  
    
    // location2angle
    //phi_a = pi;   
    //x_a   = 0.0;    
    //y_a   = 0.0;    
    //phi_d = 0.0;
    
    phi_d = atan2(y_d - y_a, x_d - x_a);
    
    // outer loop control (pcontrol_freq)
    pcontrol(phi_d, phi_a, omega, v, e_old);  
    //Serial.println(omega);
    
    // inverse kinematics
    const float R = 0.07;         // [m]
    const float L = 0.45;         // [m]
    float theta_dot_d_l = 0.0;  
    float theta_dot_d_r = 0.0;    
    
    theta_dot_d_r = (2*v + omega*L)/(2*R);
    theta_dot_d_l = (2*v - omega*L)/(2*R);
    
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
          float dt_l = motor_encoder_ertime_l - motor_encoder_srtime_l_previous; //[ms]
          float dp_l = pulse_count_l - pulse_count_l_old;
          
          // pulse2mm
          float ds_l = dp_l*(0.6);    // [mm]
          
          // calculate speed
          theta_dot_a_l = ds_l/dt_l;  // m/s
          
          //// right motor 
          // end read time of encoder       
          float motor_encoder_ertime_r = millis();  
          
          // calculate differences
          float dt_r = (motor_encoder_ertime_r - motor_encoder_srtime_r_previous); // [ms]
          float dp_r = pulse_count_r - pulse_count_r_old;
          
          // pulse2rad
          float ds_r = dp_r*(0.6);    // [mm]
          
          // calculate speed
          theta_dot_a_r = ds_r/dt_r;  // [m/s]
              
          // static controller gains
          const int K[2] = {1, 1};
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
          float c_l = K[0]*x1_l + K[1]*x2_l + F*r_l;
          MotorL = 255*c_l;
          
          float r_r = theta_dot_d_r;
          float x1_r = theta_dot_a_r;
          float x2_r = dc_current_a_r;
          float c_r = K[0]*x1_r + K[1]*x2_r + F*r_r;
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

void pcontrol(float phi_d, float phi_a, float &omega, float &v, float &e_old)
{  
  const int Kp = 1;
  const int Ki = 1;
  const int Kd = 1;
  float E = 0.0; 
  
  float e = phi_d - phi_a;
  float e_dot = e - e_old;
  E = E + e;
  omega = Kp*e + Ki*E + Kd*e_dot;
  e_old = e;
  
  v = 0.25;            // [m/s]
}

void sleep(float sensor_time, float ts) {
  float te = millis();  
  while (abs(te-ts) < sensor_time) {    
    te = millis();
  } 
}
