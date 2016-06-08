// global variables
#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

const int ledPin = 13;
float e_old = 0.0;
int motor_encoder_l_old = 0;
int motor_encoder_r_old = 0;
int pulse_count_l = 0;
int pulse_count_l_old = 0;
int pulse_count_r = 0;
int pulse_count_r_old = 0;

// send information to rpi

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

void loop() { 
  // set inner/outer loop frequencies
  const float sensor_freq = 1000.0;        // [1/s]
  const float scontrol_freq = 200.0;
  const float pcontrol_freq = 1.0;          // [1/s]
  const float sensor_time = 1000/sensor_freq;  // [ms]  
  
  // recieve rpi information
  float x_d = 0.0;
  float y_d = 0.0;
  float phi_a = 0.0;
  
  // recieve rpi information test
  //if (Serial.available())  {
  //  blink(Serial.read() - '0');  // convert the character '1'-'9' to decimal 1-9
  //}
  //delay(500);
  
  x_d = 10;
  y_d = 10;
  phi_a = 10;    
  
  // location2angle
  int phi_d = 0;
  int x_a = 0;
  int y_a = 0;
  
  phi_d = atan2(y_d - y_a, x_d - x_a);
  
  // outerloop/PID
  const int delta_t = 200;    //delta_t = h2, sample time
  const int Kp = 1;
  const int Ki = 1;
  const int Kd = 1;
  float E = 0.0; 
  
  float v = 0.25;            // [m/s]
  
  float e = phi_d - phi_a;
  float e_dot = e - e_old;
  E = E + e;
  float omega = Kp*e + Ki*E + Kd*e_dot;
  e_old = e;
  
  // inverse kinematics
  const float R = 0.07;         // [m]
  const float L = 0.45;         // [m]
  float theta_dot_d_l = 0.0;  
  float theta_dot_d_r = 0.0;    
  
  theta_dot_d_r = (2*v + omega*L)/(2*R);
  theta_dot_d_l = (2*v - omega*L)/(2*R);
  
  // inner loop
  int MotorL = 0;
  int MotorR = 0;  
  
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
    
    int motor_encoder_l = digitalRead(2);
    float motor_encoder_srtime_l = millis();    
    int motor_encoder_r = digitalRead(3);
    float motor_encoder_srtime_r = millis();
    
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
    
    if(i == sensor_freq/(pcontrol_freq*scontrol_freq)*j) // i mod x, gives 0 when x fits (multiple times) into the bigger number i
      {
      float motor_encoder_ertime_l = millis();
      float theta_dot_a_l  = (pulse_count_l - pulse_count_l_old)*(0.6/1000)/(motor_encoder_ertime_l - motor_encoder_srtime_l);
      float motor_encoder_ertime_r = millis();
      float theta_dot_a_r  = (pulse_count_r - pulse_count_r_old)*(0.6/1000)/(motor_encoder_ertime_r - motor_encoder_srtime_r);
      
      const int K[2] = {0.3101, -0.0688};
      const int F = -0.3474;
          
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
      
      // motor control (minimum needed is 150?)
      setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorL);
      setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorR); 
      
      pulse_count_l_old = pulse_count_l;
      pulse_count_r_old = pulse_count_r;
      j++;
      }
    i++;
    sleep(sensor_time, start_sensor_timer);
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

void sleep(float sensor_time, float ts) {
  float te = millis();  
  while (abs(te-ts) < sensor_time) {    
    te = millis();
  } 
}
