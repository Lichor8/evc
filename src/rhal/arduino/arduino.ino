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
  
  // recieve rpi information
  float x_d = 0.0;
  float y_d = 0.0;
  float phi_a = 0.0;
  
  // recieve rpi information test
  //if (Serial.available())  {
  //  blink(Serial.read() - '0');  // convert the character '1'-'9' to decimal 1-9
  //}
  //delay(500);
  
  //x_d = 
  //y_d =
  //phi_a = 
 
 // read sensors
  float dc_current_a_l = 0.0;
  float dc_current_a_r = 0.0;
  float theta_dot_a_l  = 0.0;
  float theta_dot_a_r  = 0.0;
  int x_a = 0;
  int y_a = 0;
  
  //dc_current_a_l = 
  //dc_current_a_r = 
  //theta_dot_a_l  = 
  //theta_dot_a_r  =    
  
  // location2angle
  int phi_d = 0;
  
  phi_d = atan2(y_d - y_a, x_d - x_a);
  
  // outerloop/PID
  const int delta_t = 200;    //delta_t = h2, sample time
  const int Kp = 1;
  const int Ki = 1;
  const int Kd = 1;
  float v = 0.0;
  float e = 0.0;
  float e_dot = 0.0;
  float E = 0.0;
  float omega = 0.0; 
  
  v = 0.25;            // [m/s]
  
  e = phi_d - phi_a;
  e_dot = e - e_old;
  E = E + e;
  omega = Kp*e + Ki*E + Kd*e_dot;
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
  
  for(int i=0; i<20; i=i+1) {
    const int K[2] = {0.3101, -0.0688};
    const int F = -0.3474;
    float r_l  = 0.0;
    float x1_l = 0.0;
    float x2_l = 0.0;
    float c_l  = 0.0;
    float r_r  = 0.0;
    float x1_r = 0.0;
    float x2_r = 0.0;
    float c_r  = 0.0;
        
    r_l = theta_dot_d_l;
    x1_l = theta_dot_a_l;
    x2_l = dc_current_a_l;
    c_l = K[0]*x1_l + K[1]*x2_l + F*r_l;
    MotorL = 255*c_l;
    
    r_r = theta_dot_d_r;
    x1_r = theta_dot_a_r;
    x2_r = dc_current_a_r;
    c_r = K[0]*x1_r + K[1]*x2_r + F*r_r;
    MotorR = 255*c_r;
    
    // motor control (minimum needed is 150?)
    setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorL);
    setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorR); 
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

void sleep(float stime, float ts) {
  float te = 0.0;  
  while (abs(te-ts) < stime) {    
    te = millis();
  } 
}
