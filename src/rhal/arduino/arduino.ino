#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

// global variables
const int delta_t = 200;    //delta_t = h2, sample time
const int Kp = 1;
const int Ki = 1;
const int Kd = 1;
const int R = 0.07;         // [m]
const int L = 0.45;         // [m]
int x_a   = 0;
int y_a   = 0;
int phi_d = 0;
int e     = 0;
int e_dot = 0;
int E     = 0;
int old_e = 0;
int vd    = 0;
int theta_dot_d_r    = 0;
int theta_dot_d_l    = 0;

// recieve rpi information
const int ledPin = 13;
int x_d = 0;
int y_d = 0;
int phi_a = 0;

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
  
  // recieve rpi information test
  if (Serial.available())  {
    blink(Serial.read() - '0');  // convert the character '1'-'9' to decimal 1-9
  }
  delay(500);
  
  // recieve rpi information
  //x_d = 
  //y_d = 
  //phi_a = 
  
  // location2angle
  phi_d = atan2(y_d - y_a, x_d - x_a)
  
  // outerloop/PID
  v = 0.25;            // [m/s]
  e = phi_d - phi_a;
  e_dot = e - e_old;
  E = E + e;
  omega = Kp*e + Ki*E + Kd*e_dot;
  old_e = e;
  
  // inverse kinematics
  theta_dot_d_r = (2*v + omega*L)/(2*R)
  theta_dot_d_l = (2*v - omega*L)/(2*R)
  
  // inner loop
  
  

  // motor control  
  int MotorL = 0;
  int MotorR = 0;

  setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorL);
  setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorR);  
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
