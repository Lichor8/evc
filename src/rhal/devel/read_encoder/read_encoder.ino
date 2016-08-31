const float sensor_freq   = 1000.0;            // sensor sample loop frequency     [1/s]
const float sensor_time   = 1000000/sensor_freq;  // sensor sample time               [ms]
const float debug_freq = 1;  // [1/s]

int motor_encoder_l_old = 0;    
int motor_encoder_r_old = 0;

//int motor_encoder_l = 0;
//int motor_encoder_r = 0;

int pulse_count_l = 0;
  int pulse_count_r = 0;

// the setup function runs once when you press reset or power the board
void setup() 
{  
  Serial.begin(9600);
}

void loop() 
{ 
  // initialize variables  
  int pulse_count_l = 0;
  int pulse_count_r = 0;  
  
  int i = 0;
  int k = 1;
  Serial.println("hi");
  while(i < sensor_freq*5) //i < sensor_freq/pcontrol_freq
  {
    float start_sensor_timer = micros(); 
    
    int motor_encoder_l = digitalRead(2); //2
    int motor_encoder_r = digitalRead(3);
    
    if(motor_encoder_l == HIGH && motor_encoder_l_old == LOW) 
    {
      pulse_count_l++;      
    }
    motor_encoder_l_old = motor_encoder_l;       
     
    if(motor_encoder_r == HIGH && motor_encoder_r_old == LOW) 
    {
      pulse_count_r++;    
    }
    motor_encoder_r_old = motor_encoder_r;   
  
        // debug statements (debug_freq)
//    if (i == sensor_freq/(debug_freq)*k)
//        {
//          Serial.println(pulse_count_r);   // sample frequency increases dp?
//          k++; 
//        }
  
    i++;
    sleep(sensor_time, start_sensor_timer);
  }
  Serial.println(pulse_count_l);
  //Serial.println(i);
} 
  
void sleep(float sensor_time, float ts) 
{
  float te = micros();  
  while (abs(te-ts) < sensor_time) 
  {    
    te = micros();
  } 
}
