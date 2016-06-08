void setup() {
}

void loop() {
  float pts = millis();
  
  const int sfreq = 100;        // [1/s]
  const int pfreq = 1;          // [1/s]
  const float stime = 1/sfreq;  // [s]  
  //const float ptime = 1/sfreq;  // [s]
  
  int i = 0;
  while (i < sfreq/pfreq) {
    float sts = millis();
    //read  
    i++;
    sleep(stime, sts);
  } 
  
  //print  
  //sleep(ptime, pts);  
}


void sleep(float stime, float ts) {
  float te = 0.0;  
  while (abs(te-ts) < stime) {    
    te = millis();
  } 
}
