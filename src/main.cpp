#include <Arduino.h>
#include <math.h>

#define CHIP 1 // 0 = 12 Bit, 1 = 16 Bit


#if CHIP == 1
  #define datapin 11
  #define clockpin 13
  #include <Adafruit_TLC59711.h>
  #include <SPI.h>
#else
  #include <Adafruit_PWMServoDriver.h>
#endif

#define NUM_OUT 5        // number of output channels
#define DT_MIN 100      // shortest fade time [ms]
#define DT_MAX 300     // longest fade time [ms]
#define DI_MIN 0.5       // minimum intensity change per fade [%]
#define A 2.5            // dimming curve factor

unsigned long* t_start;  // holds start time of current fade for each channel
unsigned long* t_stop;   // holds stop time of current fade for each channel
float* i_start;          // holds start intensity of current fade for each channel
float* i_stop;           // holds stop intensity of current fade for each channel

unsigned long t;         // holds current time
unsigned long dt;        // for measuring time deltas
unsigned long length;    // length of a fade
float intensity;         // single intensity value
float di;                // intensity delta of a fade
float prog;              // progress of a fade
unsigned long out;       // output value for PWM board

#if CHIP == 1
  Adafruit_TLC59711 pwm = Adafruit_TLC59711(1,clockpin,datapin);
#else
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif




void next(uint8_t i){
  length = (unsigned long) random(DT_MIN * 1000L, DT_MAX * 1000L);
  t_start[i] = t;
  t_stop[i] = t + length; 

  // if last fade was up, next one is down
  if (i_stop[i] > i_start[i]){
    intensity = (float)random(0, (i_stop[i] - DI_MIN)*1000)/1000;
  }
  // if last fade was down, next one is up
  else{
    intensity = (float)random((i_stop[i] + DI_MIN)*1000, 1000)/1000;
  }

  i_start[i] = i_stop[i];
  i_stop[i] = intensity;
}

void setup() {
  Serial.begin(9600);
  t_start = (unsigned long*) calloc(NUM_OUT, sizeof(unsigned long));
  t_stop = (unsigned long*) calloc(NUM_OUT, sizeof(unsigned long));
  i_start = (float*) calloc(NUM_OUT, sizeof(float));
  i_stop = (float*) calloc(NUM_OUT, sizeof(float));

  randomSeed(analogRead(0));

  #if CHIP == 1
    pwm.begin();
    pwm.write();
  #else
    pwm.begin();
  #endif
  

}

void loop() {
  for(uint8_t i=0; i<NUM_OUT; i++){
    t = micros();
    length = t_stop[i] - t_start[i];
    dt     =         t - t_start[i];    

    // if fade is over, generate next one
    if (dt >= length){
      next(i);

    }else{      
      di = i_stop[i] - i_start[i];
      prog = (float)dt/(float)length; 
      intensity = i_start[i] + sin(prog*PI/2) * di;      

    #if CHIP == 1
      out = (unsigned long)((exp(A*intensity)-1)/(exp(A)-1)*65536);
      pwm.setPWM(i, out);      
    #else
      out = (unsigned long)((exp(A*intensity)-1)/(exp(A)-1)*4096);
      pwm.setPWM(i, 0, out);
    #endif    
    }
  }
  #if CHIP == 1
    pwm.write();   
    delay(10);
  #endif
  
}