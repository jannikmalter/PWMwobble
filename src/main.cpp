#include <Arduino.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastLED.h>

#define NUM_OUT 8
#define DT_MIN 5
#define DT_MAX 10
#define DI_MIN 0.1

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long* t_start;
unsigned long* t_stop;
float* i_start;
float* i_stop;

unsigned long out;
unsigned long t;
unsigned long dt;
unsigned long length;
float new_i;
float di;
float prog;
float intensity;


void next(uint8_t i){
  t = micros();
  dt = (unsigned long) random(DT_MIN * 1000000L, DT_MAX * 1000000L);
  t_start[i] = t;
  t_stop[i] = t + dt;
  
  if (i_stop[i] > i_start[i]){
    new_i = (float)random(0, (i_stop[i] - DI_MIN)*1000)/1000;
  }
  else if (i_stop[i] <= i_start[i]){
    new_i = (float)random((i_stop[i] + DI_MIN)*1000, 1000)/1000;
  }

  i_start[i] = i_stop[i];
  i_stop[i] = new_i;
}

void setup() {
  t_start = (unsigned long*) calloc(NUM_OUT, sizeof(unsigned long));
  t_stop = (unsigned long*) calloc(NUM_OUT, sizeof(unsigned long));
  i_start = (float*) calloc(NUM_OUT, sizeof(float));
  i_stop = (float*) calloc(NUM_OUT, sizeof(float));

  Serial.begin(9600);
  Serial.println("16 channel PWM test!");
  pwm.begin();
  pwm.setPWMFreq(1600);
}

void loop() {
  t = micros();
  for(uint8_t i=0; i<NUM_OUT; i++){
    length = t_stop[i] - t_start[i];
    dt = t - t_start[i];
    prog = (float)dt/(float)length; 

    if (dt >= length){
      next(i);
    }else{
      di = i_stop[i] - i_start[i];
      intensity = i_start[i] + sin(prog*PI/2) * di;
      out = (unsigned long) exp((0.9*intensity+0.1)*log(4096))-exp(0.1);
      //out = intensity*4096.0;
      //Serial.println(out);
      pwm.setPWM(i, 0, out);
    }
  }
}