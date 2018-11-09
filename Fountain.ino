#include <SimplexNoise.h>
#include <arduinoFFT.h>
#include <FastLED.h>
#include <Timer.h>

//Pins
#define PUMP_PIN          9
#define AUDIO_PIN         14      //A0
#define LED_PIN           3
#define POT_PIN           15      //A1

//Constants
#define NUM_LEDS          60
#define SAMPLES           32     //Power of 2
#define SAMPLING_FREQ     5000   //Hz, must be less than 10000 due to ADC
//64, 5000 has range [200, 2500](Hz)
#define NUM_LAYERS        3
  
//Frequencies (Hz)
#define B_MIN       10
#define B_MAX       300
#define M_MIN       300
#define M_MAX       2400
#define T_MIN       2400
#define T_MAX       9600

//Controllers
Timer tick;
CRGB leds[NUM_LEDS];
SimplexNoise low, sn;
arduinoFFT fft;

//FFT Info
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];

//Noise Info
float val = 0, nFreq = 0, nTime = 0, lTime = 0, gTime = 0;

int MIN_SPRAY;

void setup() {
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQ));
  
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);

  pinMode(3, OUTPUT);
  pinMode(POT_PIN, INPUT);

  //Calibrate min spray
  MIN_SPRAY = map(analogRead(POT_PIN), 0, 1023, 0, 255);

  Serial.begin(9600);

  //Constrain the leds [0,255]
  for(int i=0; i<NUM_LEDS; i++){
    leds[i] |= 0;
    leds[i] &= 255;
  }
}

void loop(){

  int dt = tick.getMillis();
//  Serial.println(dt);           //Display FPS
  tick.reset();
  
  //Noise
  nTime += 0.001 * dt;
  
//  int val = (sn.noise(0, nTime) + 1) * 255 / 2;
  
//  for(int i=0; i<NUM_LEDS; i++){
//    leds[i].r = ((sn.noise(i*0.5, nTime) + 1) * 100 / 2) * 0.0;
//  }
//
//  lTime += 0.0005 * dt;
////  Serial.println(val);
//  for(int i=0; i<NUM_LEDS; i++){
//    leds[i].b = ((sn.noise(i*0.1, lTime) + 1) * 255 / 2) * 0.0;
//  }

//  lTime += 0.0005 * dt;
//  for(int i=0; i<NUM_LEDS; i++){
//    leds[i].b = (pow(sn.noise(i*0.1, lTime) + 1, 4)) * 255 / (2*2*2*2) * 1.0;
//  }
//
//  gTime += 0.0005 * dt;
//  for(int i=0; i<NUM_LEDS; i++){
//    leds[i].g = (pow(sn.noise(i*1.0, gTime) + 1, 4)) * 255 / (2*2*2*2) * 1.0;
//  }

//  double val = low.noise(l, 0)*0.7 + low.noise(m, 0)*0.25 + low.noise(h, 0)*0.05;
//  Serial.println(val);

//  if(up)
//    val += dt / 3;
//  else
//    val -= dt / 3;
//
//  if(val > 255)
//    val = 255;
//  if(val < 0)
//    val = 0;
//  
//  if(val >= 255)
//    up = false;
//  else if(val <= 0)
//    up = true;
//
//  setPump(val);

//  int lb = analogRead(AUDIO_PIN);
//  Serial.println(lb);

  getFFT();
//  Serial.println(FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQ));

  //Volume
  int vol = 0;
  for(int i=0; i<SAMPLES/2; i++){     //Make from 2 to ...
    vol += vReal[i];
//    Serial.println(vReal[i], 1);
  }
//  vol = vol / SAMPLES * 2;        //Avg freq height
Serial.println(vol);
  vol = map(vol, 10000, 40000, 0, 255);
  setPump(vol);

  //LEDS
  int l, m, h;
  clearStrip();

//  setLEDs(0, 0, vol);
  
  for(int i=0; i<NUM_LEDS; i++){
//    Serial.println(vReal[i/2 +1]);
    int val = map(vReal[i/2 + 1], 0, 5000, 0, 255);
    leds[i].r = val;
    leds[i].b = val;
//    leds[i].b = val;
//    if(10<val && val<50)
//      leds[i].b = val;
//    if(50<val && val<100)
//      leds[i].g = val;
//    else if(200<val && val<255)
//      leds[i].r = val;
  }

  
//  
//  for(int i=0; i<NUM_LEDS; i++){
//    leds[i].r = map(vol, 0, 4000, 0, 255);
//  }
  
//  l = l / SAMPLES;
//  m = m / SAMPLES;
//  h = h / SAMPLES;
//
//  l = map(l, 0, 4000, 0, 255);
//  m = map(m, 0, 4000, 0, 255);
//  h = map(h, 0, 4000, 0, 255);

//  l = (l>200) ? 255 : 0;
//  m = (l>200) ? 255 : 0;
//  h = (l>240) ? 255 : 0;

//  for(int i=0; i<20; i++){
//    leds[i].r = l;
//  }
//  for(int i=20; i<40; i++){
//    leds[i].g = m;
//  }
//  for(int i=40; i<60; i++){
//    leds[i].b = h;
//  }
  
//  int in = analogRead(AUDIO_PIN);
//  Serial.print(in);
//  Serial.print(",");

//  if(count > 5){
//    avg = avg / 5;
//    count = 0;
//    lastAvg = avg;
//  } else {
//
//    avg += in;
//    count++;
//  }

//  int rel = map(in, 0, 1023, 0, 255);
//  if(rel > leds[0].b){
//    setLEDs(rel);
//  } else {
//    for(int i=0; i<NUM_LEDS; i++){
//      leds[i].fadeToBlackBy(10);
//    }
//  }
  
//  for(int i=0; i<NUM_LEDS; i++){
//    leds[i].b += pull;
//  }

  FastLED.show();
}

//void sync(){
//  LED led;
//  for(int i=0; i<NUM_LEDS; i++){
//    led = ring.getLED(i);
//    leds[i].r = led.getR();
//    leds[i].g = led.getG();
//    leds[i].b = led.getB();
//  }
//}

void getFFT(){
// Takes ~17ms with 32 samples at 10,000Hz
  
  for(int i=0; i<SAMPLES; i++){
    microseconds = micros();    //Overflows after around 70 minutes!
 
    vReal[i] = analogRead(AUDIO_PIN);
    vImag[i] = 0;
 
    while(micros() < (microseconds + sampling_period_us)){
    }
  }

  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
}

void clearStrip(){
  for(int i=0; i<NUM_LEDS; i++){
    leds[i].r = 0;
    leds[i].g = 0;
    leds[i].b = 0;
  }
}

void setLEDs(int r, int g, int b){
  for(int i=0; i<NUM_LEDS; i++){
    leds[i].r = r;
    leds[i].g = g;
    leds[i].b = b;
  }
}

void setPump(int val){
//  if(val < MIN_SPRAY)
//    val = MIN_SPRAY;

  analogWrite(PUMP_PIN, val);
}

