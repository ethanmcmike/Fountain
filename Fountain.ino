/*
  Fountain

  Takes audio input from an auxilary cable and responds with lights and water movement. Assignment for Project Management, Design and Entrepreneurship at LeTourneau University.

  Uses Fast Fourier Transform to split audio into frequncy bins. Tracks 3 frequency ranges and their weighted average to reduce jitter. Responds with lights and water based on changes in frequency levels from their respective average.

  LED_PIN   - signal pin to individually controllable light strip
  PUMP_PIN  - controls pump power through transistor
  AUDIO_PIN - receives shifted audio signal [0,5] Volts
  POT_PIN   - potentiometer to control minimum pump power output

  Created 13 September 2018
  By Ethan McMichael
*/

#include <SimplexNoise.h>
#include <arduinoFFT.h>
#include <FastLED.h>
#include <Timer.h>

//Pins
#define LED_PIN           3
#define PUMP_PIN          9
#define AUDIO_PIN         14      //A0
#define POT_PIN           15      //A1

#define NUM_LEDS          60

//Audio constants
#define SAMPLES           32     //Must be power of 2
#define SAMPLING_FREQ     2000   //Hz, must be less than 10000 due to ADC

//Weighs to track frequency range averages
#define VOL_WEIGHT      5
#define LED_WEIGHT      3
#define PUMP_WEIGHT     5
#define BASS_WEIGHT     7
#define STATE_WEIGHT    50
#define MID_WEIGHT      7

#define TIMEOUT         1000
  
//Frequency Ranges (Hz)
#define V_MIN       2     //Volume
#define V_MAX       16
#define B_MIN       0     //Bass
#define B_MAX       3
#define M_MIN       6     //Mid-range
#define M_MAX       12
#define T_MIN       15    //Treble
#define T_MAX       16

//Controllers
Timer timer, tick, timeout, stateTimer;
CRGB leds[NUM_LEDS], history[NUM_LEDS];
SimplexNoise sn;
arduinoFFT fft;

//FFT Data
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];

//Simplex noise Data
float val = 0, nFreq = 0, nTime = 0, lTime = 0, gTime = 0;

byte state, lastState;    //0 - quiet, 1 - mid, 2 - loud
int minSpray;

//Heavy values - weighted to lag behind commands for smooth effects
int volAvg, pumpAvg, stateAvg, bassAvg, midAvg;
int lastVol;

void setup() {

  pinMode(LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  Serial.begin(9600);
  
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQ));

  //Calibrate min spray
  minSpray = map(analogRead(POT_PIN), 0, 1023, 0, 255);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);

  //Constrain FastLED [0,255]
  for(int i=0; i<NUM_LEDS; i++){
    leds[i] |= 0;
    leds[i] &= 255;
  }
}

void loop(){

  int dt = tick.getMillis();
//  Serial.println(dt);         //Display FPS
  tick.reset();

  getFFT();

  fadeAll();

  int vol = 0;
  int bass = 0;
  int mid = 0;
  
  for(int i=0; i<16; i++){
    
    if(V_MIN<i && i<V_MAX)
      vol += vReal[i];

    if(B_MIN<i && i<B_MAX)
      bass += vReal[i];

    if(M_MIN<i && i<M_MAX)
      mid += vReal[i];
  }
  
  vol = vol / (V_MAX - V_MIN);
  vol = map(vol, 0, 1500, 0, 255);
  if(vol<0) vol = 0; else if(vol>255) vol = 255;

  bass = bass / (B_MAX - B_MIN);
  bass = map(bass, 0, 4000, 0, 255);
  if(bass<0) bass = 0; else if(bass>255) bass = 255;

  mid = mid / (M_MAX - M_MIN);
  mid = map(mid, 0, 2000, 0, 255);
  if(mid<0) mid = 0; else if(mid>255) mid = 255;

  //Set averages
  volAvg += (vol - volAvg) / VOL_WEIGHT;
  pumpAvg += (vol - pumpAvg) / PUMP_WEIGHT;
  stateAvg += (vol - stateAvg) / STATE_WEIGHT;
  bassAvg += (bass - bassAvg) / BASS_WEIGHT;
  midAvg += (mid - midAvg) / MID_WEIGHT;

  setState();   //0 if no music, 1 if music, 2 if loud

  if(state == 0){
    setNoise(dt);
    setPump(0);
  }
  else{
    setAll(0, 0, map(vol, 0, 255, 0, 100));
    setPump(255);
  }

  //Mid
  setVoice(midAvg);

  //Bass
  if(bassAvg > 100 && bass > 200)
    kickBass();

  FastLED.show();
}

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

void fadeAll(){
  for(int i=0; i<NUM_LEDS; i++){
    leds[i].fadeToBlackBy(64);
  }
}

void setLED(int i, int r, int g, int b){
  if(leds[i].r < r)
    leds[i].r = r;
  if(leds[i].g < g)
    leds[i].g = g;
  if(leds[i].b < b)
    leds[i].b = b;
}

void setAll(int r, int g, int b){
  for(int i=0; i<NUM_LEDS; i++){
    setLED(i, r, g, b);
  }
}

void setPump(int val){
  val = map(val, 0, 255, minSpray, 255);
  analogWrite(PUMP_PIN, val);
}

void setNoise(int dt){
  //Displays blue wave of Simplex Noise when no music is playing
  nTime += 0.001 * dt;
  
  for(int i=0; i<NUM_LEDS; i++){
    setLED(i, 0, 0, pow((sn.noise(i*0.1, nTime) + 1), 3) / (2*2*2) * 255.0);
  }
}

void setState(){
  //Detects quiet(0), mid(1) and loud(2) volume ranges
  
  int currentState;
  if(volAvg<10)
    currentState = 0;
  else if(volAvg<200)
    currentState = 1;
  else
    currentState = 2;

  if(currentState == 0){
    
    if(state != 0 && lastState != 0){
      timeout.reset();
      
    } else if(timeout.getMillis() > TIMEOUT){
      state = 0;
    }
    
  } else {
    state = currentState;
  }

  lastState = currentState;
}

void kickBass(){
  //Flashes red on every fourth led  
  for(int i=0; i<NUM_LEDS; i+=4){
    setLED(i, 255, 0, 0);
  }
}

void setVoice(int val){
  //Displays voice volume starting from three points around the ring
  int level = map(val, 0, 255, 0, NUM_LEDS/3);

  for(int i=0; i<NUM_LEDS; i+=NUM_LEDS/3){
    for(int j=0; j<level; j++){
      setLED(i+j, 0, 255, 0);
    }
  }
}
