#include <Arduino.h>
#include "arduinoFFT.h"
#include "FastLED.h"

// #define RBL_NANOV2
#define PROMICRO

#define SERIAL_MONITOR

/******************** FFT Setting ***********************/
 arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

/* These values can be changed in order to evaluate the functions */
const uint16_t samples = 128;
#ifdef RBL_NANOV2
  const double samplingFrequency = 10000; //Hz, RBL Nano v2 can theoretically catch up 200k samples per sec 
#elif defined(PROMICRO)
  const double samplingFrequency = 4000; //Hz, Theoretically Sparkfun ProMicro can only catch up 1000 samples/s due to ADC spec, however it was possible to hear above 1kHz (with 5k sps)
#else
  const double samplingFrequency = 1000; //Hz, normally this should be up to 1k sps
#endif
unsigned int sampling_period_us;
unsigned long microseconds;
#define VOLUME_THRESHOLD 30

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
#ifdef RBL_NANOV2
  #define AUDIO_IN A4
#else
  #define AUDIO_IN A0
#endif
/******************** LED strip setting ***********************/
#define LED_PIN 2
#define NUM_LEDS 1
#define BRIGHTNESS 10
#define LED_TYPE NEOPIXEL
#define COLOR_ORDER GRB

#define LOOP_DELAY 10
CRGB leds[NUM_LEDS];
/****************************************************************/

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  pinMode(LED_BUILTIN, OUTPUT);   // set up led indication
  #ifdef SERIAL_MONITOR
    Serial.begin(9600);
  #endif
  FastLED.addLeds<LED_TYPE, LED_PIN>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
}

uint16_t sampling(){
  microseconds = micros();

  uint16_t maxRead = 0x1FF;
  uint16_t minRead = 0x1FF;

  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(AUDIO_IN);
      //get max and min volume
      if(vReal[i] > maxRead){maxRead = vReal[i];}
      if(vReal[i] < minRead){minRead = vReal[i];}
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  return maxRead - minRead;
}

void loop() {

  digitalWrite(LED_BUILTIN, HIGH);

  #ifdef SERIAL_MONITOR
    uint32_t timeStamp = millis();
  #endif

  /*SAMPLING*/
  uint16_t volume = sampling();

  #ifdef SERIAL_MONITOR
    Serial.print(volume);
    Serial.print(" ");
    Serial.print(millis() - timeStamp);
    Serial.print(" ");
  #endif
  digitalWrite(LED_BUILTIN, LOW);

  uint8_t color = 0;
  double x = 0;
  uint8_t ledBrightness = 0;
  //avoid fft loop to turn off the led, if volume is too small
  if(volume >= VOLUME_THRESHOLD){
    // FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    // FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    x = FFT.MajorPeak(vReal, samples, samplingFrequency);
    // remap frequency to hue value
    color = map((long)x, 0, samplingFrequency / 2, 0, 0xFF);
    ledBrightness = 0xFF;
  }
  leds[0] = CHSV(color, 0xFF, ledBrightness);
  FastLED.show();

  #ifdef SERIAL_MONITOR
    Serial.print((long)x);   //Print out what frequency is the most dominant.
    Serial.print(" ");
    Serial.print(color); //Print out what frequency is the most dominant.
    Serial.print(" ");
    Serial.println(millis() - timeStamp);
  #endif
  delay(LOOP_DELAY); /* Repeat after delay */
}