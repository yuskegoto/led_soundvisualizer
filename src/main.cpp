#include <Arduino.h>
#include "arduinoFFT.h"
#include "FastLED.h"
#include "math.h"

// #define RBL_NANOV2
//#define PROMICRO
#define TRINKETM0

#ifdef TRINKETM0
  #include <avdweb_AnalogReadFast.h>
#endif

#define SERIAL_MONITOR
#define LED_INDICATOR

/******************** FFT Setting ***********************/
 arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

/* These values can be changed in order to evaluate the functions */
const uint16_t SAMPLES = 128;
#ifdef RBL_NANOV2
  const double samplingFrequency = 10000; //Hz, RBL Nano v2 can theoretically catch up 200k samples per sec 
#elif defined(PROMICRO)
  const double samplingFrequency = 5000; //Hz, Theoretically Sparkfun ProMicro can only catch up 1000 samples/s due to ADC spec, however it was possible to hear above 1kHz (with 5k sps)
#elif defined(TRINKETM0)
  const double samplingFrequency = 10000; //Hz, Theoretically Sparkfun ProMicro can only catch up 1000 samples/s due to ADC spec, however it was possible to hear above 1kHz (with 5k sps)
#else
  const double samplingFrequency = 1000; //Hz, normally this should be up to 1k sps
#endif

unsigned int sampling_period_us;
unsigned long microseconds;
#define VOLUME_THRESHOLD 20
#define MAX_VOLUME 60 //max input value: 0x3FF = 1023, in decibel (root-power quantity) 20 log(1023) ~= 60. *This is not accoustic decibel!
#define LOWCUTFREQ 340 //Hz Alt recorder's lowest sound F4 is 349 Hz

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[SAMPLES];
double vImag[SAMPLES];

/******************** Pin config ***********************/
#ifdef RBL_NANOV2
  #define AUDIO_IN A4
#elif defined TRINKETM0
  #define AUDIO_IN A0
#else
  #define AUDIO_IN A0
#endif
#define LED_PIN 2

/******************** LED strip setting ***********************/

#define NUM_LEDS 8
#define LED_TYPE NEOPIXEL
#define COLOR_ORDER GRB
#define LED_MAX_BRIGHTNESS 0xFF
#define LED_MIN_BRIGHTNESS 0x64

#define LOOP_DELAY 10
CRGB leds[NUM_LEDS];
/****************************************************************/

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  #ifdef LED_INDICATOR
    pinMode(LED_BUILTIN, OUTPUT);   // set up led indication
  #endif
  
  #ifdef SERIAL_MONITOR
    Serial.begin(9600);
  #endif
  FastLED.addLeds<LED_TYPE, LED_PIN>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_MAX_BRIGHTNESS);
}

uint16_t sampling(){
  microseconds = micros();

  uint16_t maxRead = 0x1FF;
  uint16_t minRead = 0x1FF;

  for(uint16_t i=0; i<SAMPLES; i++)
  {
    #ifdef TRINKETM0
      vReal[i] = analogReadFast(AUDIO_IN);
    #else
      vReal[i] = analogRead(AUDIO_IN);
    #endif

    vImag[i] = 0;
    while(micros() - microseconds < sampling_period_us){
      //empty loop
    }
    microseconds += sampling_period_us;
  }
  // Loop for extracting max and min
  for(uint16_t i=0; i<SAMPLES; i++)
  {
    //get max and min volume
    if(vReal[i] > maxRead){maxRead = vReal[i];}
    if(vReal[i] < minRead){minRead = vReal[i];}
  }

  return maxRead - minRead;
}

uint16_t toDecibel(uint16_t vol){
  // for (uint16_t i = 0; i < SAMPLES; i ++){
  //   vReal[i] = 20 * log10(vReal[i]);
  // }
  return 20 * log10(vol);
}

void loop() {
  #ifdef LED_INDICATOR
  digitalWrite(LED_BUILTIN, HIGH);
  #endif

  #ifdef SERIAL_MONITOR
    uint32_t timeStamp = millis();
  #endif

  /*SAMPLING*/
  uint16_t volume = sampling();

  // #ifdef SERIAL_MONITOR
  //   Serial.print(volume);
  //   Serial.print(" ");
    Serial.print(millis() - timeStamp);
    Serial.print(" ");
  // #endif

  #ifdef LED_INDICATOR
  digitalWrite(LED_BUILTIN, LOW);
  #endif

  volume = toDecibel(volume);

  uint8_t color = 0;
  double f = 0;
  uint8_t ledBrightness = 0;
  uint8_t ledLength = 0;
  //avoid fft loop to turn off the led, if volume is too small
  if(volume >= VOLUME_THRESHOLD){
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); /* Compute magnitudes */
    f = FFT.MajorPeak(vReal, SAMPLES, samplingFrequency);
  
    // remap frequency to hue value
    color = map((long)f, LOWCUTFREQ, samplingFrequency / 2, 0, 0xFF);
    // color = map(0xFF - color, 0, 0xFF, 0, 0x9F) - 0x3F;//(blue at 345hz F3,1184hz to red E4)
    // color = 0xFF - color;
    // E4 red 255, F4 blue 154
  
    // invert color from red - blue to blue - red, adjust for requested band
    color = map(255 - color, 0, 0xFF, 0, 380) +26;//(blue(around 155) at 345hz F3,1184hz to red (0) E4)
    ledBrightness = map(volume, VOLUME_THRESHOLD, MAX_VOLUME, LED_MIN_BRIGHTNESS, 0xFF);
    ledLength = map(volume, VOLUME_THRESHOLD, MAX_VOLUME, 1, 8);
  }
  else{
    volume = 0;
  }

  for(uint8_t i = 0; i < NUM_LEDS; i ++){
    if(i < ledLength){
      leds[i] = CHSV(color, 0xFF, ledBrightness);
    }
    else{
      leds[i] = CHSV(color, 0xFF, 0);
    }

  }
  FastLED.show();

  #ifdef SERIAL_MONITOR
    // Serial.print(volume);
    // Serial.print(" ");
    // Serial.print(" ");
    // Serial.print(ledBrightness);
    // Serial.print(" ");
    // Serial.print(ledLength);
    // Serial.print(" ");
    Serial.print((long)f);   //Print out what frequency is the most dominant.
    Serial.print(" ");
    // Serial.print(color);
    Serial.print(millis() - timeStamp);
    Serial.println("");
  #endif

  delay(LOOP_DELAY); /* Repeat after delay */
}