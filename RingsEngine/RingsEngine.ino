/*
  (c) 2025 blueprint@poetaster.de GPLv3

   Based on  mi_Ugens, Copyright (c) 2020 Volker Böhm. All rights reserved. GPLv3
   https://vboehm.net

   Mutable Instruments sources, including the stmlib, tides, plaits elements and braids libs are
   MIT License
   Copyright (c)  2020 (emilie.o.gillet@gmail.com)
*/

bool debugging = true;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

#include <PWMAudio.h>
#define SAMPLERATE 48000
#define PWMOUT 22
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio

// rings dsp

#include <STMLIB.h>
#include <RINGS.h>

const size_t kBlockSize = rings::kMaxBlockSize;

float rings::Dsp::sr = 48000.0f;
float rings::Dsp::a3 = 440.0f / 48000.0f;


struct Unit {
  rings::Part             part;
  rings::StringSynthPart  string_synth;
  rings::Strummer         strummer;
  rings::PerformanceState performance_state;
  rings::Patch            patch;

  uint16_t                *reverb_buffer;

  float                   *silence;
  float                   *out, *aux;     // output buffers
  float                   *input; // input buffer

  bool                    prev_trig;
  int                     prev_poly;

  // we're using the original rendering, not vbs
  int16_t                 obuff[kBlockSize]; // from float[]s below in update routine
  int16_t                 abuff[kBlockSize]; // ditto
};

struct Unit voices[1];

// Plaits modulation vars
float morph_in = 0.7f; // IN(4);
float trigger_in; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.1f;
float timbre_in = 0.1f;
int engine_in;
bool easterEgg;

float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.0f; //IN(8);
float morph_mod = 0.0f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.1f ;// IN(11);
int pitch_in = 60;


// Braids vars
//    float   voct_in = IN0(0);
//    float   timbre_in = IN0(1);
//    float   color_in = IN0(2);
//    float   model_in = IN0(3);
//    float   *trig_in = IN(4);

// clock timer  stuff

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

//unsigned int SWPin = CLOCKIN;

#define TIMER0_INTERVAL_MS 20.833333333333   // \20.833333333333running at 48Khz
#define DEBOUNCING_INTERVAL_MS   2// 80
#define LOCAL_DEBUG              0

volatile int counter = 0;

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;

  if ( DAC.availableForWrite() ) {
    for (size_t i = 0; i < kBlockSize; i++) {
      DAC.write( voices[0].obuff[i]);
    }
    counter =  1;
  }
  return true;
}


// utility
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

// audio related defines

//float freqs[12] = { 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f, 493.88f};
int freqs[12] = { 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64};


void setup() {
  if (debugging) {
    Serial.begin(57600);
    Serial.println(F("YUP"));
  }
  // pwm timing setup
  // we're using a pseudo interrupt for the render callback since internal dac callbacks crash
  // Frequency in float Hz
  //ITimer0.attachInterrupt(TIMER_FREQ_HZ, TimerHandler0);
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS, TimerHandler0)) // that's 48kHz
  {
    if (debugging) Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
  }  else {
    if (debugging) Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }

  // set up Pico PWM audio output
  DAC.setBuffers(4, 32); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  DAC.begin();

  // thi is to switch to PWM for power to avoid ripple noise
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  // init the rings voice
  initVoices();

}

// initialize voice parameters
void initVoices() {

  rings::Dsp::setSr(SAMPLERATE);
  // allocate memory + init with zeros

  voices[0].reverb_buffer = (uint16_t*)malloc(32768 * sizeof(uint16_t));
  memset(voices[0].reverb_buffer, 0, 32768 * sizeof(uint16_t));

  voices[0].silence = (float*)malloc(kBlockSize * sizeof(float));
  memset(voices[0].silence, 0, kBlockSize * sizeof(float));

  voices[0].input = (float*)malloc(kBlockSize * sizeof(float));
  memset(voices[0].input, 0, kBlockSize * sizeof(float));

  voices[0].out = (float *)malloc(kBlockSize * sizeof(float));
  voices[0].aux = (float *)malloc(kBlockSize * sizeof(float));

  // zero out...
  memset(&voices[0].strummer, 0, sizeof(voices[0].strummer));
  memset(&voices[0].part, 0, sizeof(voices[0].part));
  memset(&voices[0].string_synth, 0, sizeof(voices[0].string_synth));

  voices[0].strummer.Init(0.01, 48000.0f / kBlockSize);
  voices[0].part.Init(voices[0].reverb_buffer);
  voices[0].string_synth.Init(voices[0].reverb_buffer);

  voices[0].part.set_polyphony(1);
  voices[0].part.set_model(rings::RESONATOR_MODEL_MODAL);

  voices[0].string_synth.set_polyphony(1);
  voices[0].string_synth.set_fx(rings::FX_FORMANT);
  voices[0].prev_poly = 0;

  voices[0].performance_state.fm = 0.f;       // TODO: fm not used, maybe later...
  voices[0].prev_trig = false;

  // we're fixing this for the moment until we can test with input
  voices[0].performance_state.internal_exciter = true;

  // let's see
  voices[0].performance_state.internal_strum = true;
  updateRingsAudio() ;

  // check input rates
  /*
    if(INRATE(0) == calc_FullRate)
      voices[0].performance_state.internal_exciter = false;
    else
      voices[0].performance_state.internal_exciter = true;

    if(INRATE(1) == calc_ScalarRate)
      voices[0].performance_state.internal_strum = true;
    else
      voices[0].performance_state.internal_strum = false;

    if(INRATE(2) == calc_ScalarRate)
      voices[0].performance_state.internal_note = true;
    else
      voices[0].performance_state.internal_note = false;
  */

}

void cb() {
  if (DAC.availableForWrite() > 14) {
    for (int i = 0; i <  kBlockSize; i++) {
      // out = ;   // left channel called .aux
      DAC.write( voices[0].obuff[i] );
    }
  }
}


void updateRingsAudio() {

  // our output buffers, post rendering
  int16_t *obuff = voices[0].obuff;
  int16_t *abuff = voices[0].abuff;

  float   *trig_in; // = IN(1);

  float   voct_in = pitch_in * 1.0f; // IN0(2);
  
  //Serial.println(voct_in);
  
  float   struct_in = harm_in; // IN0(3);
  float   bright_in = timbre_in; // IN0(4);

  float   damp_in = morph_in; // IN0(5);
  float   pos_in = 0.25f ; //IN0(6);

  short   model = engine_in; // IN0(7);
  short   polyphony = 4; // IN0(8);
  bool    intern_exciter = true; // (IN0(9) > 0.f);
  bool    easter_egg = easterEgg; // (IN0(10) > 0.f);
  bool    bypass = false; // (IN0(11) > 0.f);

  //float *out1 = OUT(0);
  //float *out2 = OUT(1);

  rings::Patch *patch = &voices[0].patch;
  rings::PerformanceState *ps = &voices[0].performance_state;

  float   *in = voices[0].input;
  float   *out = voices[0].out;
  float   *aux = voices[0].aux;
  size_t  size = kBlockSize;


  // check input rates for excitation input
  /* we'll get to this later
    if ((INRATE(0) == calc_FullRate) {
    std::copy(&in[0], &in[inNumSamples], &input[0]);
    // intern_exciter should be off, but user can override
    ps->internal_exciter = intern_exciter;
    }
    else {
    // if there's no audio input, set input to zero...
    input = voices[0].silence;
    // ... and use internal exciter!
    ps->internal_exciter = true;
    }
  */
  in = voices[0].silence;

  // ... and use internal exciter!
  ps->internal_exciter = true;

  // set resonator model
  CONSTRAIN(model, 0, 5);
  voices[0].part.set_model(static_cast<rings::ResonatorModel>(model));
  voices[0].string_synth.set_fx(static_cast<rings::FxType>(model));

  // set polyphony
  if (polyphony != voices[0].prev_poly) {
    CONSTRAIN(polyphony, 1, 4);
    voices[0].part.set_polyphony(polyphony);
    voices[0].string_synth.set_polyphony(polyphony);
    voices[0].prev_poly = polyphony;
  }

  // set pitch
  CONSTRAIN(voct_in, 0.f, 114.f);
  ps->tonic = 12.f;
  ps->note = voct_in;


  // set params
  CONSTRAIN(struct_in, 0.0f, 1.0f);    //0.9995f
  patch->structure = struct_in;

  float chord = struct_in * (rings::kNumChords - 1);
  voices[0].performance_state.chord = roundf(chord);

  CONSTRAIN(bright_in, 0.0f, 1.0f);
  patch->brightness = bright_in;

  CONSTRAIN(damp_in, 0.0f, 1.0f);
  patch->damping = damp_in;

  CONSTRAIN(pos_in, 0.0f, 1.0f);
  patch->position = pos_in;

  // check trigger input
  if (!ps->internal_strum) {

    bool trig = false;
    bool prev_trig = voices[0].prev_trig;
    float sum = 0.f;
    /*
      if (INRATE(1) == calc_FullRate) {   // trigger input is audio rate
      // TODO: use vDSP for the summation
      for (int i = 0; i < inNumSamples; ++i)
        sum += trig_in[i];
      trig = (sum > 0.f);
      }
      else {          // trigger input is control or scalar rate
      trig = (trig_in[0] > 0.f);
      }*/
    trig = (trigger_in > 0.f);

    if (trig) {
      if (!prev_trig)
        ps->strum = true;
      else
        ps->strum = false;
    }
    voices[0].prev_trig = trig;

  }

  voices[0].part.set_bypass(bypass);

  if (easter_egg) {
    // vbs
    /*for(int count=0; count<inNumSamples; count+=size) {

        voices[0].strummer.Process(NULL, size, ps);
        voices[0].string_synth.Process(*ps, *patch,
                               input+count, out1+count, out2+count, size);
      }*/
    /* ignore input
        for (size_t i = 0; i < size; ++i) {
       in[i] = static_cast<float>(input[i].r) / 32768.0f;
      }
    */
    voices[0].strummer.Process(NULL, size, ps);
    voices[0].string_synth.Process(*ps, *patch, in, out, aux, size);
  }
  else {

    /* vbs
      for (int count = 0; count < inNumSamples; count += size) {
      voices[0].strummer.Process(input + count, size, ps);
      voices[0].part.Process(*ps, *patch,
                         input + count, out1 + count, out2 + count, size);
      }*/
    /* ignore input
      for (size_t i = 0; i < size; ++i) {
      float in_sample = static_cast<float>(input[i].r) / 32768.0f;
      float error, gain;
      error = in_sample * in_sample - in_level;
      in_level += error * (error > 0.0f ? 0.1f : 0.0001f);
      gain = in_level <= kNoiseGateThreshold
             ? (1.0f / kNoiseGateThreshold) * in_level : 1.0f;
      in[i] = gain * in_sample;
      }*/
    voices[0].strummer.Process(in, size, ps);
    voices[0].part.Process(*ps, *patch, in, out, aux, size);


  }
  for (size_t i = 0; i < size; ++i) {
    obuff[i] = stmlib::Clip16(static_cast<int16_t>(out[i] * 32768.0f));
    //abuff[i] = stmlib::Clip16(static_cast<int16_t>(aux[i] * 32768.0f));

  }


}

void loop() {
  // when the osc buffer has been written to PWM buffer
  if ( counter > 0 ) {
    updateRingsAudio();
    counter = 0; // increments on each pass of the timer when the timer writes
  }


}
void setup1() {
  delay (200); // wait for main core to start up perhipherals
}
// second core dedicated to display foo

int engineCount = 0;
int engineInc = 0;

// second core deals with ui / control rate updates
void loop1() {

  float trigger = randomDouble(0.0, 1.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
  float harmonics = randomDouble(0.1, 0.9); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
  float timbre = randomDouble(0.1, 0.9); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
  float morph = randomDouble(0.1, 0.9) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
  float pitch = randomDouble(42, 64); // TIRand.kr(24, 48, trigger);
  float octave = randomDouble(0.2, 0.4);
  float decay = randomDouble(0.1, 0.4);
  float egg = randomDouble(0, 10);
  //octave_in = octave;

  if (egg < 2)  { easterEgg = true; } else {easterEgg = false; }
 
  pitch_in = pitch;
  harm_in = harmonics;
  morph_in = morph;
  timbre_in = timbre;
  
  if (trigger < 0.1) {
    trigger_in = 0.0f;
  } else {
    trigger_in = trigger;
  }
  
  engineInc++ ;
  if (engineInc > 8) {
    engineCount ++; // don't switch engine so often :)
    engineInc = 0;
    engine_in = engineCount;
  }
  
  if (engineCount > 5) engineCount = 0;

  delay(1300);
}
