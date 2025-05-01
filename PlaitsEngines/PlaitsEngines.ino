/*
  (c) 2025 blueprint@poetaster.de
  GPLv3 the libraries are MIT as the originals for STM from MI were also MIT.
*/

bool debugging = true;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <hardware/pwm.h>

#include <PWMAudio.h>
#define SAMPLERATE 48000
#define PWMOUT 22
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio


// plaits dsp
#include <STMLIB.h>
#include <PLAITS.h>

//using namespace plaits;
//using namespace stm_audio_bootloader;
//using namespace stmlib;

plaits::Modulations modulations;
plaits::Patch patch;
plaits::Voice voice;


char shared_buffer[16384];
stmlib::BufferAllocator allocator;

//float a0 = (440.0 / 8.0) / kSampleRate; //48000.00;
const size_t   kBlockSize = plaits::kBlockSize;

plaits::Voice::Frame outputPlaits[ plaits::kBlockSize];
//plaits::Voice::out_buffer renderBuffer;

struct Unit {
  plaits::Voice       *voice_;
  plaits::Modulations modulations;
  plaits::Patch       patch;
  float               transposition_;
  float               octave_;
  short               trigger_connected;
  short               trigger_toggle;

  char                *shared_buffer;
  void                *info_out;
  bool                prev_trig;
  float               sr;
  int                 sigvs;
};

struct Unit voices[1];

// Plaits modulation vars
float morph_in; // IN(4);
float trig_in = 0.0f; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.1f;
float timbre_in = 0.1f;
int engine_in;

float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.1f; //IN(8);
float morph_mod = 0.1f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpgColor_in = 0.5f ;// IN(11);
float pitch = 60.0f;

// clock timer  stuff which sort of works :)

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

//unsigned int SWPin = CLOCKIN;

#define TIMER0_INTERVAL_MS 20.833333333333   // running at 48Khz
#define DEBOUNCING_INTERVAL_MS   2// 80
#define LOCAL_DEBUG              1

volatile int counter = 0; // this is used to get the loop to update sample buffer

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;
  if ( DAC.availableForWrite() > 1 ) DAC.write( outputPlaits[counter].out, sync ); // offset for volume
  /*
  for (size_t i = 0; i < 16; i++) {
    if ( DAC.availableForWrite() ) DAC.write( outputPlaits[i].out + 244, sync ); // 244 is mozzi audio bias
  }*/
  counter++;
  return true;
}


// produce some random numbers in ranges.
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

// audio related defines

//float freqs[12] = { 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f, 493.88f};
float freqs[12] = { 42.f, 44.f, 46.f, 48.f, 49.f, 50.f, 51.f, 53.f, 54.f, 55.f, 56.f, 57.f};
int carrier_freq;

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
  DAC.setBuffers(4, 128); // DMA buffers
  //DAC.onTransmit(cb); // sadly, this approach blows up?
  DAC.setFrequency(SAMPLERATE);
  DAC.begin();

  pinMode(23, OUTPUT); //  switch to PWM for power to avoid ripple noise
  digitalWrite(23, HIGH);

  
  // init the Plaits engines
  initVoices();
  // fill buffer
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);
}


void initVoices() {

  // init some params
  voices[0] = {};
  voices[0].modulations = modulations;
  voices[0].modulations.engine = 0;
  voices[0].patch = patch;
  voices[0].patch.engine = 0;
  voices[0].transposition_ = 0.;
  voices[0].octave_ = 0.5;
  voices[0].patch.note = 48.0;
  voices[0].patch.harmonics = 0.1;

  stmlib::BufferAllocator allocator(shared_buffer, 16384);
  voice.Init(&allocator);

  voices[0].voice_ = &voice;
  voices[0].shared_buffer = shared_buffer;

  memset(&voices[0].patch, 0, sizeof(voices[0].patch));
  memset(&voices[0].modulations, 0, sizeof(voices[0].modulations));

  // start with no CV input
  voices[0].prev_trig = false;
  voices[0].modulations.timbre_patched = false;  //(INRATE(3) != calc_ScalarRate);
  voices[0].modulations.morph_patched = false;   // (INRATE(4) != calc_ScalarRate);
  voices[0].modulations.trigger_patched = false; //(INRATE(5) != calc_ScalarRate);
  voices[0].modulations.level_patched = false;   // (INRATE(6) != calc_ScalarRate);
  // TODO: we don't have an fm input yet.
  voices[0].modulations.frequency_patched = false;

}

/*
   does not work // neither did mozzi pwm or I2S
  void cb() {
  if (DAC.availableForWrite() > 16) {
    voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);
    for (int i = 0; i <  plaits::kBlockSize; i++) {
      uint16_t out = outputPlaits[i].out;   // left channel called .aux
      DAC.write( out );
    }
  }
  }
*/

bool canBufferAudioOutput() {
  if ( DAC.availableForWrite() > 32 ) {
    return true;
  }
  return false;
}

void loop() {

  delay(2); //
  
  if ( counter > 15 ) {
    voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);
    counter = 0; // increments on each pass of the timer after the timer writes samples
  }

}

// second core dedicated to display foo

void setup1() {
  delay (1000); // wait for main core to start up perhipherals
}

int engineCount = 0;

// second core deals with ui / control rate updates
void loop1() {

  // here we get modulations which applied every 4 seconds to each engine in order. 
  // the engine may live longer than 4 seconds :)
  float trigger = randomDouble(0.01, 1.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
  float harmonics = randomDouble(0.1, 1.0); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
  float timbre = randomDouble(0.1, 1.0); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
  float morph = randomDouble(0.1, 0.7) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
  float pitch = randomDouble(24, 48); // TIRand.kr(24, 48, trigger);
  float engine = randomDouble(0, 15); //TRand.kr(0, 15, trig: trigger).round;
  float octave = randomDouble(0.2, 0.5);
  float decay = randomDouble(0.01, 0.8);

  voices[0].patch.engine = engineCount;
  //voices[0].transposition_ = 0.;
  voices[0].octave_ = octave;
  voices[0].patch.note = pitch;
  voices[0].patch.harmonics = harmonics;
  voices[0].patch.morph = morph;
  voices[0].patch.timbre = timbre;
  voices[0].patch.decay = decay;
  voices[0].patch.lpg_colour = 0.2;
  
  if (trigger > 0.3 ) {
    engineCount ++; // don't switch engine so often :)
    voices[0].modulations.trigger = trigger;
    voices[0].modulations.trigger_patched = true;
  } else {
    
    voices[0].modulations.trigger = 0.0f;
    voices[0].modulations.trigger_patched = false;
  }


  if (engineCount > 15) engineCount = 0;

  delay(4000);


}
