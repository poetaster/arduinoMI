/*
  (c) 2025 blueprint@poetaster.de
  GPLv3 the libraries are MIT as the originals for STM from MI were also MIT.

  Change the PWMOUT number to the pin you are doing pwm on.
  Works fine 133mHz -O2 on an RP2350

  BE CAREFULL, can be loud or high pitched. Or BOTH!

*/

bool debugging = true;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

#include <PWMAudio.h>
#define SAMPLERATE 48000
#define PWMOUT 22
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio

// we can refer to sample[0] since there is only one.
#define NUM_VOICES 1
struct voice_t {
  int16_t sample;   // index of the sample structure in sampledefs.h
  int16_t level;   // 0-1000 for legacy reasons
  uint32_t sampleindex; // 20:12 fixed point index into the sample array
  uint16_t sampleincrement; // 1:12 fixed point sample step for pitch changes
  bool isPlaying;  // true when sample is playing
} voice[NUM_VOICES] = {
  0, 700, 0, 4096, false, // test sample
};
#include "samples.h" // we're just using a sample for now.
#define NUM_SAMPLES (sizeof(sample)/sizeof(sample_t))


// plaits dsp
#include <STMLIB.h>
#include <CLOUDS.h>

// the following is largely from the mi-Ugens sourc MiClouds.cpp
#include "clouds/dsp/granular_processor.h"
#include "clouds/resources.h"
#include "clouds/dsp/audio_buffer.h"
#include "clouds/dsp/mu_law.h"
#include "clouds/dsp/sample_rate_converter.h"



const uint16_t kAudioBlockSize = 32;        // sig vs can't be smaller than this!
const uint16_t kNumArgs = 14;


enum ModParams {
  PARAM_PITCH,
  PARAM_POSITION,
  PARAM_SIZE,
  PARAM_DENSITY,
  PARAM_TEXTURE,
  PARAM_DRYWET,
  PARAM_CHANNEL_LAST
};

// main output audio buffer
int16_t out_bufferL[32];
int16_t out_bufferR[32];

int16_t sample_buffer[32]; // used while we play samples for demo


struct Unit {

  clouds::GranularProcessor   *processor;

  // buffers
  //uint8_t     *large_buffer;
  //uint8_t     *small_buffer;
  // Pre-allocate big blocks in main memory and CCM. No malloc here.
  uint8_t     large_buffer[118784];
  uint8_t     small_buffer[65536 - 128];

  // parameters
  float       in_gain;
  bool        freeze;
  bool        trigger, previous_trig;
  bool        gate;

  float       pot_value_[PARAM_CHANNEL_LAST];
  float       smoothed_value_[PARAM_CHANNEL_LAST];
  float       coef;      // smoothing coefficient for parameter changes

  clouds::FloatFrame  input[kAudioBlockSize];
  clouds::FloatFrame  output[kAudioBlockSize];

  float       sr;
  long        sigvs;

  bool        gate_connected;
  bool        trig_connected;
  uint32_t      pcount;


  clouds::SampleRateConverter < -clouds::kDownsamplingFactor, 45, clouds::src_filter_1x_2_45 > src_down_;
  clouds::SampleRateConverter < +clouds::kDownsamplingFactor, 45, clouds::src_filter_1x_2_45 > src_up_;

};

struct Unit cloud[1];

// Plaits modulation vars
float morph_in = 0.1f; // IN(4);
float trigger_in = 0.0f; //IN(5);
float level_in = 0.5f; //IN(6);
float harm_in = 0.1f;
float timbre_in = 0.1f;
int engine_in;

float fm_mod = 0.f ; //IN(7);
float timb_mod = 0.f; //IN(8);
float morph_mod = 0.f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.2f ;// IN(11);
float pitch_in = 60.f;
float octave_in = 3.f;
float dw_in = 0.5f;

int engineCount = 0;
int engineInc = 0;

// clock timer  stuff

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

//unsigned int SWPin = CLOCKIN;

#define TIMER0_INTERVAL_MS  20.833333333333
//24.390243902439025 // 44.1
// \20.833333333333running at 48Khz

#define DEBOUNCING_INTERVAL_MS   2// 80
#define LOCAL_DEBUG              0

volatile int counter = 0;

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;
  if ( DAC.availableForWrite()) {
    for (size_t i = 0; i < kAudioBlockSize; i++) {
      DAC.write( out_bufferL[i] );
    }
    counter = 1;
  }

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
    Serial.begin(115200);
    Serial.println(F("YUP"));
  }
  // pwm timing setup
  // we're using a pseudo interrupt for the render callback since internal dac callbacks crash
  // comment the following block out and uncomment the DAC.onTransmit(cb) line to use the DAC callback method

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
  // start playing sample
  voice[0].sampleindex = 0; // trigger sample for this track
  voice[0].isPlaying = true;

  fillSampleBuffer();
  // init the clouds voice(s)
  initVoices();
  updateCloudsAudio();

}


void initVoices() {

  int largeBufSize = 118784;
  int smallBufSize = 65536 - 128;

  // we fixed it with static inits in the unit

  //cloud[0].large_buffer = (uint8_t*)malloc(largeBufSize * sizeof(uint8_t));
  //cloud[0].small_buffer = (uint8_t*)malloc(smallBufSize * sizeof(uint8_t));

  cloud[0].sr = SAMPLERATE;
  cloud[0].processor = new clouds::GranularProcessor;
  memset(cloud[0].processor, 0, sizeof(*cloud[0].processor));

  cloud[0].processor->Init(cloud[0].large_buffer, largeBufSize, cloud[0].small_buffer, smallBufSize);
  cloud[0].processor->set_sample_rate(cloud[0].sr);
  cloud[0].processor->set_num_channels(2);       // always use stereo setup
  cloud[0].processor->set_low_fidelity(false);
  cloud[0].processor->set_playback_mode(clouds::PLAYBACK_MODE_GRANULAR);

  // init values
  cloud[0].pot_value_[PARAM_PITCH] = cloud[0].smoothed_value_[PARAM_PITCH] = 0.f;
  cloud[0].pot_value_[PARAM_POSITION] = cloud[0].smoothed_value_[PARAM_POSITION] = 0.f;
  cloud[0].pot_value_[PARAM_SIZE] = cloud[0].smoothed_value_[PARAM_SIZE] = 0.5f;
  cloud[0].pot_value_[PARAM_DENSITY] = cloud[0].smoothed_value_[PARAM_DENSITY] = 0.1f;
  cloud[0].pot_value_[PARAM_TEXTURE] = cloud[0].smoothed_value_[PARAM_TEXTURE] = 0.5f;
  cloud[0].pot_value_[PARAM_DRYWET] = cloud[0].smoothed_value_[PARAM_DRYWET] = 1.f;
  cloud[0].processor->mutable_parameters()->stereo_spread = 0.5f;
  cloud[0].processor->mutable_parameters()->reverb = 0.f;
  cloud[0].processor->mutable_parameters()->feedback = 0.f;
  cloud[0].processor->mutable_parameters()->freeze = false;
  cloud[0].in_gain = 1.0f;
  cloud[0].coef = 0.1f;
  cloud[0].previous_trig = false;
  cloud[0].src_down_.Init();
  cloud[0].src_up_.Init();
  cloud[0].pcount = 0;
  // have to think about this :)
  //uint16_t numAudioInputs = cloud[0].mNumInputs - kNumArgs;
  Serial.println(F("INIT DONE"));



}

// main audio called from loop, cpu 2)
void updateCloudsAudio() {
  /*
    constrain(in_gain, 0.125f, 8.f);
    constrain(spread, 0.f, 1.f);
    constrain(reverb, 0.f, 1.f);
    constrain(fb, 0.f, 1.f);
    constrain(mode, 0, 3);
  */
  //  MiClouds.ar(input, pit: -15.0, pos: 0.5, size: 0.25,  dens: dens, tex: 0.5, drywet: 1, mode: 0);

  float   pitch =  constrain(pitch_in, -48.0f, 48.0f);
  float   in_gain = 1.0f; // harm_in; //IN0(6);
  float   spread = 0.5f;// IN0(7);
  float   reverb = 0.5f; // IN0(8);
  float   fb =  0.4f; // IN0(9);
  float   siz = constrain(harm_in, 0.f, 0.7f) ;// 0.35f;
  float   dens = constrain( morph_in, 0.f, 0.7f);;
  float   tex = constrain (timbre_in, 0.f, 0.8f) ;
  float   drywet = constrain(dw_in, 0.3f, 0.8f);
  bool    freeze = false; // IN0(10) > 0.f;
  short   mode = 0; // 0 -3
  bool    lofi = 1; // IN0(12) > 0.f;

  cloud[0].pot_value_[PARAM_DRYWET] = cloud[0].smoothed_value_[PARAM_DRYWET] = drywet;
  
  int vs = 32; //inNumSamples; // hmmmm

  // find out number of audio inputs
  //uint16_t numAudioInputs = cloud[0].mNumInputs - kNumArgs;

  // if (numAudioInputs == 1)
  //   Copy(inNumSamples, IN(kNumArgs + 1), IN(kNumArgs));

  clouds::FloatFrame  *input = cloud[0].input;
  clouds::FloatFrame  *output = cloud[0].output;

  float       *smoothed_value = cloud[0].smoothed_value_;
  float       coef = cloud[0].coef;
  clouds::GranularProcessor   *gp = cloud[0].processor;
  clouds::Parameters   *p = gp->mutable_parameters();

  smoothed_value[PARAM_PITCH] += coef * (pitch - smoothed_value[PARAM_PITCH]);
  p->pitch =  smoothed_value[PARAM_PITCH];

  for (int i = 1; i < PARAM_CHANNEL_LAST; ++i) {
    float value = 0.5f; // 0.0f; //IN0(i);
    value = constrain(value, 0.0f, 1.0f);

    smoothed_value[i] += coef * (value - smoothed_value[i]);
  }
  p->position = smoothed_value[PARAM_POSITION];

  smoothed_value[PARAM_SIZE] += coef * (siz - smoothed_value[PARAM_SIZE]);
  p->size = smoothed_value[PARAM_SIZE];

  smoothed_value[PARAM_DENSITY] += coef * (dens - smoothed_value[PARAM_DENSITY]);
  p->density = smoothed_value[PARAM_DENSITY];


  smoothed_value[PARAM_TEXTURE] += coef * (tex - smoothed_value[PARAM_TEXTURE]);
  p->texture = smoothed_value[PARAM_TEXTURE];

  p->dry_wet = smoothed_value[PARAM_DRYWET];
  p->stereo_spread = spread;
  p->reverb = reverb;
  p->feedback = fb;
  gp->set_low_fidelity(lofi);
  gp->set_freeze(freeze);
  gp->set_playback_mode(static_cast<clouds::PlaybackMode>(mode));

  // uint16_t trig_rate = INRATE(13); // A non-positive to positive transition causes a trigger to happen.

  for (int count = 0; count < 1; count += kAudioBlockSize) {

    for (int i = 0; i < kAudioBlockSize; ++i) {
      /*
        input[i].l = IN(kNumArgs)[i + count] * in_gain;
        input[i].r = IN(kNumArgs + 1)[i + count] * in_gain;
      */
      input[i].l = (float) sample_buffer[i + count] / 32768.0f; // grain is ko.
      input[i].r = input[i].l;
    }

    bool trigger = false;
    if (trigger_in == 1.0f) {
      trigger = true;
    }
    /*
      switch (trig_rate) {
      case 1 :
        trigger = ( trig_in[0] > 0.f );
        break;
      case 2 :
        float sum = 0.f;
        for (int i = 0; i < kAudioBlockSize; ++i) {
          sum += ( trig_in[i + count] );
        }
        trigger = ( sum > 0.f );
        break;
      }
    */

    p->trigger = (trigger && !cloud[0].previous_trig);
    cloud[0].previous_trig = trigger;

    gp->Process(input, output, kAudioBlockSize);
    gp->Prepare();      // why here?

    if (p->trigger)
      p->trigger = false;

    for (int i = 0; i < kAudioBlockSize; ++i) {
      out_bufferL[i] = stmlib::Clip16(static_cast<int32_t>( (output[i].l )  * 32768.0f) ); // in rings we had gain?
      //output[i].l; // we stick to mono since we can't test stereo :)
      //out_bufferR[i + count] = output[i].r;
    }
  }

}

void fillSampleBuffer() {
  // start / resume playing sample looping
  if (! voice[0].isPlaying) {
    voice[0].sampleindex = 0; // trigger sample for this track
    voice[0].isPlaying = true;
  }

  int32_t newsample, samplesum = 0, filtersum;
  uint32_t index;
  int16_t samp0, samp1, delta, tracksample;
  tracksample = voice[0].sample; // precompute for a little more speed below

  for (int i = 0; i < kAudioBlockSize; ++i) {

    index = voice[0].sampleindex >> 12; // get the integer part of the sample increment
    if (index >= sample[tracksample].samplesize) {
      voice[0].isPlaying = false; // have we played the whole sample?
    }
    if (voice[0].isPlaying) { // if sample is still playing, do interpolation
      samp0 = sample[tracksample].samplearray[index]; // get the first sample to interpolate
      voice[0].sampleindex += voice[0].sampleincrement; // add step increment
    }
    sample_buffer[i] = constrain( samp0, -32767, 32767); // apply clipping
    // out_bufferL[i] =sample_buffer[i]; // testing, works
  }


}

void loop() {
  if ( counter == 1 ) {
    fillSampleBuffer();
    updateCloudsAudio();
    counter = 0; // increments on each pass of the timer after the timer writes samples
  }
}

// second core dedicated to display foo

void setup1() {
  delay (200); // wait for main core to start up perhipherals
}


// second core deals with ui / control rate updates
void loop1() {


  float trigger = randomDouble(0.0, 1.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
  float harmonics = randomDouble(0.0, 0.8); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
  float timbre = randomDouble(0.0, 0.8); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
  float morph = randomDouble(0.1, 0.8) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
  float pitch = randomDouble(-48, 48); // TIRand.kr(24, 48, trigger);
  float drywet = randomDouble(0.2, 0.9);
  float octave = randomDouble(0.2, 0.4);
  float decay = randomDouble(0.1, 0.4);

  if (trigger > 0.1f) {
    trigger_in = 1.0f;
  } else {
    trigger_in = 0.0f;
  }

  //octave_in = octave;
  pitch_in = pitch;
  harm_in = harmonics;
  morph_in = morph;
  timbre_in = timbre;
  dw_in = drywet;
  
  engineInc++ ;
  if (engineInc > 4) {
    engineCount ++; // don't switch engine so often :)
    engineInc = 0;
    //voices[0].patch.engine = engineCount;
  }
  if (engineCount > 15) engineCount = 0;

  delay(3000);

}
