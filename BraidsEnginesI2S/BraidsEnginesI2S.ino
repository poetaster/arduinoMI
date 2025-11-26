/*
  (c) 2025 blueprint@poetaster.de
  GPLv3 the libraries are MIT as the originals for STM from MI were also MIT.
*/

bool debugging = true;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

#include <I2S.h>
#define SAMPLERATE 48000
#define PWMOUT 22
#define MOZZI_AUDIO_BITS 16
#define BCK 20
#define WS (BCK+1) // CANNOT BE CHANGED, HAS TO BE NEXT TO pBCLK, i.e. default is 21
#define DATA 22

I2S DAC(OUTPUT);  // 16 bit PWM audio

// braids dsp

//const uint16_t decimation_factors[] = { 1, 2, 3, 4, 6, 12, 24 };
const uint16_t bit_reduction_masks[] = {
  0xffff,
  0xfff0,
  0xff00,
  0xf800,
  0xf000,
  0xe000,
  0xc000
};

#define     MI_SAMPLERATE      96000.f
#define     BLOCK_SIZE          32      // --> macro_oscillator.h !
#define     SAMP_SCALE          (float)(1.0 / 32756.0)


#include <STMLIB.h>
#include <BRAIDS.h>

typedef struct
{
  braids::MacroOscillator *osc;

  float       samps[BLOCK_SIZE] ;
  int16_t     buffer[BLOCK_SIZE];
  uint8_t     sync_buffer[BLOCK_SIZE];

} PROCESS_CB_DATA ;

char shared_buffer[16384];


//float a0 = (440.0 / 8.0) / kSampleRate; //48000.00;
const size_t   kBlockSize = BLOCK_SIZE;


struct Unit {
  braids::Quantizer   *quantizer;
  braids::SignatureWaveshaper *ws;

  bool            last_trig;

  // resampler
  //SRC_STATE       *src_state;
  PROCESS_CB_DATA pd;
  float           *samples;
  float           ratio;
};

static long src_input_callback(void *cb_data, float **audio);

struct Unit voices[1];



// Plaits modulation vars
float morph_in = 0.7f; // IN(4);
float trigger_in; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.1f;
float timbre_in = 0.1f;
int engine_in;

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
  if ( DAC.availableForWrite() ) {
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
      DAC.write( voices[0].pd.buffer[i] , sync );
    }
    counter =  1;
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
    Serial.begin(57600);
    Serial.println(F("YUP"));
  }
  // pwm timing setup
  // we're using a pseudo interrupt for the render callback since internal dac callbacks crash
  // Frequency in float Hz
  //ITimer0.attachInterrupt(TIMER_FREQ_HZ, TimerHandler0);
  /*
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS, TimerHandler0)) // that's 48kHz
  {
    if (debugging) Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
  }  else {
    if (debugging) Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }*/

  // set up Pico PWM audio output
  DAC.setBuffers(4, 32); // plaits::kBlockSize); // DMA buffers
  DAC.setBCLK(BCK);
  DAC.setDATA(DATA);
  DAC.setBitsPerSample(16);

  DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  DAC.begin();

  // switch to PWM for power to avoid ripple noise
  //pinMode(23, OUTPUT);
  //digitalWrite(23, HIGH);
  
  // init the braids voices
  initVoices();

}

void cb() {
  bool sync = true;
  if ( DAC.availableForWrite()) {
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
      DAC.write16( voices[0].pd.buffer[i] , voices[0].pd.buffer[i]);
    }
    counter =  1;
  }
  
}

// initialize voice parameters
void initVoices() {

  voices[0].ratio = 48000.f / MI_SAMPLERATE;

  // init some params
  voices[0].pd.osc = new braids::MacroOscillator;
  memset(voices[0].pd.osc, 0, sizeof(*voices[0].pd.osc));

  voices[0].pd.osc->Init(48000.f);
  voices[0].pd.osc->set_pitch((48 << 7));
  voices[0].pd.osc->set_shape(braids::MACRO_OSC_SHAPE_VOWEL_FOF);


  voices[0].ws = new braids::SignatureWaveshaper;
  voices[0].ws->Init(123774);

  voices[0].quantizer = new braids::Quantizer;
  voices[0].quantizer->Init();
  voices[0].quantizer->Configure(braids::scales[0]);

  //unit->jitter_source.Init();

  memset(voices[0].pd.buffer, 0, sizeof(int16_t)*BLOCK_SIZE);
  memset(voices[0].pd.sync_buffer, 0, sizeof(voices[0].pd.sync_buffer));
  memset(voices[0].pd.samps, 0, sizeof(float)*BLOCK_SIZE);

  voices[0].last_trig = false;

// get some samples initially
  updateBraidsAudio();

  /*
  // Initialize the sample rate converter
  int error;
  int converter = SRC_SINC_FASTEST;       //SRC_SINC_MEDIUM_QUALITY;
  

         // check resample flag
      int resamp = (int)IN0(5);
      CONSTRAIN(resamp, 0, 2);
      switch(resamp) {
          case 0:
              SETCALC(MiBraids_next);
              //Print("resamp: OFF\n");
              break;
          case 1:
              unit->pd.osc->Init(MI_SAMPLERATE);
              SETCALC(MiBraids_next_resamp);
              Print("MiBraids: internal sr: 96kHz - resamp: ON\n");
              break;
          case 2:
              SETCALC(MiBraids_next_reduc);
              Print("MiBraids: resamp: OFF, reduction: ON\n");
              break;
      }
  */
}

void updateBraidsAudio() {

  int16_t *buffer = voices[0].pd.buffer;
  uint8_t *sync_buffer = voices[0].pd.sync_buffer;
  size_t  size = BLOCK_SIZE;

  braids::MacroOscillator *osc = voices[0].pd.osc;

  // TODO: check setting pitch
  //CONSTRAIN(voct_in, 0.f, 127.f);
  //int pit = (int)voct_in;
  //float frac = voct_in - pit;
  //osc->set_pitch((pit << 7) + (int)(frac * 128.f));

  osc->set_pitch(pitch_in << 7);

  // set parameters
  CONSTRAIN(timbre_in, 0.f, 1.f);
  int16_t timbre = timbre_in * 32767.f;

  CONSTRAIN(morph_in, 0.f, 1.f);
  int16_t color = morph_in * 32767.f;
  osc->set_parameters(timbre, color);

  // set shape/model
  uint8_t shape = (int)(engine_in);
  if (shape >= braids::MACRO_OSC_SHAPE_LAST)
    shape -= braids::MACRO_OSC_SHAPE_LAST;
  osc->set_shape(static_cast<braids::MacroOscillatorShape>(shape));

  bool trigger = (trigger_in != 0.0);
  bool trigger_flag = (trigger && (!voices[0].last_trig));

  voices[0].last_trig = trigger;

  if (trigger_flag)
    osc->Strike();
    
  for (int count = 0; count < 32; count += size) {
    // render
    osc->Render(sync_buffer, buffer, size);

    /*for (int i = 0; i < size; ++i) {
      out[count + i] = buffer[i] * SAMP_SCALE;
    }*/
  }
  
}
void loop() {
  if ( counter > 0 ) {
    updateBraidsAudio();
    counter = 0; // increments on each pass of the timer after the timer writes samples
  }



}

// second core dedicated to display foo

void setup1() {
  delay (200); // wait for main core to start up perhipherals
}



// second core deals with ui / control rate updates
void loop1() {
  delay(100);
  float trigger = randomDouble(0.0, 1.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
  float harmonics = randomDouble(0.0, 0.7); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
  float timbre = randomDouble(0.0, 0.7); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
  float morph = randomDouble(0.1, 0.7) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
  float pitch = randomDouble(38, 54); // TIRand.kr(24, 48, trigger);
  float decay = randomDouble(0.1, 0.4);
  /*
    var sub = SinOsc.ar(pitch.midicps, 0, 0.1);
    var mi = MiPlaits.ar( pitch, engine, harmonics, timbre, morph,
      trigger: trigger, decay: 0.8, lpg_colour: 0.2, mul: 0.5);
    mi + sub
  */

  //Serial.println(pitch);


  pitch_in = pitch;
  harm_in = harmonics;
  morph_in = morph;
  timbre_in = timbre;
  if (trigger > 0.2 ) {
    trigger_in = trigger;
  }
  engineInc++ ;
  if (engineInc > 2) {
    engineCount ++; // don't switch engine so often :)
    engineInc = 0;
  }
  if (engineCount > 46) engineCount = 0;
  engine_in = engineCount;
  delay(2000);

}
