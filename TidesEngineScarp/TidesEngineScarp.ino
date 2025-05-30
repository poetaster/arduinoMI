/*
  (c) 2025 blueprint@poetaster.de GPLv3

   Based on  mi_Ugens, Copyright (c) 2020 Volker Böhm. All rights reserved. GPLv3
   https://vboehm.net

   Mutable Instruments sources, including the stmlib, tides, plaits elements and braids libs are
   MIT License
   Copyright (c)  2020 (emilie.o.gillet@gmail.com)

  TIDES
  freq  - In looping mode, frequency in Hz, else attack / release time.
  shape - Waveform shape (0. -- 1.)
  slope - Waveform slope, from falling ramp |\ to rising ramp /| (0. -- 1.)
  smooth - Smoothness of the waveform. From 0.5 to 0.0 a lowpass filter is applied, from 0.5 to 1.0 a wavefolder adds kinks and bumps along the slope (0. -- 1.)
  shift - Shift/Level, controls the shift in amplitude/time/frequency (0. -- 1.)
  trig  - Trigger input. In AD mode, a trigger will reset the envelope to 0.0 and initiate an attack/decay cycle. No matter how short the trigger is, the attack/decay cycle will always complete.
      - In cyclic mode, a trigger will reset the oscillator so that it starts a new ascending phase.
      - In AR mode, the rising edge of the gate will cause the envelope to raise to +0.5 from its current value, and the falling edge of the gate will cause the envelope to fall to 0.0 from its current value – the same way an ADSR envelope with a null decay time and a 100% sustain level would behave.

  clock - Clock input for tempo or frequency-locked operation. When this input is patched, the ratio argument controls the ratio between the frequency of the signal sent to this input, and the frequency of the signals generated by MiTides.
  output_mode - Chooses the output mode. 0: GATES, 1: AMPLITUDES, 2: PHASES, 3: FREQUENCIES.
  ramp_mode - Chooses the ramp mode. 0: AD, 1: LOOPING, 2: AR
  ratio - Set the ratio to the incomping clock frequency.
       - 0: 0.0625, 16, 1: 0.125, 8, 2: 0.1666666, 6, 3: 0.25, 4, 4: 0.3333333, 3, 5: 0.5, 2, 6: 0.6666666, 3, 7: 0.75, 4, 8: 0.8, 5, 9: 1, 1, 10: 1.25, 4, 11: 1.3333333, 3, 12: 1.5, 2, 13: 2.0, 1, 14: 3.0, 1, 15: 4.0, 1, 16: 6.0, 1, 17: 8.0, 1, 18: 16.0, 1
  rate  - Chooses between 0: CONTROL and 1: AUDIO rate operation.
      - 'Control' is normally meant for low frequency envs and LFOs, while 'Audio' uses some ani-alias-filtering for audio rate frequencies - but it's up to you to choose...
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

#include "Midier.h"

// button inputs
#define BUTTON0  0 // key1 input on schematic
#define BUTTON1  2
#define BUTTON2 4
#define BUTTON3 6
#define BUTTON4 8
#define BUTTON5 10
#define BUTTON6 12
#define BUTTON7 14  // for the Pico board
#define SHIFTBUTTON 28 // was 24 USR button on VCC-GND boards

#define LED0 1 // LED1 output on schematic
#define LED1 3
#define LED2 5
#define LED3 7
#define LED4 9
#define LED5 11
#define LED6 13
#define LED7 15
#define LED 15 // shows if MIDI is being recieved
// analog freq pins

#include <MIDI.h>
#include <Wire.h>

// we do some generic timing for switch which waveform to display / play
unsigned long startMillis;
unsigned long currentMillis;
int wChannel = 0;

//MIDI_CREATE_DEFAULT_INSTANCE();
// freq to midi
static const uint32_t midi_note_at_zero_volts = 12;
static const float semitones_per_octave = 12.0f;
static const float volts_per_semitone = 1.0f / semitones_per_octave;
static const float a4_frequency = 440.0f;
static const uint32_t a4_midi_note = 69;

float midi_frequency(uint32_t midi_note) {
  float semitones_away_from_a4 = (float)(midi_note) - (float)(a4_midi_note);
  return powf(2.0f, semitones_away_from_a4 / semitones_per_octave) * a4_frequency;
}
// tides dsp

#include <STMLIB.h>
#include <TIDES.h>

const size_t kAudioBlockSize = 8;        // sig vs can't be smaller than this!
const size_t kNumOutputs = 4;

static tides::Ratio kRatios[19] = {
  { 0.0625f, 16 },
  { 0.125f, 8 },
  { 0.1666666f, 6 },
  { 0.25f, 4 },
  { 0.3333333f, 3 },
  { 0.5f, 2 },
  { 0.6666666f, 3 },
  { 0.75f, 4 },
  { 0.8f, 5 },
  { 1, 1 },
  { 1.25f, 4 },
  { 1.3333333f, 3 },
  { 1.5f, 2 },
  { 2.0f, 1 },
  { 3.0f, 1 },
  { 4.0f, 1 },
  { 6.0f, 1 },
  { 8.0f, 1 },
  { 16.0f, 1 },
};

#define     SAMP_SCALE          (float)(1.0 / 32756.0)
#define     BLOCK_SIZE          32      // --> macro_oscillator.h !
char shared_buffer[32756];

struct Unit {
  tides::PolySlopeGenerator poly_slope_generator;
  tides::RampExtractor ramp_extractor;

  tides::PolySlopeGenerator::OutputSample out[kAudioBlockSize];
  stmlib::GateFlags   no_gate[kAudioBlockSize];
  stmlib::GateFlags   gate_input[kAudioBlockSize];
  stmlib::GateFlags   clock_input[kAudioBlockSize];
  stmlib::GateFlags   previous_flags_[2 + 1];       // two inlets, TODO: why + 1
  tides::Ratio        r_;

  float   ramp[kAudioBlockSize];

  tides::OutputMode   output_mode;
  tides::OutputMode   previous_output_mode;
  tides::RampMode     ramp_mode;
  tides::Range        range;

  float       frequency, freq_lp;
  float       shape, shape_lp;
  float       slope, slope_lp;
  float       smoothness, smooth_lp;
  float       shift, shift_lp;

  bool        must_reset_ramp_extractor;

  float       sr;
  float       r_sr;
  int16_t     buffer[BLOCK_SIZE]; // we're using this to mix the channel output
};

static long src_input_callback(void *cb_data, float **audio);

struct Unit voices[1];

// Plaits modulation vars
float morph_in = 0.1f; // IN(4);
float trigger_in; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.15f;
float timbre_in = 0.1f;
int engine_in;

// Tides, we're re-using the above for inputs
float   freq_in = 200.0f;  //IN0(0);
float   shape_in = 0.1f;  //IN0(1);
float   slope_in = 0.1f;  //IN0(2);
float   smooth_in = 0.1f; // IN0(3);
float   shift_in = 0.1f; // IN0(4);
int pitch_in = 90;
float shapeIn;

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
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
      DAC.write( voices[0].buffer[i] , sync );
    }
    counter =  1;
  }
  return true;
}

// GPIO, pots
const int INTS_PIN = 26; // set the analog input for fm_intensity
const int RATE_PIN = 2; // set the analog input for mod rate
const int MODR_PIN = 27; // set the analog input for mod ratio

// encoder related
#include <RotaryEncoder.h>
const int encoderA_pin = 19;
const int encoderB_pin = 18;
const int encoderSW_pin = 28;

RotaryEncoder encoder(encoderB_pin, encoderA_pin, RotaryEncoder::LatchMode::FOUR3);

void checkEncoderPosition() {
  encoder.tick();   // call tick() to check the state.
}

// display related
const int oled_sda_pin = 20;
const int oled_scl_pin = 21;
const int oled_i2c_addr = 0x3C;
const int dw = 128;
const int dh = 64;

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//Adafruit_SSD1306 display(dw, dh, &Wire, OLED_RESET);
//#include <Adafruit_SSD1306.h>

#include <Adafruit_SH110X.h>
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire);
#define WHITE SH110X_WHITE

#include "font.h"
#include "helvnCB6pt7b.h"
#define myfont helvnCB6pt7b // Org_01 looks better but is small.
#include "shapes.h" // char array of names of instruments.
#include "display.h"


// variables for UI state management
int encoder_pos_last = 0;
int encoder_delta = 0;
uint32_t encoder_push_millis;
uint32_t step_push_millis;
bool encoder_held = false;

// buttons & knobs defines/functions
#include "control.h"
// midi related functions
#include "midi.h"

// utility
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}



// audio related defines

//float freqs[12] = { 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f, 493.88f};
int freqs[12] = { 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64};
int carrier_freq;

int current_track;
int bpm = 120;

uint16_t noteA;
uint16_t noteM;
uint16_t attackIn = 8; // up to 384
uint16_t durationIn = 200; // up to 384
uint16_t modIn = 200; // up to 2800
uint16_t modC = 0;
int pressedB;
int RATE_value = 1 ; //was an adc in kevin's orig


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
  DAC.setBuffers(4, 32); // plaits::kBlockSize); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  DAC.begin();

  // let's get more resolution
  analogReadResolution(12);

  // Additions
  // ENCODER
  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderB_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA_pin), checkEncoderPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_pin), checkEncoderPosition, CHANGE);
  // DISPLAY
  Wire.setSDA(oled_sda_pin);
  Wire.setSCL(oled_scl_pin);
  Wire.begin();

  // SSD1306 --  or SH1106 in this case
  //if (!display.begin(SSD1306_SWITCHCAPVCC, oled_i2c_addr)) {
  if (!display.begin( oled_i2c_addr)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ;  // Don't proceed, loop forever
  }
  displaySplash();

  pinMode(BUTTON0, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
  pinMode(BUTTON5, INPUT_PULLUP);
  pinMode(BUTTON6, INPUT_PULLUP);
  pinMode(BUTTON7, INPUT_PULLUP);
  pinMode(SHIFTBUTTON, INPUT_PULLUP);

  pinMode(26, INTS_PIN);
  pinMode(27, MODR_PIN);

  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);

  pinMode(LED, OUTPUT);

  // thi is to switch to PWM for power to avoid ripple noise
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);



  // init the braids voices
  initVoices();
  // initialize a mode to play
  mode = midier::Mode::Ionian;
  makeScale( roots[scaleRoot], mode);

  startMillis = millis();

}

// initialize voice parameters
void initVoices() {

  // zero out voice buffer
  memset(voices[0].buffer, 0, sizeof(int16_t)*BLOCK_SIZE);

  voices[0].sr = SAMPLERATE;
  voices[0].r_sr =  1.f / voices[0].sr;

  voices[0].poly_slope_generator.Init();
  voices[0].ramp_extractor.Init(voices[0].sr, 40.0f * voices[0].r_sr);

  std::fill(&voices[0].no_gate[0], &voices[0].no_gate[kAudioBlockSize], stmlib::GATE_FLAG_LOW);
  std::fill(&voices[0].clock_input[0], &voices[0].clock_input[kAudioBlockSize], stmlib::GATE_FLAG_LOW);
  std::fill(&voices[0].ramp[0], &voices[0].ramp[kAudioBlockSize], 0.f);

  voices[0].output_mode = tides::OUTPUT_MODE_FREQUENCY;
  voices[0].previous_output_mode = tides::OUTPUT_MODE_FREQUENCY;
  voices[0].ramp_mode = tides::RAMP_MODE_LOOPING;
  voices[0].range = tides::RANGE_CONTROL;

  voices[0].previous_flags_[0] = stmlib::GATE_FLAG_LOW;
  voices[0].previous_flags_[1] = stmlib::GATE_FLAG_LOW;

  voices[0].must_reset_ramp_extractor = false;

  voices[0].frequency = 20.f;
  voices[0].shape = 0.5f;
  voices[0].slope = 0.5f;
  voices[0].smoothness = 0.5f;
  voices[0].shift = 0.3f;


  voices[0].freq_lp = voices[0].shape_lp = voices[0].slope_lp = voices[0].smooth_lp = voices[0].shift_lp = 0.f;

  voices[0].r_.ratio = 1.0f;
  voices[0].r_.q = 1;


}

void cb() {
  if (DAC.availableForWrite() > 14) {
    for (int i = 0; i <  BLOCK_SIZE; i++) {
      // out = ;   // left channel called .aux
      DAC.write( voices[0].buffer[i] );
    }
  }
}


bool canBufferAudioOutput() {
  if ( DAC.availableForWrite() > 32 ) {
    return true;
  }
  return false;
}

void updateControl() {

  //MIDI.read();

  uint32_t now = millis();
  bool anybuttonpressed;
  anybuttonpressed = false;

  int p1 = potvalue[0]; //analogRead(MODR_PIN); // value is 0-4065
  int p2 = potvalue[1]; // analogRead(INTS_PIN); // value is 0-4065


  morph_in = (float)p1 / 4.095f; //map(p1, 0, 4065, 0.0, 1.0); // IN(2);
  timbre_in = (float)p2 / 4095.0f; //map(p2, 0, 4065, 0.0, 1.0); //IN(3);
  CONSTRAIN(morph_in, 0.0f, 1000.0f);
  CONSTRAIN(timbre_in, 0.0f, 1.0f);
  scanbuttons();


  for (int i = 0; i < 9; ++i) { // scan all the buttons
    if (button[i]) {

      anybuttonpressed = true;
      if (i < 8) {
        digitalWrite(led[i] , HIGH);

        // a track button is pressed
        current_track = i; // keypress selects track we are working on

        if ( encoder_delta == 0) {
          aNoteOff(currentMode[i], 0);
          //noteA = freqs[i];
          if (button[8]) scaleRoot = i; // change scaleroot if both encoder and another button is pressed.
          pitch_in = currentMode[i] + 24; //freqs[i];
          aNoteOn( pitch_in, 100 );
        }
        pressedB = i;
      } else {
        modeIndex = modeIndex + 1;
        if (modeIndex == 8 ) modeIndex = 0;

        switch (modeIndex) {
          case 0:
            mode = midier::Mode::Ionian;
            break;
          case 1:
            mode = midier::Mode::Dorian;
            break;
          case 2:
            mode = midier::Mode::Phrygian;
            break;
          case 3:
            mode = midier::Mode::Lydian;
            break;
          case 4:
            mode = midier::Mode::Mixolydian;
            break;
          case 5:
            mode = midier::Mode::Aeolian;
            break;
          case 6:
            mode = midier::Mode::Locrian;
            break;
        }
        makeScale( roots[scaleRoot], mode);
      }
    }
  }
}

void updateTidesAudio() {
  // ugen defaults: MiTides.ar(freq: 1, shape: 0.5, slope: 0.5, smooth: 0.5, shift: 0.2, trig: 0, clock: 0, output_mode: 3, ramp_mode: 1, ratio: 9, rate: 1, mul: 1.0, add: 0.0)

  float   freq_in = morph_in; // midi_frequency(pitch_in); // IN0(0);
  float   shape_in = shapeIn; // IN0(1);
  float   slope_in = timbre_in; // IN0(2);
  float   smooth_in = harm_in; // IN0(3);
  float   shift_in = 0.0f; // IN0(4);


  float   trig_in = trigger_in; // IN(5);
  float   clock_in = clock_in; // IN(6);

  int     outp_mode = 3;
  int     rmp_mode = 1;
  int     ratio = 9;
  int     rate = 0; //IN0(10);     // choose from CONTROL or AUDIO rate/range


  int vs = BLOCK_SIZE;

  tides::RampExtractor *ramp_extractor = &voices[0].ramp_extractor;
  tides::PolySlopeGenerator::OutputSample *out = voices[0].out;
  float   *ramp = voices[0].ramp;

  stmlib::GateFlags *clock_input = voices[0].clock_input;
  stmlib::GateFlags *gate_flags = voices[0].no_gate;
  stmlib::GateFlags *previous_flags = voices[0].previous_flags_;

  bool    must_reset_ramp_extractor = voices[0].must_reset_ramp_extractor;
  bool    use_clock = false;
  bool    use_trigger = false;

  CONSTRAIN(outp_mode, 0, 3);
  CONSTRAIN(rmp_mode, 0, 2);
  CONSTRAIN(ratio, 0, 18);

  tides::OutputMode   output_mode = (tides::OutputMode)outp_mode;
  tides::RampMode     ramp_mode =  (tides::RampMode)rmp_mode;
  tides::Range        range = (tides::Range)rate; //voices[0].range;
  tides::Ratio        r_ = kRatios[ratio];
  /*
    unit->output_mode = tides::OUTPUT_MODE_FREQUENCY;
    unit->previous_output_mode = tides::OUTPUT_MODE_FREQUENCY;
    unit->ramp_mode = tides::RAMP_MODE_LOOPING;
    unit->range = tides::RANGE_AUDIO;
  */


  float   frequency, shape, slope, shift, smoothness;

  //    float   freq_lp = voices[0].freq_lp;
  float   shape_lp = voices[0].shape_lp;
  float   slope_lp = voices[0].slope_lp;
  float   shift_lp = voices[0].shift_lp;
  float   smooth_lp = voices[0].smooth_lp;

  float   r_sr = voices[0].r_sr;

  // if ( trigger_in >0.1f ) use_trigger = true;
  /* Disabled for now



    if ( INRATE(6) == calc_FullRate )
    use_clock = true;
  */

  for (int count = 0; count < vs; count += kAudioBlockSize) {

    // check for gate/trigger input
    if (use_trigger) {
      gate_flags = voices[0].gate_input;
      for (int i = 0; i < kAudioBlockSize; ++i) {
        bool trig = trig_in > 0.01; // trig_in[i + count] > 0.01;
        previous_flags[0] = stmlib::ExtractGateFlags(previous_flags[0], trig);
        gate_flags[i] = previous_flags[0];
      }
    }

    if (use_clock) {
      if (must_reset_ramp_extractor) {
        ramp_extractor->Reset();
      }

      for (int i = 0; i < kAudioBlockSize; ++i) {
        bool trig = clock_in > 0.01; // clock_in[i + count] > 0.01;
        previous_flags[1] = stmlib::ExtractGateFlags(previous_flags[1], trig);
        clock_input[i] = previous_flags[1];
      }

      frequency = ramp_extractor->Process(range,
                                          range == tides::RANGE_AUDIO && ramp_mode == tides::RAMP_MODE_AR,
                                          r_,
                                          clock_input,
                                          ramp,
                                          kAudioBlockSize);

      must_reset_ramp_extractor = false;

    }

    else {
      frequency = (freq_in) * r_sr;
      CONSTRAIN(frequency, 0.f, 0.4f);
      // no filtering for now
      //            ONE_POLE(freq_lp, frequency, 0.3f);
      //            frequency = freq_lp;
      must_reset_ramp_extractor = true;
    }
    /*
        Serial.print("freq: ");
        Serial.println(freq_in);
            Serial.print("pitch: ");
        Serial.println(pitch_in);
    */
    // parameter inputs
    shape = shape_in;
    CONSTRAIN(shape, 0.f, 1.f);
    ONE_POLE(shape_lp, shape, 0.1f);
    
    slope = slope_in;
    CONSTRAIN(slope, 0.f, 1.f);
    ONE_POLE(slope_lp, slope, 0.1f);
    
    smoothness = smooth_in;
    CONSTRAIN(smoothness, 0.f, 1.f);
    ONE_POLE(smooth_lp, smoothness, 0.1f);
    
    shift = shift_in;
    CONSTRAIN(shift, 0.f, 1.f);
    ONE_POLE(shift_lp, shift, 0.1f);


    voices[0].poly_slope_generator.Render(ramp_mode,
                                          output_mode,
                                          range,
                                          frequency, slope_lp, shape_lp, smooth_lp, shift_lp,
                                          gate_flags,
                                          !use_trigger && use_clock ? ramp : NULL,
                                          out, kAudioBlockSize);

    float samplesum = 0.f;
    int16_t sample1;
    int16_t sample2;

    for (int i = 0; i < kAudioBlockSize; ++i) {
      for (int j = 0; j < kNumOutputs; ++j) {
        samplesum = samplesum + ( out[i].channel[j]);
      }

      sample1 = stmlib::Clip16(static_cast<int16_t>(out[i].channel[wChannel] * 32768.0f));

      //sample2 = stmlib::Clip16(static_cast<int16_t>(out[i].channel[1] * 32768.0f));
      //samplesum = (samplesum/(float)kNumOutputs) ;

      //displayGraph(out[i].channel[0], out[i].channel[1], out[i].channel[2], out[i].channel[3]);

      displayAGraph(out[i].channel[wChannel]);
      voices[0].buffer[i] = sample1; // stmlib::Clip16(static_cast<int16_t>(out[i].channel[wChannel] * 32768.0f));
      // stmlib::Mix( sample1, sample2, 0.5f ); // render sum to output
      //voices[0].buffer[i] = (int16_t) out[i].channel[0] * 32767.f;
    }
  }

  //    voices[0].freq_lp = freq_lp;
  voices[0].shape_lp = shape_lp;
  voices[0].shift_lp = shift_lp;
  voices[0].slope_lp = slope_lp;
  voices[0].smooth_lp = smooth_lp;

  voices[0].must_reset_ramp_extractor = must_reset_ramp_extractor;

}

void loop() {
  // when the osc buffer has been written to PWM buffer
  if ( counter > 0 ) {
    updateTidesAudio();
    counter = 0; // increments on each pass of the timer when the timer writes
  }


}

// second core dedicated to display foo

void setup1() {
  delay (200); // wait for main core to start up perhipherals
}

int engineCount = 0;
int engineInc = 0;



// second core deals with ui / control rate updates
void loop1() {



  updateControl(); //pots values on this loop since the other delays

  uint32_t now = millis();
  bool anybuttonpressed;
  anybuttonpressed = false;

  // UI handlers
  // first encoder
  encoder.tick();

  int encoder_pos = encoder.getPosition();
  if ( (encoder_pos != encoder_pos_last )) {
    encoder_delta = encoder_pos - encoder_pos_last;
  }
  // set play mode 0 play 1 edit patterns, 3 FX?
  if (encoder_push_millis > 0 ) {
    if ((now - encoder_push_millis) > 25 && encoder_delta ) {

      if ( !encoder_held ) encoder_held = true;
      /*
      engineCount = engineCount + encoder_delta;
      if (engineCount > 20) engineCount = 1;
      if (engineCount < 1) engineCount = 20;
      float shape = engineCount / 20.0f;
      engine_in = shape;
      */
    }

    if (step_push_millis > 0) { // we're pushing a step key too
      if (encoder_push_millis < step_push_millis) {  // and encoder was pushed first
        //strcpy(seq_info, "saveseq");
      }
    }
  }



  for (int i = 0; i < 9; ++i) { // scan all the buttons
    if (button[i]) {
      wChannel = i;
      if (wChannel > 3) wChannel = 0;

      anybuttonpressed = true;
      if (i < 8)  digitalWrite(led[i] , HIGH);

      //  if ((!potlock[1]) || (!potlock[2])) seq[i].trigger=euclid(16,map(potvalue[1],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS),map(potvalue[2],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS-1));
      // look up drum trigger pattern encoder play modes


      if ( i == 8 && encoder_delta) {
        //engineCount = engineCount + encoder_delta;
        shapeIn = shapeIn + (encoder_delta * 0.05f);
        CONSTRAIN(shape_in, 0.0f, 1.0f);
      }
      /*
      if ( (encoder_pos != encoder_pos_last ) && i == 8  ) {
        engineCount = engineCount + encoder_delta;
        if (engineCount > 10) engineCount = 1;
        if (engineCount < 1) engineCount = 10;
        float shape = engineCount / 10.0f;
        engine_in = shape;

      }*/

      // change pitch on pot 0
      if (display_mode == 0 ) { // change sample if pot has moved enough
        //noteA = (map(mozziAnalogRead(MODR_PIN), POT_MIN, POT_MAX, 200, 10000));
        //voice[current_track].sampleincrement = (uint16_t)(map(potvalue[0], POT_MIN, POT_MAX, 2048, 8192)); // change sample pitch if pot has moved enough
      }

      // change volume on pot 1
      if (display_mode == 0) {
        //mod_to_carrier_ratio = (map(mozziAnalogRead(AIN1), POT_MIN, POT_MAX, 1, 20));
        //voice[current_track].level = (int16_t)(map(potvalue[1], POT_MIN, POT_MAX, 0, 1000));
        // change sample volume level if pot has moved enough
      }

      if (!potlock[0] && display_mode == 1 ) {
        //filter_fc = potvalue[0] * (LPF_MAX + 10) / 4096;
      }

      // set track euclidean triggers if either pot has moved enough
      if (!potlock[1] && ! button[8] && display_mode == 1) {
        //  seq[i].fills = map(potvalue[1], POT_MIN, POT_MAX, 0, 16);
        //  seq[i].trigger->generateSequence(seq[i].fills, 15);
        //seq[i].trigger= drumpatterns[map(potvalue[1],POT_MIN,POT_MAX,0,NUMPATTERNS-1)];
      }

    } else {
      if (i < 8) digitalWrite(led[i] , LOW); // else the button is off.
    }
  }

  // now, after buttons check if only encoder moved and no buttons
  // this is broken by mozzi, sigh.
  if (! anybuttonpressed && encoder_delta) {
    float turn = encoder_delta * 0.01f;
    harm_in = harm_in + turn;
    CONSTRAIN(harm_in, 0.0f, 1.0f);
    //display_value(RATE_value - 50); // this is wrong, bro :)
  } 


  /// only set new pos last after buttons have had a chance to use the delta
  encoder_pos_last = encoder_pos;
  encoder_delta = 0;  // we've used it

  // start tracking time encoder button held
  if ( ! digitalRead( SHIFTBUTTON ) ) {
    encoder_push_millis = now;
  } else {
    encoder_push_millis = 0;
    encoder_held = false;
  }

  // lock pot settings when no keys are pressed so it requires more movement to change value
  // this is so when we change tracks we don't immediately change the settings on the new track
  if (!anybuttonpressed) lockpots();

  // reading A/D seems to cause noise in the audio so don't do it too often
  if ((now - pot_timer) > POT_SAMPLE_TIME) {
    readpot(0);
    readpot(1);
    pot_timer = now;
  }

  displayUpdate();

  //delay(3000);


}
