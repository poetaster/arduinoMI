/*
  (c) 2024 blueprint@poetaster.de
  GPLv3

      Some sources, including the stmlib and plaits lib are
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

//MIDI_CREATE_DEFAULT_INSTANCE();

// plaits dsp
#include <STMLIB.h>
#include <PLAITS.h>

plaits::Modulations modulations;
plaits::Patch patch;
plaits::Voice voice;

//Settings settings;
//Ui ui;
//UserData user_data;
//UserDataReceiver user_data_receiver;

char shared_buffer[32768];
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
  bool                last_trig; // from braids

  char                *shared_buffer;
  void                *info_out;
  bool                prev_trig;
  float               sr;
  int                 sigvs;
};

struct Unit voices[1];
volatile plaits::Patch renderPatch ;
volatile plaits:: Modulations renderModulations ;
volatile plaits::Voice *renderVoice;


// Plaits modulation vars
float morph_in = 0.7f; // IN(4);
float trigger_in = 0.0f; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.5f;
float timbre_in = 0.1f;
int engine_in;

float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.0f; //IN(8);
float morph_mod = 0.0f; //IN(9);
float decay_in = 0.2f; // IN(10);
float lpg_in = 0.1f ;// IN(11);
float pitch_in = 60.0f;

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
    for (size_t i = 0; i < plaits::kBlockSize; i++) {
      DAC.write( (uint16_t)outputPlaits[i].out , sync ); // 244 is mozzi audio bias
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
float freqs[12] = { 42.0f, 44.0f, 46.0f, 48.0f, 50.0f, 52.0f, 54.0f, 56.0f, 58.0f, 60.0f, 62.0f, 64.0f};
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

  pinMode(23, OUTPUT); // thi is to switch to PWM for power to avoid ripple noise
  digitalWrite(23, HIGH);

  // init the plaits voices

  initVoices();
  // prefill buffer
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  //MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  //MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  //MIDI.begin(MIDI_CHANNEL_OMNI);


}

// initialize voice parameters
void initVoices() {

  // init some params
  voices[0] = {};
  voices[0].modulations = modulations;
  voices[0].modulations.engine = 0;
  voices[0].patch = patch;
  voices[0].patch.engine = 0;
  voices[0].transposition_ = 0.;
  voices[0].octave_ = 0.6
                      ;
  voices[0].patch.note = 48.0;
  voices[0].patch.harmonics = 0.5;
  voices[0].patch.morph = 0.3;
  voices[0].patch.timbre = 0.3;
  voices[0].last_trig = false;

  voices[0].shared_buffer = shared_buffer;
  // init with zeros
  memset(voices[0].shared_buffer, 0, 32768);


  stmlib::BufferAllocator allocator(shared_buffer, 32768);
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

void cb() {
  if (DAC.availableForWrite() > 14) {
    for (int i = 0; i <  plaits::kBlockSize; i++) {
      uint16_t out = outputPlaits[i].out;   // left channel called .aux
      DAC.write( out );
    }
  }
}

/*
  void audioOutput() {
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);

  for (int i = 0; i <  plaits::kBlockSize; i++) {
    uint16_t out = outputPlaits[i].out;
    if ( DAC.availableForWrite() ) DAC.write( out );
  }
  // write samples to DMA buffer - this is a blocking call so it stalls when buffer is full
  // f.l() + MOZZI_AUDIO_BIAS
  // left
  }

  bool canBufferAudioOutput() {

  if ( DAC.availableForWrite() > 32 ) {
    return true;
  }
  return false;
  }


  bool updateAudio() {
  if (canBufferAudioOutput()) {
    audioOutput();
    return true;
  }
  return false;

  }
*/

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

  // use the debounced values from readpots
  int p1 = potvalue[0]; //analogRead(MODR_PIN); // value is 0-4065
  int p2 = potvalue[1]; // analogRead(INTS_PIN); // value is 0-4065


  morph_in = (float)p1 / 1000.0f; //map(p1, 0, 4065, 0.0, 1.0); // IN(2);
  timbre_in = (float)p2 / 1000.0f; //map(p2, 0, 4065, 0.0, 1.0); //IN(3);
  CONSTRAIN(morph_in, 0.0f, 1.0f);
  CONSTRAIN(timbre_in, 0.0f, 1.0f);
  scanbuttons();


  for (int i = 0; i < 9; ++i) { // scan all the buttons
    if (button[i]) {

      anybuttonpressed = true;
      if (i < 8)  digitalWrite(led[i] , HIGH);

      // a track button is pressed
      current_track = i; // keypress selects track we are working on

      if ( encoder_delta == 0) {
        //if (pressedB != i) {
        // turn off the last note
        //aNoteOff(freqs[pressedB],0);
        //}
        aNoteOff(freqs[pressedB], 0);
        noteA = freqs[i];
        pitch_in = freqs[i] + 24.0f;
        //voices[0].patch.note = pitch_in;
        aNoteOn( freqs[i], 100 );
      }
      pressedB = i;
    }
  }

  //we're faking triggers with button presses for now
  /*
    if ( trig_in > 0.0f ) {
    voices[0].modulations.trigger_patched = true;
    float sum = 0.f;

    //if (INRATE(5) == calc_FullRate) {

    if (trig_in > 0.9f) {
      // trigger input is audio rate
      // TODO: add vDSP vector summation
      //for (int i = 0; i < inNumSamples; ++i)
      //  sum += trig_in[i];
      sum = trig_in;
    }
    else {          // trigger input is control rate
      sum = trig_in;
    }

    voices[0].modulations.trigger = sum;
    }
    else {
    voices[0].modulations.trigger_patched = false;
    }
    if (level_in > 0.0f) {
    voices[0].modulations.level_patched = true;
    voices[0].modulations.level = level_in;
    }
    else
    voices[0].modulations.level_patched = false;
  */
}


void loop() {
  // updateAudio();
  if ( counter > 0 ) {
    voices[0].patch.frequency_modulation_amount = fm_mod;
    voices[0].patch.engine = engine_in;
    //voices[0].transposition_ = 0.;
    //voices[0].octave_ = 0.5;
    voices[0].patch.note = pitch_in;
    voices[0].patch.harmonics = harm_in;
    voices[0].patch.morph = morph_in;
    voices[0].patch.timbre = timbre_in;
    voices[0].patch.decay = decay_in; //0.5f;
    voices[0].patch.lpg_colour = lpg_in;
    voices[0].modulations.trigger = trigger_in;

    voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);
    counter = 0; // increments on each pass of the timer when the timer writes
  }


}

// second core dedicated to display foo

void setup1() {
  delay (1000); // wait for main core to start up perhipherals
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
    if ((now - encoder_push_millis) > 25 && ! encoder_delta ) {

      if ( !encoder_held ) encoder_held = true;

      display_mode = display_mode + 1;
      if ( display_mode > 2) { // switched back to play mode
        display_mode = 0;
        //configure_sequencer();
      }
    }

    if (step_push_millis > 0) { // we're pushing a step key too
      if (encoder_push_millis < step_push_millis) {  // and encoder was pushed first
        //strcpy(seq_info, "saveseq");
      }
    }
  }



  for (int i = 0; i < 9; ++i) { // scan all the buttons
    if (button[i]) {

      anybuttonpressed = true;
      if (i < 8)  digitalWrite(led[i] , HIGH);
      /*
        // a track button is pressed
        current_track = i; // keypress selects track we are working on

        if ( encoder_delta == 0) {
        //if (pressedB != i) {
        // turn off the last note
        //aNoteOff(freqs[pressedB],0);
        //}
        aNoteOff(freqs[pressedB], 0);
        noteA = freqs[i];
        pitch_in = freqs[i] + 24.0f;
        voices[0].patch.note = 24.0f + pitch; // 60.f + pitch * 12.f;
        aNoteOn( freqs[i], 100 );
        }
        pressedB = i;
      */

      //  if ((!potlock[1]) || (!potlock[2])) seq[i].trigger=euclid(16,map(potvalue[1],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS),map(potvalue[2],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS-1));
      // look up drum trigger pattern encoder play modes

      if ( i == 8) {
        engineCount = engineCount + encoder_delta;
        CONSTRAIN(engineCount, 0, 16);

        engine_in = engineCount; // ( engine +) % voices[0].voice_.GetNumEngines();

      }

      if ( i == 4 && encoder_delta != 0) {

      }

      if ( (encoder_pos != encoder_pos_last ) && i == 1  ) {
        engineCount = engineCount + encoder_delta;
        CONSTRAIN(engineCount, 0, 16);
        engine_in = engineCount;

      }

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
    //bpm = bpm + encoder_delta;
    //RATE_value = RATE_value + encoder_delta;
    //voices[0].patch.note = voices[0].patch.note + RATE_value;


    if (encoder_delta > 1) {
      harm_in = harm_in + 0.05f;
      CONSTRAIN(harm_in, 0.0f, 1.0f);

    } else {
      harm_in = harm_in - 0.05f;
      CONSTRAIN(harm_in, 0.0f, 1.0f);

    }
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



void automatic() {
  if (engineCount > 16) engineCount = 0;

  /*
    float trigger = randomDouble(0.0, 2.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
    float harmonics = randomDouble(0.2, .7); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
    float timbre = randomDouble(0.1, .6); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
    float morph = randomDouble(0.2, 0.6) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
    float pitch = abs(randomDouble(34, 60)); // TIRand.kr(24, 48, trigger);
    float octave = randomDouble(0.3, 0.5);
    float decay = randomDouble(0.05, 0.2);
    float fm_mod = randomDouble(0.05, 0.2);
    float lpg = randomDouble(0.05, 0.2);

    //voices[0].patch.frequency_modulation_amount = fm_mod;
    voices[0].patch.engine = engineCount;
    //voices[0].transposition_ = 0.;
    voices[0].octave_ = 0.3;
    //voices[0].patch.note = pitch;
    //voices[0].patch.harmonics = harmonics;
    //voices[0].patch.morph = morph;
    //voices[0].patch.timbre = timbre;
    voices[0].patch.decay = decay; //0.5f;
    voices[0].patch.lpg_colour = lpg;

    if (trigger > 0.2 ) {
    voices[0].modulations.trigger = trigger;
    voices[0].modulations.trigger_patched = true;
    } else {
    voices[0].modulations.trigger = 0.0f;
    voices[0].modulations.trigger_patched = false;
    }

    engineInc++ ;
    if (engineInc > 5) {
    engineCount ++; // don't switch engine so often :)
    engineInc = 0;
    }

  */
}
