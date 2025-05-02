/*
  (c) 2024 blueprint@poetaster.de
  GPLv3

  BASED in part on
  Arduino Mozzi MIDI FM Synthesis 2
  https://diyelectromusic.wordpress.com/2020/08/19/arduino-fm-midi-synthesis-with-mozzi-part-2/

      MIT License
      Copyright (c) 2020 diyelectromusic (Kevin)

  Using principles from the following Arduino tutorials:
    Arduino MIDI Library - https://github.com/FortySevenEffects/arduino_midi_library
    Mozzi Library        - https://sensorium.github.io/Mozzi/

  Much of this code is based on the Mozzi example Knob_LightLevel_x2_FMsynth (C) Tim Barrass
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
volatile plaits::Patch renderPatch ;
volatile plaits:: Modulations renderModulations ;
volatile plaits::Voice *renderVoice;


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

// clock timer  stuff

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

//unsigned int SWPin = CLOCKIN;

#define TIMER0_INTERVAL_MS 20.833333333333   // \20.833333333333running at 48Khz
#define DEBOUNCING_INTERVAL_MS   2// 80
#define LOCAL_DEBUG              1

volatile int counter = 0;

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;
  if ( DAC.availableForWrite() )  {
    for (size_t i = 0; i < kBlockSize; i++) {
      DAC.write( (uint16_t)outputPlaits[i].out , sync ); // 244 is mozzi audio bias
    }
    counter++;
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

enum {
  MODE_PLAY = 0,
  MODE_CONFIG,
  MODE_COUNT   // how many modes we got
};

int display_mode = 0;
uint8_t display_repeats = 0;

// variables for UI state management
int encoder_pos_last = 0;
int encoder_delta = 0;
uint32_t encoder_push_millis;
uint32_t step_push_millis;
bool encoder_held = false;

// the somewhat flake code from my pikobeats version
// pot locking and sample averaging

// pots
#define NPOTS 2 // number of pots
uint16_t potvalue[NPOTS]; // pot readings
uint16_t lastpotvalue[NPOTS]; // old pot readings
bool potlock[NPOTS]; // when pots are locked it means they must change by MIN_POT_CHANGE to register
uint32_t pot_timer; // reading pots too often causes noise
#define POT_SAMPLE_TIME 30 // delay time between pot reads
#define MIN_POT_CHANGE 25 // locked pot reading must change by this in order to register
#define MIN_COUNTS 8  // unlocked pot must change by this in order to register
#define POT_AVERAGING 20 // analog sample averaging count 
#define POT_MIN 4   // A/D may not read min value of 0 so use a bit larger value for map() function
#define POT_MAX 1019 // A/D may not read max value of 1023 so use a bit smaller value for map() function


// button handling / led handling

#define NUM_BUTTONS 9 // 8 buttons plus USR button on VCC-GND board
#define SHIFT 8 // index of "shift" USR button 
uint8_t debouncecnt[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // debounce counters
bool button[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // key active flags
int led[8] = {LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7};
int buttons[NUM_BUTTONS] = {BUTTON0, BUTTON1, BUTTON2, BUTTON3, BUTTON4, BUTTON5, BUTTON6, BUTTON7, SHIFT};

// flag all pot values as locked ie they have to change more than MIN_POT_CHANGE to register
void lockpots(void) {
  for (int i = 0; i < NPOTS; ++i) potlock[i] = 1;
}


// sample analog pot input and do filtering.
// if pots are locked, change value only if movement is greater than MIN_POT_CHANGE
uint16_t readpot(uint8_t potnum) {
  int val = 0;
  int input;
  switch (potnum) { // map potnum to RP2040 pin
    case 0:
      input = 26;
      break;
    case 1:
      input = 27;
      break;
    case 2:
    default:   // shouldn't happen
      input = INTS_PIN;
      break;
  }
  // note that Pikocore pots are wired "backwards" - max voltage is full ccw
  for (int j = 0; j < POT_AVERAGING; ++j) val += (1024 - analogRead(input)); // read the A/D a few times and average for a more stable value
  val = val / POT_AVERAGING;
  if (potlock[potnum]) {
    int delta = lastpotvalue[potnum] - val; // this needs to be done outside of the abs() function - see arduino abs() docs
    if (abs(delta) > MIN_POT_CHANGE) {
      potlock[potnum] = 0; // flag pot no longer locked
      potvalue[potnum] = lastpotvalue[potnum] = val; // save the new reading
    }
    else val = lastpotvalue[potnum];
  }
  else {
    if (abs(lastpotvalue[potnum] - val) > MIN_COUNTS) lastpotvalue[potnum] = val; // even if pot is unlocked, make sure pot has moved at least MIN_COUNT counts so values don't jump around
    else val = lastpotvalue[potnum];
    potvalue[potnum] = val; // pot is unlocked so save the reading
  }
  return val;
}

// scan buttons
bool scanbuttons(void)
{
  bool pressed;
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    switch (i) { // sample gate inputs
      case 0:
        pressed = !digitalRead(BUTTON0); // active low key inputs
        break;
      case 1:
        pressed = !digitalRead(BUTTON1);
        break;
      case 2:
        pressed = !digitalRead(BUTTON2);
        break;
      case 3:
        pressed = !digitalRead(BUTTON3);
        break;
      case 4:
        pressed = !digitalRead(BUTTON4);
        break;
      case 5:
        pressed = !digitalRead(BUTTON5);
        break;
      case 6:
        pressed = !digitalRead(BUTTON6);
        break;
      case 7:
        pressed = !digitalRead(BUTTON7);
        break;
      case 8:
        pressed = !digitalRead(SHIFTBUTTON);
        break;
    }

    if (pressed) {
      if (debouncecnt[i] <= 3) ++debouncecnt[i];
      if (debouncecnt[i] == 2) { // trigger on second sample of key active
        button[i] = 1;
      }
    }
    else {
      debouncecnt[i] = 0;
      button[i] = 0;
    }
  }
  if (pressed) return true;
  else return false;
}

// largely defunct defines
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

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

#define DISPLAY_TIME 2000 // time in ms to display numbers on LEDS
int32_t display_timer;
// show a number in binary on the LEDs
void display_value(int16_t value) {
  for (int i = 7; i >= 0; i--) { // NOPE + 1 can loop this way because port assignments are sequential
    digitalWrite(led[i], value & 1);
    value = value >> 1;
  }
  display_timer = millis();
}

// display functions
typedef struct {
  int x;
  int y;
  const char* str;
} pos_t;

//// {x,y} locations of play screen items
const int step_text_pos[] = { 0, 15, 16, 15, 32, 15, 48, 15, 64, 15, 80, 15, 96, 15, 112, 15 };
const pos_t bpm_text_pos    = {.x = 0,  .y = 15, .str = "bpm:%3d" };
const pos_t trans_text_pos  = {.x = 46, .y = 15, .str = "trs:%+2d" };
const pos_t seqno_text_pos  = {.x = 90, .y = 15, .str = "seq:%d" };
const pos_t seq_info_pos    = {.x = 0, .y = 35, .str = "" };
const pos_t mode_text_pos   = {.x = 0, .y = 55, .str = "" };
const pos_t play_text_pos   = {.x = 90, .y = 55, .str = "" };

const pos_t oct_text_offset = { .x = 3, .y = 10,  .str = "" };
const pos_t gate_bar_offset = { .x = 0, .y = -15, .str = "" };
const pos_t edit_text_offset = { .x = 3, .y = 22,  .str = "" };
const int gate_bar_width = 14;
const int gate_bar_height = 4;


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
  DAC.setBuffers(1, 16); // DMA buffers
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
  // fill buffer
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  //MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  //MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  //MIDI.begin(MIDI_CHANNEL_OMNI);


}

void updateControl() {
  //MIDI.read();

  //envelope.update();
  int p1 = readpot(0) ; //analogRead(MODR_PIN); // value is 0-4065
  int p2 = readpot(1); // analogRead(INTS_PIN); // value is 0-4065

  //int INTS_calibrated = kMapIntensity(INTS_value);

  float voct = pitch; // silly, but we're still using buttons to select freq
  float engine_in = engine_in;  // set by encoder_delta in mode 1

  harm_in = map(p1, 0, 4065, 0.0f, 1.0f); // IN(2);
  timbre_in = map(p2, 0, 4065, 0.0f, 1.0f); //IN(3);

  morph_in = 0.1f; // IN(4);
  float trig_in = trig_in; //IN(5);
  float level_in = level_in; //IN(6);

  float fm_mod = fm_mod; // IN(7);
  float timb_mod = timb_mod ; //IN(8);
  float morph_mod = morph_mod; //IN(9);
  float decay_in = decay_in; //IN(10);
  float lpgColor_in = lpgColor_in; // IN(11);


  // TODO: check setting pitch
  float pitch = voct; // fabs(voct);
  /*  CONSTRAIN(pitch, 0.f, 127.f);
    voices[0].patch.note = pitch;
  */

  /*
    int engine = int(engine_in);
    CONSTRAIN(engine, 0, 15);      // 16 engines
    voices[0].patch.engine = engine;
  */

  CONSTRAIN(harm_in, 0.0f, 1.0f);
  voices[0].patch.harmonics = map(p1, 0, 4065, 0.0f, 1.0f);

  CONSTRAIN(timbre_in, 0.0f, 1.0f);
  voices[0].patch.timbre = map(p2, 0, 4065, 0.0f, 1.0f);


  CONSTRAIN(morph_in, 0.0f, 1.0f);
  voices[0].patch.morph =  map(p1, 0, 4065, 0.0f, 1.0f);


  CONSTRAIN(fm_mod, -1.0f, 1.0f);
  voices[0].patch.frequency_modulation_amount = 0.0f;// fm_mod;

  CONSTRAIN(timb_mod, -1.0f, 1.0f);
  voices[0].patch.timbre_modulation_amount = 0.0f; // timb_mod;

  CONSTRAIN(morph_mod, -1.0f, 1.0f);
  voices[0].patch.morph_modulation_amount = 0.0f; // morph_mod;
  /*
    CONSTRAIN(decay_in, 0.0f, 1.0f);
    voices[0].patch.decay = decay_in;

    CONSTRAIN(lpgColor_in, 0.0f, 1.0f);
    voices[0].patch.lpg_colour = lpgColor_in;
  */

  /* we're faking triggers with button presses for now
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
  */

  if (level_in > 0.0f) {
    voices[0].modulations.level_patched = true;
    voices[0].modulations.level = level_in;
  }
  else
    voices[0].modulations.level_patched = false;

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
void renderSamples() {
  // called at the end of the timer render routing to generate more samples
  //voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);
}
bool canBufferAudioOutput() {
  if ( DAC.availableForWrite() > 32 ) {
    return true;
  }
  return false;
}

void loop() {
  // updateAudio();
  //delay(1);
  if ( counter > 1 ) {
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

  float trigger = randomDouble(0.01, .7); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
  float harmonics = randomDouble(0.1, .8); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
  float timbre = randomDouble(0.1, .8); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
  float morph = randomDouble(0.1, 0.8) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
  float pitch = abs(randomDouble(34, 60)); // TIRand.kr(24, 48, trigger);
  float octave = randomDouble(0.3, 0.5);
  float decay = randomDouble(0.1, 0.7);
  /*
    var sub = SinOsc.ar(pitch.midicps, 0, 0.1);
    var mi = MiPlaits.ar( pitch, engine, harmonics, timbre, morph,
      trigger: trigger, decay: 0.8, lpg_colour: 0.2, mul: 0.5);
    mi + sub
  */

  voices[0].patch.engine = engineCount;
  //voices[0].transposition_ = 0.;
  voices[0].octave_ = octave;
  voices[0].patch.note = pitch;
  voices[0].patch.harmonics = harmonics;
  voices[0].patch.morph = morph;
  voices[0].patch.timbre = timbre;
  voices[0].patch.decay = 0.5f;
  voices[0].patch.lpg_colour = 0.2;
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
  if (engineCount > 15) engineCount = 0;

  //updateControl();

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

      if ( !encoder_held ) {
        encoder_held = true;
        display_mode = display_mode + 1;
        if ( display_mode > 2) { // switched back to play mode
          display_mode = 0;
          //configure_sequencer();
        }
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

      // a track button is pressed
      current_track = i; // keypress selects track we are working on

      if ( encoder_delta == 0) {
        //if (pressedB != i) {
        // turn off the last note
        //aNoteOff(freqs[pressedB],0);
        //}
        aNoteOff(freqs[pressedB], 0);
        noteA = freqs[i];
        pitch = freqs[i];
        voices[0].patch.note = pitch; // 60.f + pitch * 12.f;
        aNoteOn( freqs[i], 100 );
      }
      pressedB = i;

      //  if ((!potlock[1]) || (!potlock[2])) seq[i].trigger=euclid(16,map(potvalue[1],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS),map(potvalue[2],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS-1));
      // look up drum trigger pattern encoder play modes

      if ( i == 2 && encoder_delta != 0 ) {
        attackIn += encoder_delta * 5;
        //envelope.setAttackTime(attackIn);
        //attackIn = constrain(attackIn, 0, 380);
      }
      if ( i == 3 && encoder_delta != 0 ) {
        durationIn += encoder_delta * 5;
        //envelope.setDecayTime(durationIn);
        //durationIn= constrain(durationIn, 16, 380);
        //pressedB = 1;
      }
      if ( i == 4 && encoder_delta != 0) {
        modIn += encoder_delta * 10;
        modIn = constrain(modIn, 10, 2800);
      }

      if ( (encoder_pos != encoder_pos_last ) && i == 5  ) {
        int engine = voices[0].patch.engine;
        voices[0].modulations.trigger_patched = false;
        voices[0].modulations.trigger = 0.0f;

        engine = engine + encoder_delta;
        if (engine > -1 && engine < 16) {
          engine_in = engine;
          voices[0].patch.engine = engine; // ( engine +) % voices[0].voice_.GetNumEngines();
        }
        //uint8_t re = seq[i].trigger->getRepeats() + encoder_delta;
        //seq[i].trigger->setRepeats(encoder_delta);
        //display_repeats = seq[i].trigger->getRepeats();

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
    RATE_value = RATE_value + encoder_delta;
    voices[0].patch.note = voices[0].patch.note * RATE_value;
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
  scanbuttons();
  displayUpdate();

  delay(2000);


}
// display functions
void displayUpdate() {
  display.clearDisplay();
  //display.setFont(&myfont); don't need to call this every time!
  //display.setTextColor(WHITE, 0);
  /*
    for (int i = 0; i < 8; i++) {
    Step s = seqr.steps[i];
    const char* nstr = notenum_to_notestr(s.note);
    int o = notenum_to_oct(s.note);
    int x = step_text_pos[i * 2], y = step_text_pos[i * 2 + 1];
    display.setCursor(x, y);
    display.print(nstr);
    display.setCursor(x + oct_text_offset.x, y + oct_text_offset.y);
    display.printf("%1d", o);
    display.setCursor(x + edit_text_offset.x, y + edit_text_offset.y);
    display.print((i == selected_step) ? '^' : (s.on) ? ' '
                                                      : '*');
    int gate_w = 1 + (s.gate * gate_bar_width / 16);
    display.fillRect(x + gate_bar_offset.x, y + gate_bar_offset.y, gate_w, gate_bar_height, WHITE);
    }
  */

  // display.setFont(&myfont2);


  /*
    const pos_t bpm_text_pos    = {.x=0,  .y=57, .str="bpm:%3d" };
    const pos_t trans_text_pos  = {.x=55, .y=57, .str="trs:%+2d" };
    const pos_t seqno_text_pos  = {.x=0,  .y=45, .str="seq:%d" };
    const pos_t seq_info_pos    = {.x=60, .y=45, .str="" };
    const pos_t play_text_pos   = {.x=110,.y=57, .str="" };
  */
  // bpm
  display.setCursor(bpm_text_pos.x, bpm_text_pos.y);
  display.print("r: ");
  display.print(voices[0].patch.engine);

  // transpose
  display.setCursor(trans_text_pos.x, trans_text_pos.y);
  display.print("p:");
  display.print(voices[0].patch.note);

  // seqno
  display.setCursor(seqno_text_pos.x, seqno_text_pos.y);
  display.print("h: ");
  display.print(voices[0].patch.harmonics);  // user sees 1-8

  // seq info / meta
  display.setCursor(seq_info_pos.x, seq_info_pos.y);
  display.print("m: ");
  display.print(voices[0].patch.morph);

  display.setCursor(mode_text_pos.x, play_text_pos.y);
  display.print("t: ");
  display.print(voices[0].patch.timbre);
  // play/pause
  display.setCursor(play_text_pos.x, play_text_pos.y);
  display.print("s: ");
  display.print(outputPlaits[0].out);

  display.display();
}

void displaySplash() {
  display.clearDisplay();
  display.setFont(&myfont);
  display.setTextColor(WHITE, 0);
  display.drawRect(0, 0, dw - 1, dh - 1, WHITE);
  display.setCursor(25, 32);
  display.print("PikoBeatBox");
  display.display();
  // a little LED dance
  /*
    for( int i=0; i<1000; i++) {
    for( int j=0; j<8; j++) {
      int v = 30 + 30 * sin( (j*6.2 / 8 ) + i/50.0 ) ;
      analogWrite( led_pins[j], v);
    }
    delay(1);
    }*/
}

void voicesNext(int inNumSamples)
{
}

// midi related

void HandleNoteOn(byte channel, byte note, byte velocity) {
  if (velocity == 0) {
    HandleNoteOff(channel, note, velocity);
    return;
  }
  //carrier_freq = mtof(note);
  //envelope.noteOn();
  digitalWrite(LED, HIGH);
}

void aNoteOn(float note, int velocity) {
  if (velocity == 0) {
    aNoteOff(note, velocity);

    return;
  }
  //voices[0].patch.note = pitch;

  voices[0].modulations.trigger_patched = true;
  voices[0].modulations.trigger = 5.f;


  carrier_freq = note;
  //envelope.noteOn();
  //digitalWrite(LED, HIGH);
}

void HandleNoteOff(byte channel, byte note, byte velocity) {
  //envelope.noteOff();
  //voices[0].modulations.trigger = 0.f;
  //voices[0].modulations.trigger_patched = false;
  //digitalWrite(LED, LOW);
}

void aNoteOff( float note, int velocity) {
  voices[0].modulations.trigger = 0.f;
  voices[0].modulations.trigger_patched = false;
  //envelope.noteOff();
  //digitalWrite(LED, LOW);
}
